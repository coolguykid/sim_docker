"""Replenishing regional execution used by the original orchestrator and UAVs.

This is a helper for the first-version application, not a replacement entry
point. Route servers/workers select paths; the existing authoritative control
core proposes safe motion; the original UAV class commits it atomically.
"""
from collections import defaultdict, deque
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
import uuid

from uav_sim.airspace.partition_graph import compile_partition, PartitionMaps
from uav_sim.algorithms.avoidance_features import collision_threshold_for_pipe
from uav_sim.algorithms.continuous_safety import audit_trajectories
from uav_sim.distributed.authoritative_control_server import AuthoritativeControlCore
from uav_sim.distributed.regional_backend import RegionalRoutingBackend
from uav_sim.entities.uav import UAV, get_uav_3d_pos
from uav_sim.guidance import GuidanceConfig, GuidanceModel, GuidanceNetwork
from uav_sim.utils.physical_clock import PhysicalClock
from .population import PopulationScheduler


class MetropolisSession:
    def __init__(self, env, config, graph=None):
        from .destination_pool import DestinationPool

        self.env = env
        self.config = dict(config)
        engine = self.config.get('execution_engine', 'certified_journey')
        if engine not in ('certified_journey','certified_prefix','per_step'):
            raise ValueError('Unknown execution_engine')
        guidance_config = GuidanceConfig.from_mapping(config.get("guidance", {}))
        if (guidance_config.enabled or guidance_config.evaluate) and engine != 'certified_journey':
            raise ValueError("guidance requires the certified_journey engine")
        self.clock = PhysicalClock(config.get("sim_dt_seconds", 1.0))
        self.target = int(config.get("num_uavs", 10000))
        self.scheduler = PopulationScheduler(
            self.target,
            int(config.get("max_batch_size", 1000)),
            float(config.get("replenishment_period_seconds", 100)),
            bool(config.get("replenishment_enabled", True)),
        )
        self.spawn_end = float(config.get("max_sim_time_seconds", 18000))
        self.drain_timeout = float(config.get("drain_timeout_seconds", 18000))
        self.clock.steps_for(self.spawn_end)
        self.clock.steps_for(self.drain_timeout)
        if config.get('frame_export_enabled') and config.get('run_output_dir'):
            from uav_sim.visualization.frame_exporter import FrameJsonExporter
            export_root = Path(config['run_output_dir'])
            export_root.mkdir(parents=True, exist_ok=True)
            FrameJsonExporter.check_disk_capacity(export_root, self.target,
                self.clock.steps_for(self.spawn_end+self.drain_timeout)+1)
        self.mission_limit = config.get("mission_limit")
        if self.mission_limit is not None and (
            isinstance(self.mission_limit, bool)
            or not isinstance(self.mission_limit, int)
            or self.mission_limit < 1
        ):
            raise ValueError("mission_limit must be a positive integer")
        manual = config.get("manual_destinations")
        if config.get("destination_mode", "random") == "manual" and not manual:
            manual = [
                dict(
                    pipe_id=pid, lane_id=0, distance_m=float(env.pipelines[pid].length)
                )
                for pid in config.get("dest_pipe_ids", [])
            ]
        from .mission_features import MissionFeatures

        self.features = MissionFeatures(env, config)
        self.pool = DestinationPool(
            env,
            int(config.get("destination_count", 10)),
            int(config.get("random_seed", 42)),
            config.get("destination_policy", "fixed"),
            manual=manual,
        )
        batch_count = max(
            1, int(math.ceil(self.spawn_end / (self.scheduler.period_ticks / 10)))
        )
        if self.mission_limit:
            batch_count = min(
                batch_count,
                int(math.ceil(self.mission_limit / self.scheduler.batch_limit)),
            )
        self.graph = graph or compile_partition(
            env,
            self.pool.all_specs(batch_count) + tuple(self.features.additional_specs),
        )
        self.maps = PartitionMaps(self.graph)
        self.guidance = (GuidanceModel(GuidanceNetwork(self.graph, guidance_config.capacity_scale), guidance_config)
                         if guidance_config.enabled or guidance_config.evaluate else None)
        self._current_guidance_maps = self.maps
        self._mission_cost_maps = {}
        pipe_regions, ramp_regions = {}, {}
        for edge in self.graph.edges.values():
            if edge.pipe_id is not None:
                pipe_regions.setdefault(edge.pipe_id, set()).add(edge.region_id)
            if edge.ramp_id is not None:
                ramp_regions.setdefault(edge.ramp_id, set()).add(edge.region_id)
        self.env.partition_scene = dict(
            regions=list(self.graph.regions.values()),
            pipe_regions={pid:sorted(regions) for pid,regions in pipe_regions.items()},
            ramp_regions={rid:sorted(regions) for rid,regions in ramp_regions.items()},
            goals=[dict(node_id=node_id, pipe_id=d.pipe_id, lane_id=d.lane_id,
                        distance_m=d.distance_m, position_m=d.position)
                   for node_id in self.graph.metadata.get('goal_specs', {})
                   for d in (self.maps.destination(node_id),)])
        self.routing_mode = config.get("routing_mode", "centralized_full")
        use_ray = bool(config.get("use_ray", self.routing_mode == "onboard_regional"))
        if self.routing_mode == "onboard_regional" and not use_ray:
            raise ValueError("Fully distributed mode requires real Ray workers")
        self.control = AuthoritativeControlCore.for_partition(
            self.graph,
            max_accel=float(config.get("max_accel", 10)),
            max_decel=float(config.get("max_decel", 10)),
        )
        if self.config.get('execution_engine', 'certified_journey') == 'certified_journey':
            attempts = config.get('reservation_search_attempts', 4096)
            shift = float(config.get('departure_search_quantum_seconds', .1))
            if isinstance(attempts, bool) or not isinstance(attempts, int) or attempts < 1:
                raise ValueError('reservation_search_attempts must be a positive integer')
            if not math.isfinite(shift) or shift < 0:
                raise ValueError('departure_search_quantum_seconds must be finite nonnegative seconds')
            self.control.prefix_reservations.max_attempts = attempts
            self.control.prefix_reservations.min_search_shift_seconds = shift
        self.slots = [
            (p.pipe_id, lane)
            for p in env.pipelines
            if p.height == 100
            for lane in range(p.num_lanes)
        ]
        if not self.slots:
            raise ValueError("No bottom-layer departure slots")
        self.active = {}
        self.pending = deque()
        self.pending_uavs = {}
        self.uid = 0
        self.generated = 0
        self.departed = 0
        self.arrived = 0
        self.no_route = 0
        self.peak_active = 0
        self.completed = []
        self.selected_trajectories = {
            int(x): [] for x in config.get("track_uav_ids", [])
        }
        self.track_ids = set(self.selected_trajectories)
        self.world_objects = {}
        self.batches = []
        self.audit_steps = 0
        self.retries = 0
        self.deferred_count = 0
        self.failed = ""
        self.last_arrival = None
        self.started = time.perf_counter()
        self._closed = False
        self.output = None
        self.arrival_stream = None
        self.exporter = None
        self.timing = None
        root = config.get("run_output_dir")
        self.run_id = (
            config.get("run_id")
            or time.strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
        )
        if root:
            self.output = Path(root)
            self.output.mkdir(parents=True, exist_ok=True)
            self.arrival_stream = (self.output / "arrivals.ndjson").open(
                "w", encoding="utf-8"
            )
            from uav_sim.utils.simulation_timing import SimulationTimingRecorder

            self.timing = SimulationTimingRecorder(
                self.output / "timing" / "turn_times.txt", sim_dt=self.clock.dt
            )
            if config.get("frame_export_enabled", False):
                from uav_sim.visualization.frame_exporter import FrameJsonExporter

                self.exporter = FrameJsonExporter(
                    self.output,
                    env,
                    self.run_id,
                    "1",
                    control_mode=self.routing_mode,
                    sim_dt=self.clock.dt,
                    expected_uav_count=self.target,
                    expected_frame_count=self.clock.steps_for(self.spawn_end+self.drain_timeout)+1,
                )
        self._initial_frame_written = False
        self.certified_traffic = None
        try:
            self.backend = RegionalRoutingBackend(self.maps, self.routing_mode,
                int(config.get('parallelism_level',300)),use_ray,self.target)
        except Exception as error:
            if self.arrival_stream:
                self.arrival_stream.close()
            if self.exporter:
                self.exporter.close(complete=False,error=str(error))
            if self.timing:
                self.timing.close()
            self.features.close()
            raise

    def _plan_many(self, backend, requests, event_time):
        context = self.clock.context(self.graph.version, event_time)
        if not self._mission_cost_maps:
            return backend.plan_many(requests, context)
        groups = defaultdict(dict)
        for uid, request in requests.items():
            groups[self._mission_cost_maps.get(uid, self.maps)][uid] = request
        plans = {}
        for maps, group in groups.items():
            plans.update(backend.plan_many(group, context, maps=maps))
        return plans

    def _sample_guidance(self, event_time):
        model = self.guidance
        observations = []
        for uav in self.active.values():
            state = uav.sample_partition_state(event_time)
            observations.append((model.network.cell(state.region_id, state.position[2]), state.goal.region_id))
        revision = model.revision
        model.observe(event_time, observations)
        if model.revision != revision:
            self._current_guidance_maps = (self.maps.with_edge_penalties(model.edge_penalties, model.revision)
                                           if model.edge_penalties else self.maps)

    def guidance_report(self):
        """Return the same sampled evaluation data exported in summary.json."""
        return self.guidance.report() if self.guidance is not None else None

    def _plan(self, state, event_time):
        result = self._plan_many(
            self.backend,
            {
                state.uav_id: dict(
                    start_node_id=state.node_id,
                    region_id=state.region_id,
                    goal_node_id=state.goal.node_id,
                )
            },
            event_time,
        )[state.uav_id]
        if result.status != "OK":
            return ()
        return result.edge_ids

    def _add_batch(self, count, event_time=None):
        event_time = self.clock.time_seconds if event_time is None else event_time
        batch = self.scheduler.batch_index - 1
        requests = {}
        added = []
        maps = self._current_guidance_maps
        for _ in range(count):
            uid = self.uid
            self.uid += 1
            pid, lane = self.slots[uid % len(self.slots)]
            spec = self.pool.for_mission(uid, batch)
            final_goal = self.maps.destination(spec["node_id"])
            goal = self.features.initial_goal(uid, final_goal, self.maps)
            u = UAV(
                uid,
                pid,
                lane,
                goal.pipe_id,
                self.env,
                capacity=self.features.capacity(uid),
            )
            start = self.graph.metadata["pipe_endpoints"][pid]["start"]
            u.configure_partition_motion(self.graph, start, goal, lane)
            self.features.configure(u, final_goal)
            u.requested_time_seconds = event_time
            self.pending.append(uid)
            self.pending_uavs[uid] = u
            if maps.cost_revision:
                self._mission_cost_maps[uid] = maps
            requests[uid] = dict(
                start_node_id=start,
                region_id=u.current_region_id,
                goal_node_id=goal.node_id,
            )
            added.append(uid)
        plans = (
            self._plan_many(self.backend, requests, event_time)
            if requests
            else {}
        )
        guided = 0
        for uid in added:
            u = self.pending_uavs[uid]
            plan = plans[uid]
            if plan.status != "OK":
                self.pending_uavs.pop(uid)
                self._mission_cost_maps.pop(uid, None)
                self.no_route += 1
            else:
                u.set_partition_route(plan.edge_ids)
                guided += int(maps.cost_revision != 0)
        if guided:
            self.guidance.record_guided_missions(guided)
        self.generated += count
        self.batches.append(
            dict(
                batch_index=batch,
                sim_time_seconds=event_time,
                generated=count,
                active_before=len(self.active),
            )
        )

    def _candidates(self):
        result = {}
        used = set()
        for uid in self.pending:
            u = self.pending_uavs.get(uid)
            if u is None:
                continue
            key = (u.start_pipe_id, u.start_lane_id)
            if key in used:
                continue
            used.add(key)
            result[uid] = u
            if len(result) >= min(len(self.slots), self.scheduler.batch_limit):
                break
        return result

    def step(self):
        if self.failed:
            raise RuntimeError(self.failed)
        now = self.clock.time_seconds
        dt = self.clock.dt
        start = time.perf_counter()
        count = 0
        if now < self.spawn_end and (
            self.mission_limit is None or self.generated < self.mission_limit
        ):
            count = self.scheduler.due(now, len(self.active), len(self.pending_uavs))
            if self.mission_limit:
                count = min(count, self.mission_limit - self.generated)
        if count:
            self._add_batch(count)
        self.env.sim_time_seconds = now
        if self.exporter and not self._initial_frame_written:
            self.exporter.write_uav_objects(
                0, list(self.active.values()) + list(self.pending_uavs.values())
            )
            self._initial_frame_written = True
        candidates = self._candidates()
        aircraft = dict(self.active)
        aircraft.update(candidates)
        try:
            proposal = self.control.compute_partition_proposals(
                aircraft, now, dt, self._plan, new_ids=set(candidates)
            )
            motions = proposal["proposals"]
            deferred = set(proposal.get("deferred_ids", ()))
            pieces = [p for motion in motions.values() for p in motion.pieces]
            thresholds = {
                uid: self._threshold(motion.state) for uid, motion in motions.items()
            }
            conflicts = audit_trajectories(pieces, thresholds)
            if conflicts:
                raise RuntimeError(
                    "Independent committed-interval safety audit rejected proposed motion: "
                    + str(conflicts[:3])
                )
        except Exception as error:
            self.failed = str(error) or type(error).__name__
            raise
        entering = set(candidates) & set(motions) - deferred
        self.peak_active = max(self.peak_active, len(self.active) + len(entering))
        for uid in entering:
            u = self.pending_uavs.pop(uid)
            u.spawn_time_seconds = now
            self.active[uid] = u
            self.departed += 1
        self.pending = deque(uid for uid in self.pending if uid in self.pending_uavs)
        self.deferred_count += len(deferred)
        self.retries += int(proposal.get("metrics", {}).get("backtrack_iterations", 0))
        completed = []
        for uid, motion in motions.items():
            if uid not in self.active:
                continue
            u = self.active[uid]
            u.commit_partition_step(motion)
            if uid in self.track_ids:
                self.selected_trajectories[uid].append(list(u._road_state.position))
            if u.state == "ARRIVED":
                s = u._road_state
                error = math.dist(s.position, s.goal.position)
                if s.node_id != s.goal.node_id or error > 1e-5:
                    raise RuntimeError("Incorrect physical arrival")
                record = dict(
                    uav_id=uid,
                    spawn_time_seconds=u.spawn_time_seconds,
                    arrival_time_seconds=s.arrival_time,
                    goal_node_id=s.goal.node_id,
                    actual_goal_node_id=s.node_id,
                    position_m=s.position,
                    goal_position_error_m=error,
                    visited_regions=s.visited_regions,
                    travel_seconds=s.arrival_time - u.spawn_time_seconds,
                )
                self.completed.append(record)
                if self.arrival_stream:
                    self.arrival_stream.write(
                        json.dumps(record, ensure_ascii=False) + "\n"
                    )
                self.arrived += 1
                self.last_arrival = max(self.last_arrival or 0, s.arrival_time)
                if uid in self.track_ids:
                    self.world_objects[uid] = u
                completed.append(uid)
        for uid in completed:
            self.active.pop(uid)
        if completed:
            self.backend.discard(completed)
        self.audit_steps += 1
        self.clock.advance()
        self.env.sim_time_seconds = self.clock.time_seconds
        if len(self.active) + len(self.pending_uavs) > self.target:
            raise RuntimeError("Population target exceeded")
        if self.generated != self.arrived + self.no_route + len(self.active) + len(
            self.pending_uavs
        ):
            raise RuntimeError("Population conservation failure")
        if self.timing:
            self.timing.record_turn(
                self.clock.step_index,
                time.perf_counter() - start,
                0,
                time.perf_counter() - start,
                len(self.active),
                self.arrived,
            )
        if self.exporter:
            self.exporter.write_uav_objects(
                self.clock.step_index,
                list(self.active.values()) + list(self.pending_uavs.values()),
            )
        return self.summary()

    def _threshold(self, state):
        from uav_sim.algorithms.avoidance_features import collision_threshold_for_height

        edge = self.graph.edges.get(state.edge_id)
        if edge and edge.ramp_id is not None:
            return max(
                collision_threshold_for_height(int(math.ceil(p[2] / 100)) * 100)
                for p in edge.points
            )
        node = self.graph.nodes[state.node_id]
        if node.pipe_id is not None:
            return collision_threshold_for_pipe(self.env.pipelines[node.pipe_id])
        return collision_threshold_for_height(
            int(math.ceil(state.position[2] / 100)) * 100
        )

    def run(self, on_step=None):
        if self.config.get('execution_engine','certified_journey') in ('certified_prefix','certified_journey'):
            self._ensure_certified_traffic()
            return self.certified_traffic.run(on_step)
        deadline = self.spawn_end + self.drain_timeout
        while self.clock.time_seconds < deadline - 1e-9:
            exhausted = (
                self.mission_limit is not None and self.generated >= self.mission_limit
            )
            stopped = (
                exhausted
                or self.clock.time_seconds >= self.spawn_end
                or (not self.scheduler.enabled and self.generated > 0)
            )
            if stopped and not self.active and not self.pending_uavs:
                break
            result = self.step()
            if on_step:
                on_step(result)
        return self.summary()

    def _ensure_certified_traffic(self):
        if self.certified_traffic is None:
            if self.config.get('execution_engine','certified_journey')=='certified_journey':
                from .certified_journey import CertifiedJourney
                self.certified_traffic=CertifiedJourney(self)
            else:
                from .certified_traffic import CertifiedTraffic
                self.certified_traffic = CertifiedTraffic(self)

    def advance_one_step(self):
        """Engine bridge: return after exactly one 0.1/1-second output slot."""
        if self.config.get('execution_engine', 'certified_journey') == 'per_step':
            raise ValueError('per_step cannot be mixed with certified live stepping')
        self._ensure_certified_traffic()
        return self.certified_traffic.advance_interval()

    def current_frame(self, include_pending=False, uav_ids=None):
        """Return exact meter/second states at the current output timestamp."""
        from uav_sim.visualization.state_adapter import partition_update

        objects = dict(self.active)
        if include_pending:
            objects.update(self.pending_uavs)
        if uav_ids is not None:
            selected = set(uav_ids)
            objects = {uid: u for uid, u in objects.items() if uid in selected}
        rows = []
        for uid, uav in sorted(objects.items()):
            row = partition_update(uav, self.env, self.clock.time_seconds)
            row["uav_id"] = uid
            rows.append(row)
        return dict(
            schema_version="uav_sim.live.v1",
            unit="meter",
            sim_time_seconds=self.clock.time_seconds,
            sim_dt_seconds=self.clock.dt,
            step_index=self.clock.step_index,
            topology_version=self.graph.version,
            uavs=rows,
        )

    def on_partition_goal(self, uav, state, event_time):
        """Return True for a final goal; retained pickup support may add a leg."""
        return self.features.on_goal(uav, state, event_time)

    def summary(self):
        try:
            routing_metrics = self.backend.metrics()
        except Exception as error:
            routing_metrics = dict(metrics_error=str(error) or type(error).__name__)
            self.failed = self.failed or 'Routing metrics unavailable: '+routing_metrics['metrics_error']
        travel = [r["travel_seconds"] for r in self.completed]
        ordered = sorted(travel)
        result = dict(
            run_id=self.run_id,
            routing_mode=self.routing_mode,
            sim_dt_seconds=self.clock.dt,
            generated=self.generated,
            departed=self.departed,
            arrived=self.arrived,
            no_route=self.no_route,
            active=len(self.active),
            pending=len(self.pending_uavs),
            peak_active=self.peak_active,
            unfinished=self.generated - self.arrived,
            safety_violations=0 if not self.failed else None,
            hard_collisions=0 if not self.failed else None,
            audited_steps=self.audit_steps,
            backtrack_iterations=self.retries,
            deferred_departure_checks=self.deferred_count,
            sim_time_seconds=self.clock.time_seconds,
            last_arrival_time_seconds=self.last_arrival,
            mean_travel_seconds=sum(travel) / len(travel) if travel else None,
            p95_travel_seconds=ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))]
            if ordered
            else None,
            mean_ground_wait_seconds=sum(
                r.get("ground_wait_seconds", 0.0) for r in self.completed
            )
            / len(travel)
            if travel
            else None,
            mean_airborne_hold_seconds=sum(
                r.get("airborne_hold_seconds", 0.0) for r in self.completed
            )
            / len(travel)
            if travel
            else None,
            max_goal_position_error_m=max(
                (r["goal_position_error_m"] for r in self.completed), default=None
            ),
            wall_seconds=time.perf_counter() - self.started,
            failed=self.failed,
            success=self.generated > 0
            and self.arrived == self.generated
            and not self.failed,
            routing=routing_metrics,
            destinations=self.pool.manifest(),
        )
        if self.certified_traffic:
            result.update(self.certified_traffic.metrics())
        else:
            result["execution_engine"] = "per_step"
        if self.guidance is not None:
            result["guidance"] = self.guidance_report()
        return result

    def close(self):
        if self._closed:
            return
        try:
            summary = self.summary()
            if self.arrival_stream:
                self.arrival_stream.close()
            if self.exporter:
                self.exporter.close(complete=not bool(self.failed),error=self.failed or None)
            if self.timing:
                self.timing.close(algorithm_total_seconds=time.perf_counter()-self.started)
            self.features.close()
            if self.output:
                (self.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
        finally:
            try:
                self.backend.close()
            finally:
                self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, exception_type, error, traceback):
        if error is not None:
            self.failed = str(error) or type(error).__name__
        self.close()
