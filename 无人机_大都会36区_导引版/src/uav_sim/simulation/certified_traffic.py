"""Exact event execution of continuously certified regional flight profiles.

The output clock remains a fixed 0.1/1 second grid. Motion between output
instants is a set of analytic acceleration phases, not a second hidden time
step. Each phase and the stationary wait following it is checked before it is
installed. An unchanged certificate is reused while sampling that phase.
"""
from collections import defaultdict, deque
from dataclasses import replace
import heapq
import itertools
import json
import math
import time

from uav_sim.entities.partition_motion import PreparedMotion
from uav_sim.entities.trajectory_profile import build_prefix_profile


class CertifiedTraffic:
    def __init__(self, session):
        self.session = session
        self.reservations = session.control.prefix_reservations
        self.events = []
        self.sequence = itertools.count()
        self.remaining = {}
        self.prepared_candidates = {}
        self.profiles = {}
        self.waiting = {}
        self.dependents = defaultdict(set)
        self.ready = deque()
        self.ready_set = set()
        self.processing_ready = False
        self.prefix_count = 0
        self.event_count = 0
        self.retries = 0
        self.max_wait = 0.0
        self.total_wait = 0.0
        self.partial_prefixes = 0
        self.profile_builds = 0
        self.motion_seconds = defaultdict(float)
        self.peak_reached = False
        self.last_event_time = 0.0
        self.started = False
        self.progress_callback = None
        self._push(0.0, "REPLENISH", None)
        if session.guidance is not None:
            self._push(0.0, "GUIDANCE", 0)

    def _push(self, when, kind, uid):
        heapq.heappush(self.events, (when, next(self.sequence), kind, uid))

    def _uav(self, uid):
        s = self.session
        return s.active.get(uid) or s.pending_uavs.get(uid)

    def _enqueue(self, uid):
        if self._uav(uid) is not None and uid not in self.ready_set:
            self.ready.append(uid)
            self.ready_set.add(uid)

    def _wake(self, uid):
        for dependent in self.dependents.pop(uid, ()):
            self._enqueue(dependent)

    def _clear_wait(self, uid):
        for blocker in self.waiting.pop(uid, ()):
            self.dependents[blocker].discard(uid)

    def _prefix(self, uid):
        s = self.session
        u = self._uav(uid)
        edges = self.remaining.get(uid)
        if edges is None:
            edges = tuple(u._road_state.route[u._road_state.route_index :])
        prefix = []
        for edge_id in edges:
            prefix.append(edge_id)
            if s.graph.edges[edge_id].kind == "BOUNDARY":
                break
        self.remaining[uid] = edges
        return tuple(prefix)

    def _plan_ready(self, ready, now):
        s = self.session
        requests = {}
        for uid in ready:
            u = self._uav(uid)
            if u is None or uid in self.profiles:
                continue
            if uid in self.remaining and self.remaining[uid]:
                continue
            state = u._road_state
            requests[uid] = dict(
                start_node_id=state.node_id,
                region_id=state.region_id,
                goal_node_id=state.goal.node_id,
            )
        if requests:
            plans = s.backend.plan_many(requests, s.clock.context(s.graph.version, now))
            for uid, plan in plans.items():
                if plan.status != "OK":
                    raise RuntimeError(
                        "No reachable regional continuation for airborne/requested UAV %s: %s"
                        % (uid, plan.reason)
                    )
                self.remaining[uid] = tuple(plan.edge_ids)

    def _offer(self, uid, now):
        s = self.session
        u = self._uav(uid)
        if u is None or uid in self.profiles:
            return
        self._clear_wait(uid)
        prefix = self._prefix(uid)
        if not prefix and u._road_state.node_id != u._road_state.goal.node_id:
            raise RuntimeError("An empty prefix cannot advance UAV %s" % uid)
        cold = uid in s.pending_uavs
        signature = (id(u._road_state), prefix, s.graph.version)
        previous = self.prepared_candidates.get(uid)
        if previous is None or previous[0] != signature:
            previous = (signature, {})
            self.prepared_candidates[uid] = previous
        candidates_by_limit = previous[1]

        def prepare(limit=None):
            candidate = candidates_by_limit.get(limit)
            if candidate is None:
                options = {} if limit is None else dict(distance_limit_m=limit)
                value = build_prefix_profile(
                    u._road_state,
                    s.graph,
                    prefix,
                    start_time=0.0,
                    max_accel=s.config.get("max_accel", 10.0),
                    max_decel=s.config.get("max_decel", 10.0),
                    stop_at_end=not s.features.is_final_leg(u),
                    **options,
                )
                thresholds = tuple(
                    s._threshold(value.sample(p.t0)) for p in value.pieces
                )
                candidate = (value, thresholds)
                candidates_by_limit[limit] = candidate
                self.profile_builds += 1
            value, thresholds = candidate
            certificate = self.reservations.reserve(
                uid,
                value.pieces,
                now,
                start_position=u._road_state.position,
                end_position=value.end_state.position,
                end_hold=value.end_state.status != "ARRIVED"
                or not s.features.is_final_leg(u),
                cold=cold,
                thresholds=thresholds,
            )
            return value, certificate

        profile, result = prepare()
        if not result.accepted:
            # A future endpoint hold must not lock an entire five-kilometre
            # region. Fly a shorter certified prefix and stop physically in
            # its interior. Detailed route ownership remains in this region.
            distance = sum(
                math.dist(p.position0, p.position_at(p.t1)) for p in profile.pieces
            )
            margin = max(10.0, 2 * s._threshold(u._road_state))
            candidates = [distance - margin] + [
                distance * f for f in (0.9, 0.75, 0.5, 0.25, 0.1)
            ]
            blocked = set(result.blocked_by)
            for limit in candidates:
                if limit < margin or limit >= distance:
                    continue
                shorter, trial = prepare(limit)
                if trial.accepted:
                    profile, result = shorter, trial
                    self.partial_prefixes += 1
                    break
                blocked.update(trial.blocked_by)
            if not result.accepted:
                result = replace(result, blocked_by=tuple(sorted(blocked)))
        if not result.accepted:
            s.deferred_count += 1
            self.waiting[uid] = result.blocked_by
            for blocker in result.blocked_by:
                self.dependents[blocker].add(uid)
            return
        self.prefix_count += 1
        self.motion_seconds[uid] += profile.duration
        self.prepared_candidates.pop(uid, None)
        self.max_wait = max(self.max_wait, result.shift_seconds)
        self.total_wait += result.shift_seconds
        retimed = profile.shifted(result.start_time)
        self.profiles[uid] = retimed
        u._prefix_profile = retimed
        self.remaining[uid] = self.remaining[uid][profile.end_state.route_index :]
        if cold:
            self._push(result.start_time, "DEPART", uid)
        self._push(result.end_time, "FINISH", uid)
        # Replacing an infinite endpoint hold can immediately release waiting
        # aircraft, even when the replacement flight departs a little later.
        self._wake(uid)

    def _flush_ready(self, now):
        if self.processing_ready:
            return
        self.processing_ready = True
        try:
            # Each wake-up corresponds to an actual change in a certificate.
            # Failed requests simply retain occupancy until a blocker changes.
            while self.ready:
                batch = list(self.ready)
                self.ready.clear()
                self.ready_set.difference_update(batch)
                self._plan_ready(batch, now)
                for index,uid in enumerate(batch,1):
                    self.retries += 1
                    self._offer(uid, now)
                    interval=max(1,int(self.session.config.get('admission_progress_interval',50)))
                    if (self.session.config.get('report_admission_progress',False) and self.progress_callback
                            and (index%interval==0 or index==len(batch))):
                        progress=self.progress()
                        progress.update(stage='admission',admission_completed=index,admission_batch_size=len(batch))
                        self.progress_callback(progress)
        finally:
            self.processing_ready = False

    def _depart(self, uid, now):
        s = self.session
        u = s.pending_uavs.pop(uid, None)
        if u is None:
            return
        u.spawn_time_seconds = now
        u.state = "FLYING"
        s.active[uid] = u
        s.departed += 1
        s.peak_active = max(s.peak_active, len(s.active))
        if s.peak_active >= s.target:
            self.peak_reached = True

    def _arrive(self, uid, u, state, now):
        s = self.session
        error = math.dist(state.position, state.goal.position)
        if (
            state.node_id != state.goal.node_id
            or state.lane_id != state.goal.lane_id
            or error > 1e-5
        ):
            raise RuntimeError("Incorrect physical arrival for UAV %s" % uid)
        record = dict(
            uav_id=uid,
            spawn_time_seconds=u.spawn_time_seconds,
            arrival_time_seconds=state.arrival_time,
            goal_node_id=state.goal.node_id,
            actual_goal_node_id=state.node_id,
            position_m=state.position,
            goal_position_error_m=error,
            visited_regions=state.visited_regions,
            travel_seconds=state.arrival_time - u.spawn_time_seconds,
            requested_time_seconds=getattr(
                u, "requested_time_seconds", u.spawn_time_seconds
            ),
            ground_wait_seconds=u.spawn_time_seconds
            - getattr(u, "requested_time_seconds", u.spawn_time_seconds),
            moving_profile_seconds=self.motion_seconds.pop(uid, 0.0),
        )
        record["airborne_hold_seconds"] = max(
            0.0, record["travel_seconds"] - record["moving_profile_seconds"]
        )
        if s.guidance is not None:
            record["guidance_cost_revision"] = s._mission_cost_maps.get(uid, s.maps).cost_revision
        s.completed.append(record)
        if s.arrival_stream:
            s.arrival_stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        s.arrived += 1
        s.last_arrival = max(s.last_arrival or 0.0, state.arrival_time)
        if uid in s.track_ids or uid in s.features.trace.uav_ids:
            s.world_objects[uid] = u
        if uid in s.track_ids:
            s.selected_trajectories[uid].append(list(state.position))
        s.active.pop(uid)
        self.remaining.pop(uid, None)
        self.prepared_candidates.pop(uid, None)
        self.reservations.release(uid, now)
        self._wake(uid)

    def _finish(self, uid, now):
        s = self.session
        u = s.active.get(uid)
        if u is None:
            raise RuntimeError("A flight profile completed before its departure")
        profile = self.profiles.pop(uid)
        state = profile.end_state
        u.commit_partition_step(PreparedMotion(state, (), ()))
        del u._prefix_profile
        if state.status == "ARRIVED":
            if s.on_partition_goal(u, state, now):
                self._arrive(uid, u, state, now)
                return True
            # Optional retained pickup handling installed a new physical leg.
            self.remaining.pop(uid, None)
        self._enqueue(uid)
        return False

    def _can_spawn(self, now):
        s = self.session
        return (
            now < s.spawn_end
            and (s.mission_limit is None or s.generated < s.mission_limit)
            and not (
                s.config.get("stop_after_peak_active", False) and self.peak_reached
            )
        )

    def _replenish(self, now):
        s = self.session
        if self._can_spawn(now):
            before = set(s.pending_uavs)
            count = s.scheduler.due(now, len(s.active), len(s.pending_uavs))
            if s.mission_limit:
                count = min(count, s.mission_limit - s.generated)
            if count:
                s._add_batch(count, event_time=now)
            for uid in s.pending_uavs.keys() - before:
                self.remaining[uid] = tuple(s.pending_uavs[uid]._road_state.route)
                self._enqueue(uid)
        next_time = s.scheduler.next_tick / 10.0
        if s.scheduler.enabled and next_time > now and self._can_spawn(next_time):
            self._push(next_time, "REPLENISH", None)

    def _events_until(self, ending):
        s = self.session
        while self.events and self.events[0][0] <= ending:
            now = self.events[0][0]
            self.last_event_time = now
            s.env.sim_time_seconds = now
            self.reservations.expire(now)
            batch = []
            while self.events and self.events[0][0] == now:
                batch.append(heapq.heappop(self.events))
            # All physical departures count before any same-time finishes;
            # the recorded peak is an actual simultaneous population.
            for _, _, kind, uid in batch:
                if kind == "DEPART":
                    self._depart(uid, now)
            self._extra_events(batch, now)
            arrived = []
            for _, _, kind, uid in batch:
                self.event_count += 1
                if kind == "FINISH" and self._finish(uid, now):
                    arrived.append(uid)
            if arrived:
                s.backend.discard(arrived)
            next_sample = None
            for _, _, kind, index in batch:
                if kind == "GUIDANCE":
                    s._sample_guidance(now)
                    next_sample = index + 1
            for _, _, kind, uid in batch:
                if kind == "REPLENISH":
                    self._replenish(now)
            self._flush_ready(now)
            if next_sample is not None and self.events:
                self._push(next_sample * s.guidance.config.sample_interval_seconds, "GUIDANCE", next_sample)
            if not s.active and not s.pending_uavs and len(self.events) == 1 and self.events[0][2] == "GUIDANCE":
                heapq.heappop(self.events)
            self._check_population()

    def _extra_events(self,batch,now):
        """Optional exact-time region or service events in a longer certificate."""
        return None

    def _check_population(self):
        s = self.session
        if len(s.active) + len(s.pending_uavs) > s.target:
            raise RuntimeError("Population target exceeded")
        if s.generated != s.arrived + s.no_route + len(s.active) + len(s.pending_uavs):
            raise RuntimeError("Population conservation failure")

    def progress(self):
        s = self.session
        return dict(
            routing_mode=s.routing_mode,
            sim_dt_seconds=s.clock.dt,
            generated=s.generated,
            departed=s.departed,
            arrived=s.arrived,
            active=len(s.active),
            pending=len(s.pending_uavs),
            peak_active=s.peak_active,
            sim_time_seconds=s.clock.time_seconds,
            wall_seconds=time.perf_counter() - s.started,
            certified_prefixes=self.prefix_count,
            failed=s.failed,
        )

    def start(self):
        s = self.session
        if self.started:
            return
        self.started = True
        self._events_until(0.0)
        if s.exporter:
            s.exporter.write_uav_objects(
                0, list(s.active.values()) + list(s.pending_uavs.values())
            )
            s._initial_frame_written = True

    def advance_interval(self):
        """Advance exactly one configured output slot, keeping sub-slot events."""
        s = self.session
        try:
            self.start()
            if s.failed:
                return self.progress()
            if not self.events:
                if s.active or s.pending_uavs:
                    s.failed = (
                        "No executable event remains: %s airborne and %s ground requests are blocked"
                        % (len(s.active), len(s.pending_uavs))
                    )
                return self.progress()
            started = time.perf_counter()
            ending = (s.clock.step_index + 1) * s.clock.dt
            self._events_until(ending)
            s.clock.advance()
            s.env.sim_time_seconds = s.clock.time_seconds
            s.audit_steps += 1
            for uid in s.track_ids & s.active.keys():
                s.selected_trajectories[uid].append(
                    list(s.active[uid].sample_partition_state(ending).position)
                )
            s.features.record_grid(s)
            if s.exporter:
                s.exporter.write_uav_objects(
                    s.clock.step_index,
                    list(s.active.values()) + list(s.pending_uavs.values()),
                )
            if s.timing:
                total = time.perf_counter() - started
                s.timing.record_turn(
                    s.clock.step_index, total, 0.0, total, len(s.active), s.arrived
                )
        except Exception as error:
            s.failed = str(error) or type(error).__name__
            raise
        return self.progress()

    def run(self, on_step=None):
        s = self.session
        self.progress_callback=on_step
        deadline = s.spawn_end + s.drain_timeout
        try:
            self.start()
        except Exception as error:
            s.failed = str(error) or type(error).__name__
            raise
        while s.clock.time_seconds < deadline - 1e-9 and not s.failed:
            if not self.events and not s.active and not s.pending_uavs:
                break
            result = self.advance_interval()
            if on_step:
                on_step(result)
        if s.active or s.pending_uavs:
            s.failed = (
                s.failed
                or "Simulation drain timeout: unfinished physical flights remain"
            )
        return s.summary()

    def metrics(self):
        values = self.reservations.metrics()
        return dict(
            execution_engine="certified_prefix",
            certified_prefixes=self.prefix_count,
            partial_queue_prefixes=self.partial_prefixes,
            candidate_profile_builds=self.profile_builds,
            exact_motion_events=self.event_count,
            reservation_retries=self.retries,
            continuous_phase_pairs_checked=values["phase_pairs_checked"],
            mean_prefix_wait_seconds=self.total_wait / self.prefix_count
            if self.prefix_count
            else 0.0,
            max_prefix_wait_seconds=self.max_wait,
            blocked_requests=len(self.waiting),
            reservations=values,
        )
