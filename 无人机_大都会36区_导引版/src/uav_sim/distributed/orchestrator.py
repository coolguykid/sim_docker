"""
orchestrator.py — V6.0 分布式仿真编排器
管理全局时钟、Barrier 同步、碰撞检测、结果聚合。
替代 simulation.py 中的串行 run_simulation() 循环。
"""
try:
    import ray
except ModuleNotFoundError:
    ray = None
import time
import numpy as np
from uav_sim.algorithms.avoidance_features import collision_threshold_for_pipe


class SimulationOrchestrator:
    @classmethod
    def for_metropolis(cls,env,config,graph=None):
        """Use the original coordinator with its enhanced population loop."""
        instance=cls.__new__(cls)
        instance._metropolis_env=env;instance._metropolis_config=dict(config);instance._metropolis_graph=graph
        return instance

    def _run_metropolis_loop(self):
        from uav_sim.simulation.metropolis_runtime import MetropolisSession
        with MetropolisSession(self._metropolis_env,self._metropolis_config,self._metropolis_graph) as session:
            summary=session.run()
            self.metropolis_summary=summary
            self.metropolis_uavs=list(session.world_objects.values())+list(session.active.values())
            kl=session.features.kl_history
            return (summary['last_arrival_time_seconds'] or -1,summary['safety_violations'],kl,
                    summary['unfinished'],sum(kl)/len(kl) if kl else 0.,session.features.gain_history,session.selected_trajectories,
                    dict(hard_collisions=summary['hard_collisions'],metropolis_summary=summary,
                         _tracked_uav_objects={u.uav_id:u for u in self.metropolis_uavs}))

    def __init__(
        self,
        env,
        uav_groups,
        registry,
        total_uav_count,
        mode="DIJKSTRA",
        max_turns=3500,
        track_uav_ids=None,
        pickup_points=None,
        route_server=None,
        control_mode="legacy_local",
        authoritative_control_server=None,
        server_avoidance_policy="rule",
        safety_substep=0.2,
        trace_uav_ids=None,
        trace_log_path=None,
        trace_algo_id=None,
        ramp_state_provider=None,
        ramp_poll_interval_turns=1,
        ramp_poll_region_id=None,
        enable_kl_divergence=False,
        enable_channel_gain=False,
        run_output_dir=None,
        frame_export_enabled=False,
        run_id=None,
        algorithm_id=None,
        initial_route_planning_seconds=0.0,
    ):
        if mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        self.env = env
        self.uav_groups = uav_groups
        self.registry = registry
        self.route_server = route_server
        self.authoritative_control_server = authoritative_control_server
        self.control_mode = control_mode
        self.server_authoritative = control_mode == "server_authoritative"
        self.centralized_control = control_mode in (
            "centralized_route",
            "centralized_full",
            "server_authoritative",
        )
        self.server_avoidance_policy = server_avoidance_policy
        self.safety_substep = safety_substep
        self.trace_uav_ids = list(trace_uav_ids or [])
        self.trace_log_path = trace_log_path
        self.trace_algo_id = trace_algo_id or mode
        self._trace_logger = None
        self.total_uav_count = total_uav_count
        self.mode = mode
        self.max_turns = max_turns
        self.track_uav_ids = set(track_uav_ids) if track_uav_ids else set()
        self.pickup_points = list(pickup_points or [])
        self._pending_route_ref = None
        self.enable_kl_divergence = bool(enable_kl_divergence)
        self.enable_channel_gain = bool(enable_channel_gain)
        self.run_output_dir = run_output_dir
        self.frame_export_enabled = bool(frame_export_enabled)
        self.run_id = str(run_id or "run")
        self.algorithm_id = str(algorithm_id or self.trace_algo_id)
        self.initial_route_planning_seconds = float(initial_route_planning_seconds)
        self._timing_recorder = None
        self._frame_exporter = None
        self._algorithm_started_at = None
        self.ramp_poll_interval_turns = max(1, int(ramp_poll_interval_turns))
        self._ramp_state_poller = None
        if ramp_state_provider is not None:
            from uav_sim.distributed.ramp_state_sync import RampStatePoller

            self._ramp_state_poller = RampStatePoller(
                ramp_state_provider, region_id=ramp_poll_region_id
            )

    def run(self):
        """主仿真循环 — 每个 tick 通过 Barrier 同步所有 Actor"""
        if hasattr(self,'_metropolis_config'):return self._run_metropolis_loop()
        if ray is None:
            raise RuntimeError("Ray 未安装，无法运行分布式仿真；请安装 ray 或关闭 Ray 分布式模式。")
        result = None
        error = None
        try:
            self._open_run_outputs()
            if self.server_authoritative:
                result = self._run_server_authoritative_loop()
            else:
                result = self._run_distributed_loop()
            return result
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            timing_summary = self._close_run_outputs(error=error)
            if result is not None and len(result) >= 8 and (timing_summary is not None):
                result[7]["timing"] = timing_summary

    def _run_distributed_loop(self):
        print(
            f"\n======== 开始【{self.mode}】分布式仿真 | {self.total_uav_count} 架 | {len(self.uav_groups)} 个 Actor ========"
        )
        dynamic_ramps = [r for r in self.env.ramps if r.is_dynamic]
        ray.get(
            self.registry.init_ramp_throughput.remote(
                [r.ramp_id for r in dynamic_ramps]
            )
        )
        for pt in self.pickup_points:
            pt.reset()
        total_safety_violations = 0
        total_hard_collisions = 0
        arrived_uavs = set()
        last_arrival_turn = -1
        kl_history = []
        avg_gain_history = []
        previous_turn_violations = set()
        previous_hard_collisions = set()
        uav_trajectories = {uid: [] for uid in self.track_uav_ids}
        load_balance_metrics = {}
        self._open_trace_logger()
        for turn in range(self.max_turns):
            turn_started_at = time.perf_counter()
            self._poll_external_ramp_states(turn)
            pipe_uavs_map = ray.get(self.registry.get_pipe_uavs_map.remote())
            shared_state = ray.get(
                self.registry.get_shared_context.remote(current_turn=turn)
            )
            collisions_mult = shared_state["collisions_multiplier"]
            traffic_load = shared_state["traffic_load"]
            ramp_states = shared_state["ramp_states"]
            self._record_optional_metrics(
                pipe_uavs_map,
                None,
                kl_history,
                avg_gain_history,
                ramp_states=ramp_states,
            )
            active_count = sum((len(v) for v in pipe_uavs_map.values()))
            if active_count == 0 and len(arrived_uavs) == self.total_uav_count:
                break
            global_context = {
                "pipe_uavs_map": pipe_uavs_map,
                "collisions_multiplier": collisions_mult,
                "traffic_load": traffic_load,
                "ramp_states": ramp_states,
                "topology_version": shared_state.get("topology_version", 0),
                "ramp_state_version": shared_state.get("ramp_state_version", 0),
                "traffic_cost_version": shared_state.get("traffic_cost_version", 0),
                "track_uav_ids": self.track_uav_ids,
            }
            if self.centralized_control:
                self._service_route_promises(turn, global_context)
            if self.centralized_control:
                local_contexts = self._build_local_contexts(global_context)
                tick_futures = [
                    group.tick.remote(turn, self.mode, local_contexts[idx])
                    for idx, group in enumerate(self.uav_groups)
                ]
            else:
                tick_futures = [
                    group.tick.remote(turn, self.mode, global_context)
                    for group in self.uav_groups
                ]
            tick_results = ray.get(tick_futures)
            all_uav_updates = {}
            all_traffic = {}
            all_ramp_throughput = {}
            all_arrivals = []
            all_pickup_updates = {}
            for result in tick_results:
                all_uav_updates.update(result["uav_updates"])
                all_arrivals.extend(result["arrivals"])
                for pid, c in result["traffic_counts"].items():
                    all_traffic[pid] = all_traffic.get(pid, 0) + c
                for rid, c in result["ramp_throughput"].items():
                    all_ramp_throughput[rid] = all_ramp_throughput.get(rid, 0) + c
                for pt_id, amount in result.get("pickup_updates", {}).items():
                    all_pickup_updates[pt_id] = (
                        all_pickup_updates.get(pt_id, 0) + amount
                    )
                for uid, pos_3d in result.get("trajectories", {}).items():
                    if uid in uav_trajectories:
                        arr = np.array(pos_3d)
                        if not uav_trajectories[uid] or not np.allclose(
                            uav_trajectories[uid][-1], arr
                        ):
                            uav_trajectories[uid].append(arr)
            update_futures = [
                self.registry.batch_update_uav_positions.remote(all_uav_updates),
                self.registry.batch_update_traffic.remote(all_traffic),
            ]
            if all_ramp_throughput:
                update_futures.append(
                    self.registry.update_ramp_throughput.remote(all_ramp_throughput)
                )
            ray.get(update_futures)
            if all_pickup_updates:
                self._apply_pickup_updates(all_pickup_updates)
            self._log_trace(turn, all_uav_updates)
            self._record_optional_metrics(
                None, all_uav_updates, kl_history, avg_gain_history
            )
            new_pipe_uavs_map = {}
            for uid, info in all_uav_updates.items():
                if info.get("state") == "FLYING":
                    pid = info["pipe_id"]
                    new_pipe_uavs_map.setdefault(pid, []).append(
                        {"uav_id": uid, **info}
                    )
            (
                violations,
                speed_adjustments,
                collision_pipe_counts,
                hard_collisions,
            ) = self._detect_collisions(new_pipe_uavs_map, previous_turn_violations)
            total_safety_violations += len(violations - previous_turn_violations)
            total_hard_collisions += len(hard_collisions - previous_hard_collisions)
            if collision_pipe_counts:
                ray.get(
                    self.registry.batch_update_collisions_window.remote(
                        collision_pipe_counts
                    )
                )
            if speed_adjustments:
                adjust_futures = []
                group_adjustments = self._partition_adjustments_by_group(
                    speed_adjustments
                )
                for gid, adj in group_adjustments.items():
                    adjust_futures.append(
                        self.uav_groups[gid].apply_speed_adjustments.remote(adj)
                    )
                if adjust_futures:
                    ray.get(adjust_futures)
            previous_turn_violations = violations
            previous_hard_collisions = hard_collisions
            for uid in all_arrivals:
                arrived_uavs.add(uid)
                last_arrival_turn = turn
            turn_should_stop = (
                not any(
                    (info.get("state") == "FLYING" for info in all_uav_updates.values())
                )
                and len(arrived_uavs) == self.total_uav_count
            )
            if turn > 0 and turn % 40 == 0:
                pass
            if turn > 0 and turn % 100 == 0:
                print(
                    f"  [{self.mode}] 已推演至第 {turn} 回合... (已到达: {len(arrived_uavs)}/{self.total_uav_count})"
                )
                ray.get(
                    [
                        self.registry.reset_traffic_history.remote(),
                        self.registry.reset_ramp_throughput.remote(),
                    ]
                )
            self._finalize_turn_output(
                turn,
                turn_started_at,
                all_uav_updates,
                active_uavs=sum(
                    (
                        1
                        for info in all_uav_updates.values()
                        if info.get("state") in ("FLYING", "SWITCHING")
                    )
                ),
                arrived_uavs=len(arrived_uavs),
            )
            if turn_should_stop:
                break
        effective_turns = last_arrival_turn if last_arrival_turn > 0 else turn + 1
        avg_kl = (
            sum(kl_history[:effective_turns]) / effective_turns
            if effective_turns > 0
            else 0.0
        )
        self._close_trace_logger()
        metrics = {
            "load_balance": load_balance_metrics,
            "hard_collisions": total_hard_collisions,
        }
        return (
            last_arrival_turn,
            total_safety_violations,
            kl_history,
            self.total_uav_count - len(arrived_uavs),
            avg_kl,
            avg_gain_history,
            uav_trajectories,
            metrics,
        )

    def _run_server_authoritative_loop(self):
        print(
            f"\n======== 开始【{self.mode}】服务器权威分布式仿真 | {self.total_uav_count} 架 | {len(self.uav_groups)} 个 Actor | 避障={self.server_avoidance_policy} ========"
        )
        if self.authoritative_control_server is None:
            raise RuntimeError("server_authoritative 模式需要 AuthoritativeControlServer。")
        dynamic_ramps = [r for r in self.env.ramps if r.is_dynamic]
        ray.get(
            self.registry.init_ramp_throughput.remote(
                [r.ramp_id for r in dynamic_ramps]
            )
        )
        for pt in self.pickup_points:
            pt.reset()
        total_safety_violations = 0
        total_hard_collisions = 0
        arrived_uavs = set()
        last_arrival_turn = -1
        kl_history = []
        avg_gain_history = []
        previous_turn_violations = set()
        previous_hard_collisions = set()
        uav_trajectories = {uid: [] for uid in self.track_uav_ids}
        load_balance_metrics = {}
        safety_metrics = {
            "safety_shield_miss": 0,
            "safety_modified_count": 0,
            "hold_count": 0,
            "hard_brake_count": 0,
        }
        self._open_trace_logger()
        for turn in range(self.max_turns):
            turn_started_at = time.perf_counter()
            self._poll_external_ramp_states(turn)
            observation_parts = ray.get(
                [
                    group.report_control_observations.remote(turn=turn)
                    for group in self.uav_groups
                ]
            )
            observations = self._concat_observations(observation_parts)
            current_map = self._pipe_map_from_observations(observations)
            shared_context = ray.get(
                self.registry.get_shared_context.remote(current_turn=turn)
            )
            self._record_optional_metrics(
                current_map,
                None,
                kl_history,
                avg_gain_history,
                ramp_states=shared_context.get("ramp_states", {}),
            )
            active_count = sum((len(v) for v in current_map.values()))
            if active_count == 0 and len(arrived_uavs) == self.total_uav_count:
                break
            traffic_load = shared_context["traffic_load"]
            control_result = ray.get(
                self.authoritative_control_server.compute_commands.remote(
                    turn, observations, shared_context
                )
            )
            commands = control_result.get("commands")
            route_updates = control_result.get("route_updates", [])
            control_metrics = control_result.get("metrics", {})
            safety_metrics["safety_modified_count"] += control_metrics.get(
                "safety_modified_count", 0
            )
            safety_metrics["hold_count"] += control_metrics.get("hold_count", 0)
            safety_metrics["hard_brake_count"] += control_metrics.get(
                "hard_brake_count", 0
            )
            actor_payloads = self._partition_authoritative_payloads(
                commands, route_updates
            )
            exec_futures = []
            for gid, payload in actor_payloads.items():
                exec_futures.append(
                    self.uav_groups[gid].apply_server_commands.remote(
                        payload["commands"],
                        route_updates=payload["route_updates"],
                        turn=turn,
                        track_uav_ids=self.track_uav_ids,
                        ramp_states=shared_context.get("ramp_states", {}),
                    )
                )
            tick_results = ray.get(exec_futures) if exec_futures else []
            all_uav_updates = {}
            all_traffic = {}
            all_ramp_throughput = {}
            all_arrivals = []
            all_pickup_updates = {}
            for result in tick_results:
                all_uav_updates.update(result["uav_updates"])
                all_arrivals.extend(result["arrivals"])
                for pid, c in result["traffic_counts"].items():
                    all_traffic[pid] = all_traffic.get(pid, 0) + c
                for rid, c in result["ramp_throughput"].items():
                    all_ramp_throughput[rid] = all_ramp_throughput.get(rid, 0) + c
                for pt_id, amount in result.get("pickup_updates", {}).items():
                    all_pickup_updates[pt_id] = (
                        all_pickup_updates.get(pt_id, 0) + amount
                    )
                for uid, pos_3d in result.get("trajectories", {}).items():
                    if uid in uav_trajectories:
                        arr = np.array(pos_3d)
                        if not uav_trajectories[uid] or not np.allclose(
                            uav_trajectories[uid][-1], arr
                        ):
                            uav_trajectories[uid].append(arr)
            update_futures = [
                self.registry.batch_update_uav_positions.remote(all_uav_updates),
                self.registry.batch_update_traffic.remote(all_traffic),
            ]
            if all_ramp_throughput:
                update_futures.append(
                    self.registry.update_ramp_throughput.remote(all_ramp_throughput)
                )
            ray.get(update_futures)
            if all_pickup_updates:
                self._apply_pickup_updates(all_pickup_updates)
            self._log_trace(turn, all_uav_updates)
            self._record_optional_metrics(
                None, all_uav_updates, kl_history, avg_gain_history
            )
            new_pipe_uavs_map = {}
            for uid, info in all_uav_updates.items():
                if info.get("state") == "FLYING":
                    pid = info["pipe_id"]
                    new_pipe_uavs_map.setdefault(pid, []).append(
                        {"uav_id": uid, **info}
                    )
            (
                violations,
                _speed_adjustments,
                collision_pipe_counts,
                hard_collisions,
            ) = self._detect_collisions(new_pipe_uavs_map, previous_turn_violations)
            new_violations = violations - previous_turn_violations
            total_safety_violations += len(new_violations)
            safety_metrics["safety_shield_miss"] += len(new_violations)
            total_hard_collisions += len(hard_collisions - previous_hard_collisions)
            if collision_pipe_counts:
                ray.get(
                    self.registry.batch_update_collisions_window.remote(
                        collision_pipe_counts
                    )
                )
            previous_turn_violations = violations
            previous_hard_collisions = hard_collisions
            for uid in all_arrivals:
                arrived_uavs.add(uid)
                last_arrival_turn = turn
            turn_should_stop = (
                not any(
                    (info.get("state") == "FLYING" for info in all_uav_updates.values())
                )
                and len(arrived_uavs) == self.total_uav_count
            )
            if turn > 0 and turn % 40 == 0:
                pass
            if turn > 0 and turn % 100 == 0:
                print(
                    f"  [{self.mode}|server] 已推演至第 {turn} 回合... (已到达: {len(arrived_uavs)}/{self.total_uav_count})"
                )
                ray.get(
                    [
                        self.registry.reset_traffic_history.remote(),
                        self.registry.reset_ramp_throughput.remote(),
                    ]
                )
            self._finalize_turn_output(
                turn,
                turn_started_at,
                all_uav_updates,
                active_uavs=sum(
                    (
                        1
                        for info in all_uav_updates.values()
                        if info.get("state") in ("FLYING", "SWITCHING")
                    )
                ),
                arrived_uavs=len(arrived_uavs),
            )
            if turn_should_stop:
                break
        effective_turns = last_arrival_turn if last_arrival_turn > 0 else turn + 1
        avg_kl = (
            sum(kl_history[:effective_turns]) / effective_turns
            if effective_turns > 0
            else 0.0
        )
        self._close_trace_logger()
        metrics = {
            "load_balance": load_balance_metrics,
            "safety": safety_metrics,
            "hard_collisions": total_hard_collisions,
        }
        return (
            last_arrival_turn,
            total_safety_violations,
            kl_history,
            self.total_uav_count - len(arrived_uavs),
            avg_kl,
            avg_gain_history,
            uav_trajectories,
            metrics,
        )

    def _service_route_promises(self, turn, global_context):
        if self.route_server is None:
            return
        if self._pending_route_ref is not None:
            ready, pending = ray.wait([self._pending_route_ref], timeout=0)
            if ready:
                route_updates = ray.get(ready[0])
                self._apply_route_updates(route_updates)
                self._pending_route_ref = None
                del route_updates
            else:
                self._pending_route_ref = pending[0]
        if self._pending_route_ref is not None:
            return
        request_lists = ray.get(
            [
                group.build_route_requests.remote(turn=turn, force_all=False)
                for group in self.uav_groups
            ]
        )
        route_requests = []
        for group_requests in request_lists:
            route_requests.extend(group_requests or [])
        if not route_requests:
            return
        route_context = {
            "collisions_multiplier": global_context.get("collisions_multiplier", {}),
            "traffic_load": global_context.get("traffic_load", {}),
            "ramp_states": global_context.get("ramp_states", {}),
            "topology_version": global_context.get("topology_version", 0),
            "ramp_state_version": global_context.get("ramp_state_version", 0),
            "traffic_cost_version": global_context.get("traffic_cost_version", 0),
        }
        self._pending_route_ref = self.route_server.plan_routes.remote(
            route_requests, route_context
        )
        del request_lists, route_requests, route_context

    def _poll_external_ramp_states(self, turn):
        if self._ramp_state_poller is None:
            return None
        if int(turn) % self.ramp_poll_interval_turns != 0:
            return None
        update = self._ramp_state_poller.poll(turn)
        if update is None:
            return None
        result = ray.get(
            self.registry.apply_external_ramp_states.remote(
                update.states,
                source_version=update.source_version,
                region_id=update.region_id,
                effective_from_turn=update.effective_from_turn,
                effective_until_turn=update.effective_until_turn,
                current_turn=turn,
            )
        )
        if result.get("accepted"):
            self._ramp_state_poller.acknowledge(update.source_version)
        return result

    def _open_run_outputs(self):
        if not self.run_output_dir:
            return
        from pathlib import Path
        from uav_sim.utils.simulation_timing import SimulationTimingRecorder

        output_dir = Path(self.run_output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        self._algorithm_started_at = time.perf_counter()
        self._timing_recorder = SimulationTimingRecorder(
            output_dir / "timing" / "turn_times.txt", sim_dt=1.0
        )
        if self.frame_export_enabled:
            from uav_sim.visualization.frame_exporter import FrameJsonExporter

            self._frame_exporter = FrameJsonExporter(
                output_dir,
                self.env,
                run_id=self.run_id,
                algorithm_id=self.algorithm_id,
                routing_mode=self.mode,
                control_mode=self.control_mode,
                expected_uav_count=self.total_uav_count,
                expected_frame_count=self.max_turns,
            )

    def _finalize_turn_output(
        self, turn, turn_started_at, uav_updates, active_uavs, arrived_uavs
    ):
        if self._timing_recorder is None and self._frame_exporter is None:
            return
        compute_seconds = time.perf_counter() - turn_started_at
        export_seconds = 0.0
        if self._frame_exporter is not None:
            export_started_at = time.perf_counter()
            self._frame_exporter.write_frame(turn, uav_updates)
            export_seconds = time.perf_counter() - export_started_at
        total_seconds = time.perf_counter() - turn_started_at
        if self._timing_recorder is not None:
            self._timing_recorder.record_turn(
                turn=turn,
                compute_seconds=compute_seconds,
                export_seconds=export_seconds,
                total_seconds=total_seconds,
                active_uavs=active_uavs,
                arrived_uavs=arrived_uavs,
            )

    def _close_run_outputs(self, error=None):
        if self._frame_exporter is not None:
            self._frame_exporter.close(complete=error is None, error=error)
        if self._timing_recorder is None:
            return None
        algorithm_total_seconds = (
            time.perf_counter()
            - self._algorithm_started_at
            + self.initial_route_planning_seconds
            if self._algorithm_started_at is not None
            else None
        )
        summary = self._timing_recorder.close(
            initial_route_planning_seconds=self.initial_route_planning_seconds,
            algorithm_total_seconds=algorithm_total_seconds,
        )
        print(
            f"[Timing] 平均每回合≈{summary.average_turn_seconds:.3f}s | 仿真循环≈{summary.simulation_loop_seconds:.3f}s | 算法总时间≈{summary.algorithm_total_seconds:.3f}s | 实时倍率≈{summary.real_time_factor:.3f}x",
            flush=True,
        )
        return summary.to_dict()

    def _apply_route_updates(self, route_updates):
        if not route_updates:
            return
        grouped = {}
        for update in route_updates:
            uid = update.get("uav_id") if isinstance(update, dict) else update.uav_id
            gid = self._uid_to_group_id(uid)
            grouped.setdefault(gid, []).append(update)
        futures = [
            self.uav_groups[gid].apply_route_updates.remote(updates)
            for gid, updates in grouped.items()
        ]
        if futures:
            ray.get(futures)

    def _build_local_contexts(self, global_context):
        window_lists = ray.get(
            [
                group.get_local_query_windows.remote(radius=2000.0)
                for group in self.uav_groups
            ]
        )
        local_maps = ray.get(
            [
                self.registry.get_geo_fenced_pipe_uavs_map.remote(windows)
                for windows in window_lists
            ]
        )
        contexts = []
        for local_map in local_maps:
            ctx = dict(global_context)
            ctx["pipe_uavs_map"] = local_map
            contexts.append(ctx)
        return contexts

    @staticmethod
    def _concat_observations(observation_parts):
        parts = [
            part for part in observation_parts if part is not None and len(part) > 0
        ]
        if not parts:
            from uav_sim.distributed.server_control_messages import (
                make_observation_array,
            )

            return make_observation_array([])
        return np.concatenate(parts)

    @staticmethod
    def _pipe_map_from_observations(observations):
        pipe_map = {}
        for obs in observations:
            if int(obs["state_code"]) != 1:
                continue
            pid = int(obs["pipe_id"])
            pipe_map.setdefault(pid, []).append(
                {
                    "uav_id": int(obs["uav_id"]),
                    "region_id": int(obs["region_id"])
                    if "region_id" in observations.dtype.names
                    else 0,
                    "pipe_id": pid,
                    "lane_id": int(obs["lane_id"]),
                    "pos_1d": float(obs["pos_1d"]),
                    "speed": float(obs["speed"]),
                    "state": "FLYING",
                    "current_ramp_id": int(obs["current_ramp_id"])
                    if int(obs["current_ramp_id"]) >= 0
                    else None,
                    "gaussian_intensity": float(obs["gaussian_intensity"]),
                }
            )
        return pipe_map

    def _partition_authoritative_payloads(self, commands, route_updates):
        payloads = {
            gid: {"commands": [], "route_updates": []}
            for gid in range(len(self.uav_groups))
        }
        for gid in payloads:
            if commands is not None and len(commands) > 0:
                mask = np.array(
                    [
                        self._uid_to_group_id(int(cmd["uav_id"])) == gid
                        for cmd in commands
                    ],
                    dtype=bool,
                )
                payloads[gid]["commands"] = commands[mask]
            else:
                payloads[gid]["commands"] = commands
        for update in route_updates or []:
            uid = update.get("uav_id") if isinstance(update, dict) else update.uav_id
            gid = self._uid_to_group_id(uid)
            payloads[gid]["route_updates"].append(update)
        return payloads

    def _open_trace_logger(self):
        if self.trace_uav_ids and self.trace_log_path and (self._trace_logger is None):
            from uav_sim.utils.trace_logger import TurnTraceLogger

            self._trace_logger = TurnTraceLogger(
                self.trace_uav_ids, self.trace_log_path
            )

    def _close_trace_logger(self):
        if self._trace_logger is not None:
            self._trace_logger.close()
            self._trace_logger = None

    def _log_trace(self, turn, uav_updates):
        if self._trace_logger is not None:
            self._trace_logger.log_turn(
                self.trace_algo_id, self.mode, turn, uav_updates, env=self.env
            )

    def _detect_collisions(self, pipe_uavs_map, previous_violations):
        violations = set()
        hard_collisions = set()
        speed_adjustments = {}
        collision_pipe_counts = {}
        for p_id, uav_infos in pipe_uavs_map.items():
            if len(uav_infos) < 2:
                continue
            col_th = collision_threshold_for_pipe(self.env.pipelines[p_id])
            lane_groups = {}
            for info in uav_infos:
                lane_groups.setdefault(info["lane_id"], []).append(info)
            for lane_id, uavs_in_lane in lane_groups.items():
                if len(uavs_in_lane) < 2:
                    continue
                uavs_in_lane.sort(key=lambda x: x["pos_1d"])
                for i in range(len(uavs_in_lane) - 1):
                    follower = uavs_in_lane[i]
                    leader = uavs_in_lane[i + 1]
                    dist = leader["pos_1d"] - follower["pos_1d"]
                    pair = (follower["uav_id"], leader["uav_id"])
                    if dist <= 1e-06:
                        hard_collisions.add(pair)
                    is_violating = dist < col_th or (
                        pair in previous_violations and dist < col_th * 1.5
                    )
                    if is_violating:
                        violations.add(pair)
                        if pair not in previous_violations:
                            collision_pipe_counts[p_id] = (
                                collision_pipe_counts.get(p_id, 0) + 1
                            )
                        speed_adjustments[follower["uav_id"]] = min(
                            follower["speed"], leader["speed"]
                        )
        return (violations, speed_adjustments, collision_pipe_counts, hard_collisions)

    def _partition_adjustments_by_group(self, adjustments):
        group_adj = {}
        for uid, speed in adjustments.items():
            gid = self._uid_to_group_id(uid)
            group_adj.setdefault(gid, {})[uid] = speed
        return group_adj

    def _uid_to_group_id(self, uid):
        n_groups = len(self.uav_groups)
        return uid % n_groups

    def _apply_pickup_updates(self, pickup_updates):
        pt_by_id = {pt.pt_id: pt for pt in self.pickup_points}
        for pt_id, amount in pickup_updates.items():
            pt = pt_by_id.get(pt_id)
            if pt is not None:
                pt.remaining = max(0, pt.remaining - amount)

    def _compute_kl_from_uav_updates(self, uav_updates, sigma=None):
        if sigma is None:
            sigma = max(500.0, getattr(self.env, "xy_span", 5000.0) / 10.0)
        xs = np.linspace(self.env.x_min, self.env.x_max, 10)
        ys = np.linspace(self.env.y_min, self.env.y_max, 10)
        zs = np.array([100, 200, 300, 400, 500, 600])
        grid_x, grid_y, grid_z = np.meshgrid(xs, ys, zs, indexing="ij")
        grid_pts = np.stack(
            [grid_x.flatten(), grid_y.flatten(), grid_z.flatten()], axis=1
        )
        P = np.zeros(len(grid_pts))
        Q = np.zeros(len(grid_pts))
        for pt in self.pickup_points:
            if pt.remaining > 0:
                pipe = self.env.pipelines[pt.pipe_id]
                pt_3d = pipe.start_pt + pipe.direction * pt.pos_1d
                dist_sq = np.sum((grid_pts - pt_3d) ** 2, axis=1)
                P += pt.remaining * np.exp(-dist_sq / (2 * sigma**2))
        for info in uav_updates.values():
            if info.get("state") in ["ARRIVED", "WAITING"]:
                continue
            intensity = info.get("gaussian_intensity", 0.0)
            if intensity <= 0:
                continue
            pipe_id = info.get("pipe_id")
            if pipe_id is None:
                continue
            pipe = self.env.pipelines[pipe_id]
            pos_3d = pipe.start_pt + pipe.direction * info.get("pos_1d", 0.0)
            dist_sq = np.sum((grid_pts - pos_3d) ** 2, axis=1)
            Q += intensity * np.exp(-dist_sq / (2 * sigma**2))
        P_sum = np.sum(P)
        Q_sum = np.sum(Q)
        if P_sum < 1e-06:
            return 0.0
        P_norm = P / P_sum
        Q_norm = Q / Q_sum if Q_sum > 1e-06 else np.ones_like(Q) / Q.size
        eps = 1e-09
        P_norm = np.clip(P_norm, eps, 1.0)
        Q_norm = np.clip(Q_norm, eps, 1.0)
        return np.sum(P_norm * np.log(P_norm / Q_norm))

    def _compute_kl_from_registry(self, turn):
        states_futures = [g.get_uav_states.remote() for g in self.uav_groups]
        all_states_list = ray.get(states_futures)
        all_states = {}
        for states in all_states_list:
            all_states.update(states)
        return self._compute_kl_from_uav_updates(all_states)

    def _compute_beamforming_gain(self, pipe_uavs_map, ramp_states=None):
        if ramp_states is None:
            active_ramps_count = sum(
                (
                    1
                    for ramp in self.env.ramps
                    if not getattr(ramp, "is_dynamic", False) or ramp.is_active
                )
            )
        else:
            active_ramps_count = sum(
                (
                    1
                    for ramp in self.env.ramps
                    if bool(ramp_states.get(int(ramp.ramp_id), True))
                )
            )
        if active_ramps_count == 0:
            return 0.0
        return 40.0 * 8 * 8 / active_ramps_count

    def _record_optional_metrics(
        self, pipe_uavs_map, uav_updates, kl_history, avg_gain_history, ramp_states=None
    ):
        if self.enable_channel_gain and pipe_uavs_map is not None:
            if ramp_states is None:
                gain = self._compute_beamforming_gain(pipe_uavs_map)
            else:
                gain = self._compute_beamforming_gain(
                    pipe_uavs_map, ramp_states=ramp_states
                )
            avg_gain_history.append(gain)
        if self.enable_kl_divergence and uav_updates is not None:
            kl_history.append(self._compute_kl_from_uav_updates(uav_updates))
