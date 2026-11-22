"""
simulation.py — 仿真引擎核心（run_simulation）
"""
import numpy as np
import time
from uav_sim.algorithms.kl_divergence import compute_kl_divergence
from uav_sim.entities.uav import get_lead_info
from uav_sim.algorithms.avoidance_features import (
    collision_threshold_for_pipe,
    find_safe_lanes,
)
from uav_sim.algorithms.avoidance_actions import desired_speed_for_action


def run_simulation(
    env,
    uavs,
    pickup_points,
    mode="DIJKSTRA",
    max_turns=3500,
    track_uav_ids=None,
    trace_uav_ids=None,
    trace_log_path=None,
    trace_algo_id=None,
    enable_kl_divergence=False,
    enable_channel_gain=False,
    run_output_dir=None,
    frame_export_enabled=False,
    run_id=None,
    algorithm_id=None,
    initial_route_planning_seconds=0.0,
    metropolis_config=None,
):
    if metropolis_config is not None:
        from uav_sim.distributed.orchestrator import SimulationOrchestrator
        return SimulationOrchestrator.for_metropolis(env,metropolis_config).run()
    if mode != "DIJKSTRA":
        raise ValueError("Only DIJKSTRA routing is available")
    print(f"\n======== 开始【{mode}】模式下的 {len(uavs)} 架仿真演练 ========")
    algorithm_started_at = time.perf_counter()
    timing_recorder = None
    frame_exporter = None
    if run_output_dir:
        from pathlib import Path
        from uav_sim.utils.simulation_timing import SimulationTimingRecorder

        output_dir = Path(run_output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        timing_recorder = SimulationTimingRecorder(
            output_dir / "timing" / "turn_times.txt", sim_dt=1.0
        )
        if frame_export_enabled:
            from uav_sim.visualization.frame_exporter import FrameJsonExporter

            frame_exporter = FrameJsonExporter(
                output_dir,
                env,
                run_id=run_id or "run",
                algorithm_id=algorithm_id or trace_algo_id or mode,
                routing_mode=mode,
                control_mode="sequential",
                expected_uav_count=len(uavs),
                expected_frame_count=max_turns,
            )
    env.routing_mode = mode
    dynamic_ramps = [r for r in env.ramps if r.is_dynamic]
    for r in dynamic_ramps:
        r.is_active = False
    pipe_traffic_history = {p.pipe_id: 0 for p in env.pipelines}
    ramp_throughput_history = {r.ramp_id: 0 for r in dynamic_ramps}
    if hasattr(env, "pipe_collisions_window"):
        env.pipe_collisions_window = {p.pipe_id: 0 for p in env.pipelines}
        env.pipe_collisions_multiplier = {p.pipe_id: 1.0 for p in env.pipelines}
    if hasattr(env, "pipe_traffic_load"):
        env.pipe_traffic_load = {p.pipe_id: 0 for p in env.pipelines}
    for pt in pickup_points:
        pt.reset()
    for u in uavs:
        u.reset()
    initial_planning_order = list(uavs)
    for u in initial_planning_order:
        u.plan()
    total_safety_violations = 0
    total_hard_collisions = 0
    arrived_uavs = set()
    last_arrival_turn = -1
    kl_history = []
    avg_gain_history = []
    previous_turn_violations = set()
    previous_hard_collisions = set()
    trace_logger = None
    if trace_uav_ids and trace_log_path:
        from uav_sim.utils.trace_logger import TurnTraceLogger

        trace_logger = TurnTraceLogger(trace_uav_ids, trace_log_path)
    uav_trajectories = {uid: [] for uid in track_uav_ids} if track_uav_ids else {}
    load_balance_metrics = {}
    for turn in range(max_turns):
        turn_started_at = time.perf_counter()
        for u in uavs:
            if u.state == "WAITING" and turn >= u.spawn_turn:
                u.state = "FLYING"
        if enable_kl_divergence:
            kl_history.append(compute_kl_divergence(uavs, pickup_points, env))
        if enable_channel_gain:
            active_ramps_count = sum(
                (
                    1
                    for ramp in env.ramps
                    if not getattr(ramp, "is_dynamic", False) or ramp.is_active
                )
            )
            avg_gain = (
                40.0 * 8 * 8 / active_ramps_count if active_ramps_count > 0 else 0.0
            )
            avg_gain_history.append(avg_gain)
        active_uavs = [u for u in uavs if u.state == "FLYING"]
        if not active_uavs and len(arrived_uavs) == len(uavs):
            break
        for u in active_uavs:
            pipe_traffic_history[u.current_pipe_id] += 1
        collisions_changed_this_turn = False
        current_turn_violations = set()
        pipe_uavs_map = {}
        for u in active_uavs:
            u.avoidance_controlled_this_turn = False
            pipe_uavs_map.setdefault(u.current_pipe_id, []).append(u)
        for p_id, uavs_in_pipe in pipe_uavs_map.items():
            pipe = env.pipelines[p_id]
            pipe_max_speed = pipe.max_speed
            sense_range = pipe_max_speed * 5.0
            col_th = collision_threshold_for_pipe(pipe)
            for u in uavs_in_pipe:
                d_lead, lead_speed = get_lead_info(
                    p_id,
                    u.current_lane_id,
                    u.pos_1d,
                    uavs_in_pipe,
                    sense_range,
                    pipe_max_speed,
                )
                current_speed_limit = (
                    min(pipe_max_speed, lead_speed)
                    if d_lead < col_th * 15
                    else pipe_max_speed
                )
                if d_lead < sense_range:
                    u.avoidance_controlled_this_turn = True
                    safe_lanes = find_safe_lanes(
                        u, uavs_in_pipe, env.pipelines[p_id].num_lanes, sense_range
                    )
                    if safe_lanes:
                        u.current_lane_id = min(
                            safe_lanes, key=lambda x: abs(x - u.current_lane_id)
                        )
                        desired = desired_speed_for_action(
                            2,
                            u.speed,
                            pipe_max_speed,
                            u.max_accel,
                            u.max_decel,
                            current_speed_limit=current_speed_limit,
                        )
                        u.apply_speed_command(desired)
                    else:
                        desired = desired_speed_for_action(
                            4,
                            u.speed,
                            pipe_max_speed,
                            u.max_accel,
                            u.max_decel,
                            current_speed_limit=current_speed_limit,
                        )
                        u.apply_speed_command(desired)
                else:
                    action = 1
                    desired = desired_speed_for_action(
                        action,
                        u.speed,
                        pipe_max_speed,
                        u.max_accel,
                        u.max_decel,
                        current_speed_limit=current_speed_limit,
                    )
                    u.apply_speed_command(desired)
                safe_speed_limit = max(0.0, d_lead - col_th * 1.2 + lead_speed)
                if u.speed > safe_speed_limit:
                    u.apply_speed_command(safe_speed_limit)
        new_lanes_map = {}
        for u in active_uavs:
            new_lanes_map.setdefault((u.current_pipe_id, u.current_lane_id), []).append(
                u
            )
        current_hard_collisions = set()
        for (p_id, l_id), uav_list in new_lanes_map.items():
            uav_list.sort(key=lambda x: x.pos_1d)
            col_th = collision_threshold_for_pipe(env.pipelines[p_id])
            for i in range(len(uav_list) - 1):
                follower, leader = (uav_list[i], uav_list[i + 1])
                dist = leader.pos_1d - follower.pos_1d
                pair = (follower.uav_id, leader.uav_id)
                if dist <= 1e-06:
                    current_hard_collisions.add(pair)
                is_violating = dist < col_th or (
                    pair in previous_turn_violations and dist < col_th * 1.5
                )
                if is_violating:
                    current_turn_violations.add(pair)
                    if pair not in previous_turn_violations:
                        total_safety_violations += 1
                    if follower.speed > leader.speed:
                        follower.apply_speed_command(leader.speed)
        uav_step_order = list(uavs)
        for u in uav_step_order:
            if u.state not in ["ARRIVED", "WAITING"]:
                needs_replan = (
                    not u.avoidance_controlled_this_turn and u.was_avoidance_controlled
                )
                if needs_replan:
                    u.replan_path()
                u.was_avoidance_controlled = u.avoidance_controlled_this_turn
                old_state = u.state
                u.step(active_uavs)
                if (
                    old_state == "FLYING"
                    and u.state == "SWITCHING"
                    and (u.path_index < len(u.path))
                ):
                    ramp_id = env._extract_ramp_id_from_node(u.path[u.path_index])
                    if (
                        ramp_id is not None
                        and 0 <= ramp_id < len(env.ramps)
                        and env.ramps[ramp_id].is_dynamic
                    ):
                        ramp_throughput_history[ramp_id] += 1
                    else:
                        next_pipe_id = env.node_info[u.path[u.path_index]]["pipe_id"]
                        for r in dynamic_ramps:
                            if (
                                r.pipe_a_id == u.current_pipe_id
                                and r.pipe_b_id == next_pipe_id
                                or (
                                    r.pipe_b_id == u.current_pipe_id
                                    and r.pipe_a_id == next_pipe_id
                                )
                            ):
                                ramp_throughput_history[r.ramp_id] += 1
                                break
                if u.state == "ARRIVED":
                    arrived_uavs.add(u.uav_id)
                    last_arrival_turn = turn
        if track_uav_ids is not None:
            for uid in track_uav_ids:
                tracked_uav = uavs[uid]
                if tracked_uav.state in ["FLYING", "SWITCHING", "ARRIVED"]:
                    pipe_obj = env.pipelines[tracked_uav.current_pipe_id]
                    actual_3d_pos = (
                        pipe_obj.start_pt + pipe_obj.direction * tracked_uav.pos_1d
                    )
                    if not uav_trajectories[uid] or not np.allclose(
                        uav_trajectories[uid][-1], actual_3d_pos
                    ):
                        uav_trajectories[uid].append(actual_3d_pos)
        if trace_logger is not None:
            trace_logger.log_uav_objects(trace_algo_id or mode, mode, turn, uavs, env)
        previous_turn_violations = current_turn_violations
        total_hard_collisions += len(current_hard_collisions - previous_hard_collisions)
        previous_hard_collisions = current_hard_collisions
        if turn > 0 and turn % 100 == 0:
            print(
                f"  [{mode}] 已推演至第 {turn} 回合... (已到达: {len(arrived_uavs)}/{len(uavs)})"
            )
            pipe_traffic_history = {p.pipe_id: 0 for p in env.pipelines}
            ramp_throughput_history = {r.ramp_id: 0 for r in dynamic_ramps}
        compute_seconds = time.perf_counter() - turn_started_at
        export_seconds = 0.0
        if frame_exporter is not None:
            export_started_at = time.perf_counter()
            try:
                frame_exporter.write_uav_objects(turn, uavs)
            except Exception as exc:
                frame_exporter.close(complete=False, error=str(exc))
                if timing_recorder is not None:
                    timing_recorder.close(
                        initial_route_planning_seconds=initial_route_planning_seconds,
                        algorithm_total_seconds=time.perf_counter()
                        - algorithm_started_at,
                    )
                if trace_logger is not None:
                    trace_logger.close()
                raise
            export_seconds = time.perf_counter() - export_started_at
        if timing_recorder is not None:
            timing_recorder.record_turn(
                turn=turn,
                compute_seconds=compute_seconds,
                export_seconds=export_seconds,
                total_seconds=time.perf_counter() - turn_started_at,
                active_uavs=sum(
                    (1 for uav in uavs if uav.state in ("FLYING", "SWITCHING"))
                ),
                arrived_uavs=len(arrived_uavs),
            )
    effective_turns = last_arrival_turn if last_arrival_turn > 0 else turn + 1
    avg_kl = (
        sum(kl_history[:effective_turns]) / effective_turns
        if effective_turns > 0
        else 0.0
    )
    if trace_logger is not None:
        trace_logger.close()
    timing_summary = None
    if frame_exporter is not None:
        frame_exporter.close()
    if timing_recorder is not None:
        timing_summary = timing_recorder.close(
            initial_route_planning_seconds=initial_route_planning_seconds,
            algorithm_total_seconds=time.perf_counter() - algorithm_started_at,
        )
        print(
            f"[Timing] 平均每回合≈{timing_summary.average_turn_seconds:.3f}s | 仿真循环≈{timing_summary.simulation_loop_seconds:.3f}s | 算法总时间≈{timing_summary.algorithm_total_seconds:.3f}s | 实时倍率≈{timing_summary.real_time_factor:.3f}x"
        )
    metrics = {
        "load_balance": load_balance_metrics,
        "hard_collisions": total_hard_collisions,
        "timing": timing_summary.to_dict() if timing_summary is not None else {},
    }
    return (
        last_arrival_turn,
        total_safety_violations,
        kl_history,
        len(uavs) - len(arrived_uavs),
        avg_kl,
        avg_gain_history,
        uav_trajectories,
        metrics,
    )
