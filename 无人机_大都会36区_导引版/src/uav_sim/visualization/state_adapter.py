"""Materialize an enhanced UAV's exact state for traces and engine frames."""


def partition_update(uav, env, time_seconds):
    state = uav.sample_partition_state(time_seconds)
    graph = uav._partition_graph
    edge = graph.edges.get(state.edge_id)
    node = graph.nodes[state.node_id]
    ramp = getattr(env, "ramp_by_id", {}).get(state.transit_ramp)
    pid = edge.pipe_id if edge is not None else node.pipe_id
    if pid is None and state.transit_ramp is None:
        pid = node.pipe_id
    if pid is None:
        pid = (
            (ramp.pipe_b_id if state.transit_direction == "reverse" else ramp.pipe_a_id)
            if ramp
            else uav.current_pipe_id
        )
    pipe = env.pipelines[pid]
    distance = sum(
        (p - s) * d for p, s, d in zip(state.position, pipe.start_pt, pipe.direction)
    )
    return dict(
        sim_time_seconds=time_seconds,
        pipe_id=pid,
        external_pipe_id=getattr(pipe, "external_pipe_id", pid),
        region_id=state.region_id,
        lane_id=state.lane_id,
        pos_1d=distance,
        pos_3d=state.position,
        speed=state.speed,
        state=state.status,
        phase=uav.phase,
        max_accel=uav.max_accel,
        max_decel=uav.max_decel,
        last_acceleration=state.last_acceleration,
        last_action=getattr(uav, "last_action", None),
        current_ramp_id=state.transit_ramp,
        external_ramp_id=None
        if ramp is None
        else getattr(ramp, "external_ramp_id", ramp.ramp_id),
        ramp_length_m=0.0 if ramp is None else float(ramp.length),
        ramp_remaining_seconds=0.0,
        goal_node_id=state.goal.node_id,
        goal_position_m=state.goal.position,
        arrival_time_seconds=state.arrival_time,
    )
