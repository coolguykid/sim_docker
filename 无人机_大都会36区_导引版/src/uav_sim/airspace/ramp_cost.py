"""Shared, speed-aware ramp cost helpers."""


def speed_bucket(speed_mps):
    """Return the common cache/coalescing bucket for a UAV speed."""
    if speed_mps is None:
        return None
    return round(max(float(speed_mps), 1.0), 0)


def ramp_travel_speed(env, ramp, requested_speed=None):
    if requested_speed is not None and float(requested_speed) > 0.0:
        return float(requested_speed)
    source = env.pipelines[int(ramp.source_pipe_id)]
    target = env.pipelines[int(ramp.target_pipe_id)]
    return max(
        1.0,
        min(float(source.max_speed), float(target.max_speed)),
    )


def switch_edge_cost(env, ramp_id, requested_speed=None):
    ramp_id = int(ramp_id)
    ramp_by_id = getattr(env, "ramp_by_id", None)
    if ramp_by_id is not None:
        ramp = ramp_by_id[ramp_id]
    else:
        candidate = env.ramps[ramp_id]
        if int(candidate.ramp_id) == ramp_id:
            ramp = candidate
        else:
            ramp = next(item for item in env.ramps if int(item.ramp_id) == ramp_id)
    speed = ramp_travel_speed(env, ramp, requested_speed)
    if hasattr(ramp, "travel_time_seconds"):
        return float(ramp.travel_time_seconds(speed))
    return float(ramp.length) / speed
