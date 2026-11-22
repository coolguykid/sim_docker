"""Collision thresholds and safe lanes for rule avoidance."""
COLLISION_THRESHOLD_BY_HEIGHT = {100: 8, 200: 10, 300: 15, 400: 20, 500: 25, 600: 30}


def collision_threshold_for_height(height):
    return COLLISION_THRESHOLD_BY_HEIGHT.get(int(height), 15)


def collision_threshold_for_pipe(pipe):
    configured = getattr(pipe, "d_safe", None)
    if configured is not None and float(configured) > 0.0:
        return float(configured)
    return float(collision_threshold_for_height(pipe.height))


def _value(obj, name, default=None):
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _same_uav(other, uav):
    return _value(other, "uav_id") == getattr(uav, "uav_id", None)


def _other_lane(other):
    return _value(other, "lane_id", _value(other, "current_lane_id", 0))


def _other_pos(other):
    return float(_value(other, "pos_1d", 0.0))


def find_safe_lanes(uav, pipe_uavs, num_lanes, sense_range):
    safe_lanes = []
    for lane_idx in range(num_lanes):
        if lane_idx == uav.current_lane_id:
            continue
        is_safe = True
        for other in pipe_uavs:
            if _same_uav(other, uav):
                continue
            if (
                _other_lane(other) == lane_idx
                and abs(_other_pos(other) - uav.pos_1d) < sense_range
            ):
                is_safe = False
                break
        if is_safe:
            safe_lanes.append(lane_idx)
    return safe_lanes
