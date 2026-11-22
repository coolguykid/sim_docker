"""Shared UAV motion constraints.

The pipeline owns speed limits. The UAV owns acceleration and braking limits.
"""

DEFAULT_DT = 1.0
DEFAULT_UAV_MAX_ACCEL = 10.0
DEFAULT_UAV_MAX_DECEL = 10.0


def _positive_float(value, default):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return float(default)
    return value if value > 0.0 else float(default)


def max_accel_for_uav(uav):
    return _positive_float(
        getattr(uav, "max_accel", DEFAULT_UAV_MAX_ACCEL),
        DEFAULT_UAV_MAX_ACCEL,
    )


def max_decel_for_uav(uav):
    return _positive_float(
        getattr(uav, "max_decel", DEFAULT_UAV_MAX_DECEL),
        DEFAULT_UAV_MAX_DECEL,
    )


def _obs_value(obs, name, default):
    if hasattr(obs, "dtype") and getattr(obs.dtype, "names", None) and name in obs.dtype.names:
        return obs[name]
    if isinstance(obs, dict):
        return obs.get(name, default)
    return getattr(obs, name, default)


def max_accel_from_observation(obs):
    return _positive_float(_obs_value(obs, "max_accel", DEFAULT_UAV_MAX_ACCEL),
                           DEFAULT_UAV_MAX_ACCEL)


def max_decel_from_observation(obs):
    return _positive_float(_obs_value(obs, "max_decel", DEFAULT_UAV_MAX_DECEL),
                           DEFAULT_UAV_MAX_DECEL)


def compute_acceleration(old_speed, new_speed, dt=DEFAULT_DT):
    dt = max(float(dt), 1e-9)
    return (float(new_speed) - float(old_speed)) / dt


def clamp_speed_change(current_speed, desired_speed, pipe_max_speed,
                       max_accel=DEFAULT_UAV_MAX_ACCEL,
                       max_decel=DEFAULT_UAV_MAX_DECEL,
                       dt=DEFAULT_DT):
    current = max(0.0, float(current_speed))
    desired = max(0.0, float(desired_speed))
    pipe_limit = max(0.0, float(pipe_max_speed))
    dt = max(float(dt), 1e-9)
    accel_limit = _positive_float(max_accel, DEFAULT_UAV_MAX_ACCEL) * dt
    decel_limit = _positive_float(max_decel, DEFAULT_UAV_MAX_DECEL) * dt

    lower = max(0.0, current - decel_limit)
    upper = min(pipe_limit, current + accel_limit)
    if upper < lower:
        lower = upper
    next_speed = min(max(desired, lower), upper)
    return next_speed, compute_acceleration(current, next_speed, dt)


def braking_distance(speed, max_decel=DEFAULT_UAV_MAX_DECEL):
    speed = max(0.0, float(speed))
    decel = _positive_float(max_decel, DEFAULT_UAV_MAX_DECEL)
    return speed * speed / (2.0 * decel)


def normalized_acceleration(acceleration, max_accel=DEFAULT_UAV_MAX_ACCEL,
                            max_decel=DEFAULT_UAV_MAX_DECEL):
    acceleration = float(acceleration)
    if acceleration >= 0.0:
        denom = _positive_float(max_accel, DEFAULT_UAV_MAX_ACCEL)
    else:
        denom = _positive_float(max_decel, DEFAULT_UAV_MAX_DECEL)
    return max(-1.0, min(1.0, acceleration / denom))
