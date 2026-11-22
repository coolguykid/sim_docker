"""Shared micro-avoidance action semantics."""

ACTION_KEEP = 0
ACTION_ACCEL_LIGHT = 1
ACTION_ACCEL_FULL = 2
ACTION_BRAKE_LIGHT = 3
ACTION_BRAKE_FULL = 4
ACTION_CHANGE_LANE = 5
ACTION_COUNT = 6


def desired_speed_for_action(action, current_speed, pipe_max_speed,
                             max_accel, max_decel, dt=1.0,
                             current_speed_limit=None):
    action = int(action)
    current_speed = max(0.0, float(current_speed))
    pipe_max_speed = max(0.0, float(pipe_max_speed))
    max_accel = max(0.0, float(max_accel))
    max_decel = max(0.0, float(max_decel))
    dt = max(float(dt), 1e-9)
    cap = pipe_max_speed if current_speed_limit is None else min(
        pipe_max_speed, max(0.0, float(current_speed_limit)))

    if action == ACTION_KEEP:
        desired = current_speed
    elif action == ACTION_ACCEL_LIGHT:
        desired = current_speed + 0.5 * max_accel * dt
    elif action == ACTION_ACCEL_FULL:
        desired = current_speed + max_accel * dt
    elif action == ACTION_BRAKE_LIGHT:
        desired = current_speed - 0.5 * max_decel * dt
    elif action == ACTION_BRAKE_FULL:
        desired = current_speed - max_decel * dt
    elif action == ACTION_CHANGE_LANE:
        desired = current_speed
    else:
        desired = current_speed

    return min(max(0.0, desired), cap)
