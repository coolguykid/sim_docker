"""
safety_shield.py — server-side command validation for UAV motion.

The shield works on NumPy structured arrays and uses pipe/lane sorted indexes.
It checks adjacent UAVs in each lane instead of scanning all pairs globally.
"""
import math

import numpy as np

from uav_sim.algorithms.avoidance_features import collision_threshold_for_pipe
from uav_sim.algorithms.motion_constraints import (
    DEFAULT_UAV_MAX_ACCEL,
    DEFAULT_UAV_MAX_DECEL,
    clamp_speed_change,
)
from uav_sim.distributed.server_control_messages import STATE_FLYING, STATE_SWITCHING


def build_pipe_lane_index(observations):
    """Build {(pipe_id, lane_id): row_indexes_sorted_by_pos}."""
    index = {}
    if observations is None or len(observations) == 0:
        return index

    order = np.lexsort((
        observations["pos_1d"],
        observations["lane_id"],
        observations["pipe_id"],
    ))
    for idx in order:
        obs = observations[idx]
        if int(obs["state_code"]) != STATE_FLYING:
            continue
        key = (int(obs["pipe_id"]), int(obs["lane_id"]))
        index.setdefault(key, []).append(int(idx))
    return index


def predict_position(pos_1d, speed, tau):
    return float(pos_1d) + float(speed) * float(tau)


def _safe_distance(env, pipe_id, safety_factor=1.2,
                   rear_speed=0.0, front_speed=0.0,
                   rear_max_decel=DEFAULT_UAV_MAX_DECEL,
                   delta_t=0.2):
    pipe = env.pipelines[int(pipe_id)]
    base = safety_factor * collision_threshold_for_pipe(pipe)
    rear_speed = max(0.0, float(rear_speed))
    front_speed = max(0.0, float(front_speed))
    rear_max_decel = max(float(rear_max_decel), 1e-9)
    braking_margin = max(0.0, rear_speed * rear_speed - front_speed * front_speed) / (
        2.0 * rear_max_decel)
    reaction_margin = rear_speed * max(0.0, float(delta_t))
    return base + braking_margin + reaction_margin


def _obs_float(obs, name, default):
    if hasattr(obs, "dtype") and getattr(obs.dtype, "names", None) and name in obs.dtype.names:
        return float(obs[name])
    return float(default)


def _reachable_command_speed(obs, cmd_idx, commands, env, pipe_id):
    current_speed = _obs_float(obs, "speed", 0.0)
    if cmd_idx is None:
        return current_speed
    desired = float(commands[cmd_idx]["target_speed"])
    pipe = env.pipelines[int(pipe_id)]
    next_speed, _accel = clamp_speed_change(
        current_speed=current_speed,
        desired_speed=desired,
        pipe_max_speed=pipe.max_speed,
        max_accel=_obs_float(obs, "max_accel", DEFAULT_UAV_MAX_ACCEL),
        max_decel=_obs_float(obs, "max_decel", DEFAULT_UAV_MAX_DECEL),
    )
    return next_speed


def _command_lookup(commands):
    return {int(cmd["uav_id"]): i for i, cmd in enumerate(commands)}


def _candidate_lane(obs, command_by_uid, commands):
    cmd_idx = command_by_uid.get(int(obs["uav_id"]))
    if cmd_idx is None:
        return int(obs["lane_id"])
    return int(commands[cmd_idx]["target_lane_id"])


def _prediction_base(obs):
    state = int(obs["state_code"])
    if state == STATE_FLYING:
        return True, int(obs["pipe_id"]), float(obs["pos_1d"])
    if state == STATE_SWITCHING:
        next_pipe_id = int(obs["next_pipe_id"]) if "next_pipe_id" in obs.dtype.names else -1
        switch_timer = int(obs["switch_timer"]) if "switch_timer" in obs.dtype.names else 999
        if next_pipe_id >= 0 and switch_timer <= 1:
            return True, next_pipe_id, float(obs["next_pos_1d"])
    return False, -1, 0.0


def build_predicted_lane_index(observations, commands):
    """Index by pipe and commanded lane after target-lane decisions."""
    command_by_uid = _command_lookup(commands)
    tmp = []
    for idx, obs in enumerate(observations):
        active, pipe_id, pos_1d = _prediction_base(obs)
        if not active:
            continue
        tmp.append((
            pipe_id,
            _candidate_lane(obs, command_by_uid, commands),
            pos_1d,
            idx,
        ))
    tmp.sort(key=lambda item: (item[0], item[1], item[2]))
    index = {}
    for pipe_id, lane_id, _pos, idx in tmp:
        index.setdefault((pipe_id, lane_id), []).append(idx)
    return index


def _predicted_pos(obs):
    _active, _pipe_id, pos_1d = _prediction_base(obs)
    return pos_1d


def _predicted_pipe(obs):
    _active, pipe_id, _pos_1d = _prediction_base(obs)
    return pipe_id


def detect_conflicts(commands, observations, env, delta_t=0.2, horizon=1.0):
    command_by_uid = _command_lookup(commands)
    predicted_index = build_predicted_lane_index(observations, commands)
    conflicts = []
    steps = max(1, int(math.ceil(horizon / max(delta_t, 1e-6))))

    for (pipe_id, lane_id), indexes in predicted_index.items():
        if len(indexes) < 2:
            continue
        for left, right in zip(indexes, indexes[1:]):
            rear = observations[left]
            front = observations[right]
            rear_cmd_idx = command_by_uid.get(int(rear["uav_id"]))
            front_cmd_idx = command_by_uid.get(int(front["uav_id"]))
            rear_speed = _reachable_command_speed(rear, rear_cmd_idx, commands, env, pipe_id)
            front_speed = _reachable_command_speed(front, front_cmd_idx, commands, env, pipe_id)
            d_safe = _safe_distance(
                env,
                pipe_id,
                rear_speed=rear_speed,
                front_speed=front_speed,
                rear_max_decel=_obs_float(rear, "max_decel", DEFAULT_UAV_MAX_DECEL),
                delta_t=delta_t,
            )
            rear_base_pos = _predicted_pos(rear)
            front_base_pos = _predicted_pos(front)
            for step in range(steps + 1):
                tau = min(horizon, step * delta_t)
                rear_pos = predict_position(rear_base_pos, rear_speed, tau)
                front_pos = predict_position(front_base_pos, front_speed, tau)
                gap = front_pos - rear_pos
                if gap < d_safe:
                    conflicts.append({
                        "kind": "same_lane",
                        "rear_uav": int(rear["uav_id"]),
                        "front_uav": int(front["uav_id"]),
                        "pipe_id": int(pipe_id),
                        "lane_id": int(lane_id),
                        "time_offset": float(tau),
                        "distance": float(gap),
                        "required_distance": float(d_safe),
                        "rear_pos": float(rear_base_pos),
                        "front_pos": float(front_base_pos),
                    })
                    break
    return conflicts


def _priority(obs):
    # Higher tuple wins right-of-way. Lower uid is deterministic tie-breaker.
    return (
        int(obs["current_ramp_id"]) >= 0,
        float(obs["pos_1d"]),
        -int(obs["uav_id"]),
    )


def _resolve_one_conflict(commands, observations, conflict, obs_by_uid, cmd_by_uid, env, horizon):
    rear_uid = conflict["rear_uav"]
    front_uid = conflict["front_uav"]
    rear = obs_by_uid[rear_uid]
    front = obs_by_uid[front_uid]

    # In normal lane-following conflicts the rear vehicle yields. If positions
    # are degenerate, use deterministic right-of-way instead of stopping both.
    rear_switching = int(rear["state_code"]) == STATE_SWITCHING
    front_switching = int(front["state_code"]) == STATE_SWITCHING
    yield_uid = rear_uid
    if rear_switching and not front_switching:
        yield_uid = rear_uid
    elif front_switching and not rear_switching:
        yield_uid = front_uid
    elif float(rear["pos_1d"]) >= float(front["pos_1d"]) and _priority(rear) > _priority(front):
        yield_uid = front_uid

    yield_cmd_idx = cmd_by_uid[yield_uid]
    other_obs = front if yield_uid == rear_uid else rear
    yielding_obs = rear if yield_uid == rear_uid else front
    if int(yielding_obs["state_code"]) == STATE_SWITCHING:
        commands[yield_cmd_idx]["target_speed"] = 0.0
        commands[yield_cmd_idx]["target_lane_id"] = int(yielding_obs["lane_id"])
        commands[yield_cmd_idx]["hold_position"] = True
        commands[yield_cmd_idx]["hard_brake"] = True
        commands[yield_cmd_idx]["safety_modified"] = True
        commands[yield_cmd_idx]["action_id"] = 4
        return

    yielding_speed = _obs_float(yielding_obs, "speed", 0.0)
    yielding_decel = _obs_float(yielding_obs, "max_decel", DEFAULT_UAV_MAX_DECEL)
    gap = abs(_predicted_pos(other_obs) - _predicted_pos(yielding_obs))
    other_cmd_idx = cmd_by_uid[int(other_obs["uav_id"])]
    other_speed = _reachable_command_speed(other_obs, other_cmd_idx, commands, env, int(conflict["pipe_id"]))
    d_safe = _safe_distance(
        env,
        int(conflict["pipe_id"]),
        rear_speed=yielding_speed,
        front_speed=other_speed,
        rear_max_decel=yielding_decel,
        delta_t=max(horizon, 1e-6),
    )
    safe_speed = max(0.0, other_speed + (gap - d_safe) / max(horizon, 1e-6))
    if gap < d_safe:
        safe_speed = min(safe_speed, max(0.0, other_speed - 1e-3))

    commands[yield_cmd_idx]["target_speed"] = min(
        float(commands[yield_cmd_idx]["target_speed"]), safe_speed)
    commands[yield_cmd_idx]["target_lane_id"] = int(yielding_obs["lane_id"])
    commands[yield_cmd_idx]["safety_modified"] = True
    commands[yield_cmd_idx]["action_id"] = 4
    if safe_speed <= max(0.0, yielding_speed - yielding_decel) + 1e-6 or gap < d_safe:
        commands[yield_cmd_idx]["hard_brake"] = True
    if safe_speed <= 1e-6 and yielding_speed <= 1e-6:
        commands[yield_cmd_idx]["hold_position"] = True


def apply_safety_shield(commands, observations, env, delta_t=0.2, horizon=1.0,
                        max_passes=4):
    safe_commands = np.array(commands, copy=True)
    obs_by_uid = {int(obs["uav_id"]): obs for obs in observations}
    all_conflicts = []

    for _ in range(max_passes):
        cmd_by_uid = _command_lookup(safe_commands)
        conflicts = detect_conflicts(
            safe_commands, observations, env, delta_t=delta_t, horizon=horizon)
        if not conflicts:
            break
        all_conflicts.extend(conflicts)
        for conflict in conflicts:
            _resolve_one_conflict(
                safe_commands, observations, conflict, obs_by_uid,
                cmd_by_uid, env, horizon)

    return safe_commands, all_conflicts

