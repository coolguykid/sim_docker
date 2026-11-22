"""
server_control_messages.py — compact control-plane arrays for server-authoritative mode.

High-frequency state uploads and command downloads use NumPy structured arrays
so Ray can move contiguous buffers instead of many small Python dicts.
"""
import numpy as np


STATE_WAITING = 0
STATE_FLYING = 1
STATE_SWITCHING = 2
STATE_ARRIVED = 3

PHASE_TO_PICKUP = 0
PHASE_TO_DEST = 1

STATE_TO_CODE = {
    "WAITING": STATE_WAITING,
    "FLYING": STATE_FLYING,
    "SWITCHING": STATE_SWITCHING,
    "ARRIVED": STATE_ARRIVED,
}
CODE_TO_STATE = {v: k for k, v in STATE_TO_CODE.items()}

PHASE_TO_CODE = {
    "TO_PICKUP": PHASE_TO_PICKUP,
    "TO_DEST": PHASE_TO_DEST,
}
CODE_TO_PHASE = {v: k for k, v in PHASE_TO_CODE.items()}


OBSERVATION_DTYPE = np.dtype([
    ("uav_id", np.int32),
    ("region_id", np.int16),
    ("pipe_id", np.int32),
    ("lane_id", np.int16),
    ("pos_1d", np.float64),
    ("speed", np.float64),
    ("max_accel", np.float64),
    ("max_decel", np.float64),
    ("last_acceleration", np.float64),
    ("state_code", np.int8),
    ("phase_code", np.int8),
    ("path_index", np.int16),
    ("dest_pipe_id", np.int32),
    ("final_dest_pipe_id", np.int32),
    ("pickup_pipe_id", np.int32),
    ("pickup_pos_1d", np.float64),
    ("spawn_turn", np.int32),
    ("current_ramp_id", np.int32),
    ("next_pipe_id", np.int32),
    ("next_pos_1d", np.float64),
    ("switch_timer", np.int16),
    ("ramp_length_m", np.float64),
    ("ramp_remaining_seconds", np.float64),
    ("last_action", np.int16),
    ("gaussian_intensity", np.float64),
    ("needs_route", np.bool_),
    ("blocked_ticks", np.int16),
])


COMMAND_DTYPE = np.dtype([
    ("uav_id", np.int32),
    ("target_speed", np.float64),
    ("target_lane_id", np.int16),
    ("action_id", np.int16),
    ("hold_position", np.bool_),
    ("hard_brake", np.bool_),
    ("enter_ramp_id", np.int32),
    ("safety_modified", np.bool_),
    ("route_update", np.bool_),
])


def make_observation_array(rows):
    arr = np.zeros(len(rows), dtype=OBSERVATION_DTYPE)
    arr["pickup_pipe_id"] = -1
    arr["current_ramp_id"] = -1
    arr["next_pipe_id"] = -1
    arr["next_pos_1d"] = -1.0
    arr["switch_timer"] = -1
    arr["ramp_length_m"] = 0.0
    arr["ramp_remaining_seconds"] = 0.0
    arr["last_action"] = -1
    arr["final_dest_pipe_id"] = -1
    for idx, row in enumerate(rows):
        for name in OBSERVATION_DTYPE.names:
            if name in row:
                arr[idx][name] = row[name]
    return arr


def make_command_array(rows):
    arr = np.zeros(len(rows), dtype=COMMAND_DTYPE)
    arr["enter_ramp_id"] = -1
    arr["hard_brake"] = False
    for idx, row in enumerate(rows):
        for name in COMMAND_DTYPE.names:
            if name in row:
                arr[idx][name] = row[name]
    return arr


def observation_from_uav(uav):
    pickup_pipe = -1
    pickup_pos = -1.0
    if getattr(uav, "pickup_pt", None) is not None:
        pickup_pipe = int(uav.pickup_pt.pipe_id)
        pickup_pos = float(uav.pickup_pt.pos_1d)
    next_pipe_id = -1
    next_pos_1d = -1.0
    if getattr(uav, "state", None) == "SWITCHING":
        path = getattr(uav, "path", []) or []
        path_index = int(getattr(uav, "path_index", 0))
        if 0 <= path_index < len(path):
            node_info = getattr(uav.env, "node_info", {}).get(path[path_index])
            if node_info is not None:
                next_pipe_id = int(node_info["pipe_id"])
                next_pos_1d = float(node_info["dist"])
    current_ramp_id = getattr(uav, "current_ramp_id", -1)
    if current_ramp_id is None:
        current_ramp_id = -1
    ramp_length_m = 0.0
    if int(current_ramp_id) >= 0:
        ramp_by_id = getattr(uav.env, "ramp_by_id", None)
        if ramp_by_id is not None:
            ramp = ramp_by_id.get(int(current_ramp_id))
        else:
            ramp = None
            if int(current_ramp_id) < len(getattr(uav.env, "ramps", ())):
                ramp = uav.env.ramps[int(current_ramp_id)]
        if ramp is not None:
            ramp_length_m = float(getattr(ramp, "length", 0.0))
    sim_dt = max(float(getattr(uav, "sim_dt", 1.0)), 1e-9)
    ramp_remaining_seconds = max(
        0.0,
        float(getattr(uav, "switch_timer", 0)) * sim_dt,
    )
    return {
        "uav_id": int(uav.uav_id),
        "region_id": int(getattr(uav, "current_region_id", 0)),
        "pipe_id": int(uav.current_pipe_id),
        "lane_id": int(uav.current_lane_id),
        "pos_1d": float(uav.pos_1d),
        "speed": float(uav.speed),
        "max_accel": float(getattr(uav, "max_accel", 10.0)),
        "max_decel": float(getattr(uav, "max_decel", 10.0)),
        "last_acceleration": float(getattr(uav, "last_acceleration", 0.0)),
        "state_code": STATE_TO_CODE.get(uav.state, STATE_FLYING),
        "phase_code": PHASE_TO_CODE.get(uav.phase, PHASE_TO_DEST),
        "path_index": int(getattr(uav, "path_index", 0)),
        "dest_pipe_id": int(uav.dest_pipe_id),
        "final_dest_pipe_id": int(getattr(uav, "final_dest_pipe_id", uav.dest_pipe_id)),
        "pickup_pipe_id": pickup_pipe,
        "pickup_pos_1d": pickup_pos,
        "spawn_turn": int(getattr(uav, "spawn_turn", 0)),
        "current_ramp_id": int(current_ramp_id),
        "next_pipe_id": next_pipe_id,
        "next_pos_1d": next_pos_1d,
        "switch_timer": int(getattr(uav, "switch_timer", -1)),
        "ramp_length_m": ramp_length_m,
        "ramp_remaining_seconds": ramp_remaining_seconds,
        "last_action": int(getattr(uav, "last_action", -1) if getattr(uav, "last_action", None) is not None else -1),
        "gaussian_intensity": float(getattr(uav, "gaussian_intensity", 0.0)),
        "needs_route": bool(getattr(uav, "needs_route_replan", False) or not getattr(uav, "path", [])),
        "blocked_ticks": int(getattr(uav, "micro_block_ticks", 0)),
    }


def observations_from_uavs(uavs):
    return make_observation_array([observation_from_uav(u) for u in uavs])


def commands_by_uav(command_array):
    return {int(cmd["uav_id"]): cmd for cmd in command_array}
