import pickle
import random
import sys
import numpy as np
from uav_sim.airspace import Airspace
from uav_sim.distributed.authoritative_control_server import AuthoritativeControlCore
from uav_sim.app import _build_uav_configs, generate_uavs
from uav_sim.algorithms.safety_shield import apply_safety_shield, build_pipe_lane_index
from uav_sim.algorithms.avoidance_features import collision_threshold_for_height
from uav_sim.algorithms.safety_shield import _safe_distance
from uav_sim.distributed.server_control_messages import (
    COMMAND_DTYPE,
    OBSERVATION_DTYPE,
    make_command_array,
    make_observation_array,
    observation_from_uav,
)
from uav_sim.entities.uav import UAV
from uav_sim.distributed.uav_actor import UAVGroupActorBase


def test_control_messages_are_numpy_and_pickle_safe():
    observations = make_observation_array(
        [
            {
                "uav_id": 1,
                "pipe_id": 0,
                "lane_id": 2,
                "pos_1d": 10.0,
                "speed": 20.0,
                "state_code": 1,
                "phase_code": 1,
                "dest_pipe_id": 5,
                "final_dest_pipe_id": 5,
                "spawn_turn": 0,
            }
        ]
    )
    commands = make_command_array(
        [
            {
                "uav_id": 1,
                "target_speed": 15.0,
                "target_lane_id": 2,
                "action_id": 3,
                "hold_position": False,
            }
        ]
    )
    assert observations.dtype == OBSERVATION_DTYPE
    assert commands.dtype == COMMAND_DTYPE
    assert pickle.loads(pickle.dumps(observations))[0]["uav_id"] == 1
    assert pickle.loads(pickle.dumps(commands))[0]["target_speed"] == 15.0


def test_observation_includes_uav_acceleration_limits():
    assert "max_accel" in OBSERVATION_DTYPE.names
    assert "max_decel" in OBSERVATION_DTYPE.names
    assert "last_acceleration" in OBSERVATION_DTYPE.names
    env = Airspace(include_dynamic=False)
    uav = UAV(7, 0, 0, 1, env, capacity=0)
    uav.last_acceleration = -5.0
    row = observation_from_uav(uav)
    assert row["max_accel"] == 10.0
    assert row["max_decel"] == 10.0
    assert row["last_acceleration"] == -5.0


def test_command_dtype_has_hard_brake_audit_flag():
    assert "hard_brake" in COMMAND_DTYPE.names


def test_pipe_lane_index_sorts_without_global_pairwise_scan():
    observations = make_observation_array(
        [
            {
                "uav_id": 1,
                "pipe_id": 0,
                "lane_id": 0,
                "pos_1d": 300.0,
                "speed": 20.0,
                "state_code": 1,
            },
            {
                "uav_id": 2,
                "pipe_id": 0,
                "lane_id": 0,
                "pos_1d": 100.0,
                "speed": 20.0,
                "state_code": 1,
            },
            {
                "uav_id": 3,
                "pipe_id": 1,
                "lane_id": 0,
                "pos_1d": 200.0,
                "speed": 20.0,
                "state_code": 1,
            },
        ]
    )
    index = build_pipe_lane_index(observations)
    assert [int(observations[i]["uav_id"]) for i in index[0, 0]] == [2, 1]
    assert [int(observations[i]["uav_id"]) for i in index[1, 0]] == [3]


def test_safety_shield_prevents_cross_actor_rear_end_without_global_hover():
    env = Airspace(mode="standard", include_dynamic=False)
    observations = make_observation_array(
        [
            {
                "uav_id": 10,
                "pipe_id": 0,
                "lane_id": 0,
                "pos_1d": 100.0,
                "speed": 10.0,
                "state_code": 1,
            },
            {
                "uav_id": 11,
                "pipe_id": 0,
                "lane_id": 0,
                "pos_1d": 70.0,
                "speed": 60.0,
                "state_code": 1,
            },
        ]
    )
    commands = make_command_array(
        [
            {"uav_id": 10, "target_speed": 10.0, "target_lane_id": 0, "action_id": 0},
            {"uav_id": 11, "target_speed": 60.0, "target_lane_id": 0, "action_id": 2},
        ]
    )
    safe_commands, conflicts = apply_safety_shield(
        commands, observations, env, delta_t=0.2, horizon=1.0
    )
    by_uid = {int(cmd["uav_id"]): cmd for cmd in safe_commands}
    assert conflicts
    assert bool(by_uid[11]["safety_modified"])
    assert not bool(by_uid[10]["hold_position"])
    assert float(by_uid[11]["target_speed"]) < 60.0


def test_authoritative_core_generates_safe_rule_commands():
    env = Airspace(mode="standard", include_dynamic=False)
    core = AuthoritativeControlCore(
        env.get_readonly_snapshot(),
        routing_mode="DIJKSTRA",
        avoidance_policy="rule",
        safety_substep=0.2,
    )
    observations = make_observation_array(
        [
            {
                "uav_id": 1,
                "pipe_id": 0,
                "lane_id": 0,
                "pos_1d": 100.0,
                "speed": 10.0,
                "state_code": 1,
            },
            {
                "uav_id": 2,
                "pipe_id": 0,
                "lane_id": 0,
                "pos_1d": 90.0,
                "speed": 60.0,
                "state_code": 1,
            },
        ]
    )
    result = core.compute_commands(turn=0, observations=observations, shared_context={})
    by_uid = {int(cmd["uav_id"]): cmd for cmd in result["commands"]}
    assert result["conflicts"]
    assert bool(by_uid[2]["safety_modified"])
    assert float(by_uid[2]["target_speed"]) < 60.0


def test_authoritative_core_can_prepare_initial_routes_before_turn_zero():
    env = Airspace(mode="standard", include_dynamic=False)
    actor = UAVGroupActorBase(
        group_id=0,
        uav_configs=[
            {
                "uav_id": 0,
                "start_pipe_id": 0,
                "lane_id": 0,
                "dest_pipe_id": 10,
                "capacity": 0,
                "spawn_turn": 0,
                "final_dest_pipe_id": 10,
                "pickup_info": None,
            }
        ],
        env_snapshot_ref=env.get_readonly_snapshot(),
        routing_mode="DIJKSTRA",
        control_mode="server_authoritative",
    )
    actor.reset_all()
    core = AuthoritativeControlCore(
        env.get_readonly_snapshot(), routing_mode="DIJKSTRA", avoidance_policy="rule"
    )
    observations = actor.report_control_observations(turn=0)
    updates = core.prepare_initial_routes(
        0,
        observations,
        {
            "ramp_states": {ramp.ramp_id: True for ramp in env.ramps},
            "ramp_state_version": 0,
            "traffic_cost_version": 0,
        },
    )
    assert len(updates) == 1
    assert updates[0].raw_path[-1] == "V_END"
    assert actor.apply_route_updates(updates) == 1
    assert actor.uavs[0].needs_route_replan is False


def test_authoritative_core_replans_flying_uavs_when_ramp_version_changes():
    env = Airspace(mode="standard", include_dynamic=False)
    actor = UAVGroupActorBase(
        group_id=0,
        uav_configs=[
            {
                "uav_id": 0,
                "start_pipe_id": 0,
                "lane_id": 0,
                "dest_pipe_id": 10,
                "capacity": 0,
                "spawn_turn": 0,
                "final_dest_pipe_id": 10,
                "pickup_info": None,
            }
        ],
        env_snapshot_ref=env.get_readonly_snapshot(),
        routing_mode="DIJKSTRA",
        control_mode="server_authoritative",
    )
    actor.reset_all()
    core = AuthoritativeControlCore(
        env.get_readonly_snapshot(), routing_mode="DIJKSTRA", avoidance_policy="rule"
    )
    states = {ramp.ramp_id: True for ramp in env.ramps}
    initial = core.prepare_initial_routes(
        0,
        actor.report_control_observations(0),
        {"ramp_states": states, "ramp_state_version": 0, "traffic_cost_version": 0},
    )
    actor.apply_route_updates(initial)
    observations = actor.report_control_observations(1)
    assert bool(observations[0]["needs_route"]) is False
    result = core.compute_commands(
        1,
        observations,
        {"ramp_states": states, "ramp_state_version": 1, "traffic_cost_version": 0},
    )
    assert len(result["route_updates"]) == 1
    assert result["route_updates"][0].uav_id == 0


def test_server_authoritative_actor_refreshes_external_ramp_states():
    env = Airspace(mode="standard", include_dynamic=False)
    actor = UAVGroupActorBase(
        group_id=0,
        uav_configs=[
            {
                "uav_id": 0,
                "start_pipe_id": 0,
                "lane_id": 0,
                "dest_pipe_id": 10,
                "capacity": 0,
                "spawn_turn": 0,
                "final_dest_pipe_id": 10,
                "pickup_info": None,
            }
        ],
        env_snapshot_ref=env.get_readonly_snapshot(),
        routing_mode="DIJKSTRA",
        control_mode="server_authoritative",
    )
    actor.reset_all()
    ramp_id = env.ramps[0].ramp_id
    commands = make_command_array([])
    actor.apply_server_commands(commands, turn=0, ramp_states={ramp_id: False})
    assert actor.local_env.ramps[ramp_id].is_active is False


def test_safe_distance_includes_rear_braking_distance():
    env = Airspace(include_dynamic=False)
    pipe_id = 0
    d_safe = _safe_distance(
        env, pipe_id, rear_speed=20.0, front_speed=0.0, rear_max_decel=10.0, delta_t=0.2
    )
    static_only = 1.2 * collision_threshold_for_height(env.pipelines[pipe_id].height)
    assert d_safe > static_only + 20.0


def test_server_authoritative_hard_brake_is_rate_limited():
    env = Airspace(mode="standard", include_dynamic=False)
    actor = UAVGroupActorBase(
        group_id=0,
        uav_configs=[
            {
                "uav_id": 0,
                "start_pipe_id": 0,
                "lane_id": 0,
                "dest_pipe_id": 5,
                "capacity": 0,
                "spawn_turn": 0,
                "final_dest_pipe_id": 5,
                "pickup_info": None,
            }
        ],
        env_snapshot_ref=env.get_readonly_snapshot(),
        routing_mode="DIJKSTRA",
        control_mode="server_authoritative",
    )
    actor.reset_all()
    uav = actor.uavs[0]
    uav.state = "FLYING"
    uav.speed = 20.0
    actor.local_env.node_info["TEST_DEST"] = {
        "pipe_id": 0,
        "dist": 1000.0,
        "type": "END",
    }
    uav.path = ["CUR", "TEST_DEST"]
    uav.path_index = 0
    commands = make_command_array(
        [
            {
                "uav_id": 0,
                "target_speed": 0.0,
                "target_lane_id": 0,
                "action_id": 4,
                "hard_brake": True,
            }
        ]
    )
    actor.apply_server_commands(commands, turn=0)
    assert uav.speed == 10.0
    assert uav.last_acceleration == -10.0


def test_server_authoritative_actor_needs_no_neural_runtime():
    before = set(sys.modules)
    env = Airspace(mode="standard", include_dynamic=False)
    actor = UAVGroupActorBase(
        group_id=0,
        uav_configs=[
            {
                "uav_id": 0,
                "start_pipe_id": 0,
                "lane_id": 0,
                "dest_pipe_id": 5,
                "capacity": 0,
                "spawn_turn": 0,
                "final_dest_pipe_id": 5,
                "pickup_info": None,
            }
        ],
        env_snapshot_ref=env.get_readonly_snapshot(),
        routing_mode="DIJKSTRA",
        control_mode="server_authoritative",
    )
    assert actor.local_env.routing_mode == "DIJKSTRA"
    assert "torch" not in set(sys.modules) - before


def test_safety_shield_checks_switching_exit_against_flying_uav():
    env = Airspace(mode="standard", include_dynamic=False)
    observations = make_observation_array(
        [
            {
                "uav_id": 1,
                "pipe_id": 0,
                "lane_id": 4,
                "pos_1d": 2500.0,
                "speed": 20.0,
                "state_code": 1,
            },
            {
                "uav_id": 2,
                "pipe_id": 1,
                "lane_id": 4,
                "pos_1d": 100.0,
                "speed": 20.0,
                "state_code": 2,
                "next_pipe_id": 0,
                "next_pos_1d": 2500.0,
                "switch_timer": 1,
            },
        ]
    )
    commands = make_command_array(
        [
            {"uav_id": 1, "target_speed": 20.0, "target_lane_id": 4, "action_id": 0},
            {"uav_id": 2, "target_speed": 20.0, "target_lane_id": 4, "action_id": 0},
        ]
    )
    safe_commands, conflicts = apply_safety_shield(
        commands, observations, env, delta_t=0.2, horizon=1.0
    )
    by_uid = {int(cmd["uav_id"]): cmd for cmd in safe_commands}
    assert conflicts
    assert bool(by_uid[2]["safety_modified"])
    assert bool(by_uid[2]["hold_position"])


def test_server_authoritative_switching_exit_uses_command_lane(monkeypatch):
    env = Airspace(mode="standard", include_dynamic=False)
    actor = UAVGroupActorBase(
        group_id=0,
        uav_configs=[
            {
                "uav_id": 0,
                "start_pipe_id": 0,
                "lane_id": 0,
                "dest_pipe_id": 5,
                "capacity": 0,
                "spawn_turn": 0,
                "final_dest_pipe_id": 5,
                "pickup_info": None,
            }
        ],
        env_snapshot_ref=env.get_readonly_snapshot(),
        routing_mode="DIJKSTRA",
        control_mode="server_authoritative",
    )
    actor.reset_all()
    uav = actor.uavs[0]
    target_node = "TEST_SWITCH_TARGET"
    actor.local_env.node_info[target_node] = {
        "pipe_id": 1,
        "dist": 120.0,
        "type": "RAMP_EXIT",
    }
    uav.path = ["CUR", target_node]
    uav.path_index = 1
    uav.state = "SWITCHING"
    uav.switch_timer = 1
    uav.current_ramp_id = 0
    import uav_sim.entities.uav as uav_module

    monkeypatch.setattr(uav_module.random, "choice", lambda lanes: 2)
    commands = make_command_array(
        [{"uav_id": 0, "target_speed": 20.0, "target_lane_id": 7, "action_id": 0}]
    )
    actor.apply_server_commands(commands, turn=0)
    assert uav.state == "FLYING"
    assert uav.current_pipe_id == 1
    assert uav.current_lane_id == 7
    assert uav.pos_1d == 120.0


def _hard_collision_pairs(env, uav_updates):
    lanes = {}
    for uid, info in uav_updates.items():
        if info.get("state") != "FLYING":
            continue
        lanes.setdefault((info["pipe_id"], info["lane_id"]), []).append((uid, info))
    hard_pairs = set()
    for (pipe_id, _lane_id), lane_rows in lanes.items():
        lane_rows.sort(key=lambda row: row[1]["pos_1d"])
        for follower, leader in zip(lane_rows, lane_rows[1:]):
            dist = leader[1]["pos_1d"] - follower[1]["pos_1d"]
            if dist <= 1e-06:
                hard_pairs.add((follower[0], leader[0]))
    return hard_pairs


def _run_authoritative_core_smoke(mode, avoidance_policy):
    random.seed(23)
    np.random.seed(23)
    env = Airspace(
        mode="standard", inter_dynamic_level="low", intra_dynamic_level="low"
    )
    for ramp in env.ramps:
        ramp.is_active = not getattr(ramp, "is_dynamic", False)
    config = {
        "num_uavs": 12,
        "capacity_weights": {0: 1},
        "dest_pipe_ids": [30, 35, 40, 45],
        "batch_interval": 50,
    }
    uavs = generate_uavs(env, config)
    actor = UAVGroupActorBase(
        group_id=0,
        uav_configs=_build_uav_configs(uavs, []),
        env_snapshot_ref=env.get_readonly_snapshot(),
        routing_mode=mode,
        control_mode="server_authoritative",
    )
    actor.reset_all()
    actor.set_routing_mode(mode)
    core = AuthoritativeControlCore(
        env.get_readonly_snapshot(),
        routing_mode=mode,
        avoidance_policy=avoidance_policy,
        safety_substep=0.2,
    )
    arrived = set()
    hard_collisions = set()
    for turn in range(500):
        observations = actor.report_control_observations(turn=turn)
        traffic_load = {p.pipe_id: 0 for p in env.pipelines}
        for obs in observations:
            if int(obs["state_code"]) == 1:
                traffic_load[int(obs["pipe_id"])] += 1
        context = {
            "traffic_load": traffic_load,
            "ramp_states": {r.ramp_id: r.is_active for r in env.ramps},
            "collisions_multiplier": {p.pipe_id: 1.0 for p in env.pipelines},
        }
        control_result = core.compute_commands(turn, observations, context)
        tick_result = actor.apply_server_commands(
            control_result["commands"],
            route_updates=control_result["route_updates"],
            turn=turn,
        )
        arrived.update(tick_result["arrivals"])
        hard_collisions.update(_hard_collision_pairs(env, tick_result["uav_updates"]))
        if len(arrived) == len(uavs):
            break
    return (len(uavs) - len(arrived), hard_collisions)


def test_server_authoritative_core_reaches_destination_without_hard_collisions():
    dijkstra_missed, dijkstra_hard = _run_authoritative_core_smoke("DIJKSTRA", "rule")
    assert dijkstra_missed == 0
    assert dijkstra_hard == set()
