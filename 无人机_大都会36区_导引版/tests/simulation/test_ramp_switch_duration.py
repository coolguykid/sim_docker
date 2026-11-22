import math

from uav_sim.airspace import Airspace
from uav_sim.distributed.server_control_messages import observation_from_uav
from uav_sim.entities.uav import UAV


def _directed_env_and_ramp():
    env = Airspace(include_dynamic=False, topology_profile="balanced_sparse_v1")
    ramp = next(r for r in env.ramps if r.ramp_type == "inter_layer")
    return env, ramp


def test_uav_switch_timer_uses_ramp_length_and_entry_speed():
    env, ramp = _directed_env_and_ramp()
    uav = UAV(1, ramp.source_pipe_id, 0, ramp.target_pipe_id, env)
    uav.speed = 20.0

    expected = max(1, math.ceil(ramp.length / uav.speed / 1.0))

    assert uav._ramp_switch_ticks(ramp.ramp_id, sim_dt=1.0) == expected


def test_entering_directed_ramp_freezes_entry_speed_and_total_ticks():
    env, ramp = _directed_env_and_ramp()
    uav = UAV(2, ramp.source_pipe_id, 0, ramp.target_pipe_id, env)
    source_node = f"P{ramp.source_pipe_id}_R{ramp.ramp_id}"
    target_node = f"P{ramp.target_pipe_id}_R{ramp.ramp_id}"
    uav.state = "FLYING"
    uav.speed = 25.0
    uav.path = [source_node, target_node, "V_END"]
    uav.path_index = 0
    uav.pos_1d = ramp.dist_from_start_a

    uav.step([])

    expected = max(1, math.ceil(ramp.length / 25.0))
    assert uav.state == "SWITCHING"
    assert uav.current_ramp_id == ramp.ramp_id
    assert uav.ramp_entry_speed_mps == 25.0
    assert uav.ramp_total_ticks == expected
    assert uav.switch_timer == expected


def test_subpipe_continue_transition_does_not_enter_switching_state():
    env, _ramp = _directed_env_and_ramp()
    source_pipe = env.pipelines[0]
    target_pipe = env.pipelines[1]
    uav = UAV(3, source_pipe.pipe_id, 0, target_pipe.pipe_id, env)
    env.node_info["CONTINUE_TEST"] = {
        "pipe_id": target_pipe.pipe_id,
        "dist": 0.0,
        "type": "CONTINUE",
    }
    uav.state = "FLYING"
    uav.path = ["SOURCE_TEST", "CONTINUE_TEST", "V_END"]
    uav.path_index = 0

    uav.step([])

    assert uav.state == "FLYING"
    assert uav.current_pipe_id == target_pipe.pipe_id
    assert uav.path_index == 1
    assert uav.current_ramp_id is None


def test_switch_observation_reports_ramp_length_and_remaining_seconds():
    env, ramp = _directed_env_and_ramp()
    uav = UAV(4, ramp.source_pipe_id, 0, ramp.target_pipe_id, env)
    uav.state = "SWITCHING"
    uav.current_ramp_id = ramp.ramp_id
    uav.switch_timer = 7

    observation = observation_from_uav(uav)

    assert observation["ramp_length_m"] == ramp.length
    assert observation["ramp_remaining_seconds"] == 7.0
