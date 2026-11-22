from types import SimpleNamespace

from uav_sim.airspace import Airspace
from uav_sim.entities.uav import UAV


def test_uav_switching_uses_target_pipe_num_lanes_instead_of_fixed_ten():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    start_pipe = env.pipelines[0]
    target_pipe = next(
        pipe for pipe in env.pipelines
        if pipe.pipe_id != start_pipe.pipe_id and pipe.num_lanes == 30
    )
    target_node = "TEST_TARGET_30_LANE_PIPE"
    env.node_info[target_node] = {
        "pipe_id": target_pipe.pipe_id,
        "dist": 100.0,
        "type": "RAMP",
        "region_id": target_pipe.region_id,
    }

    uav = UAV(
        uav_id=999,
        start_pipe_id=start_pipe.pipe_id,
        lane_id=0,
        dest_pipe_id=target_pipe.pipe_id,
        env=env,
        capacity=0,
    )
    uav.state = "SWITCHING"
    uav.switch_timer = 1
    uav.path = [target_node]
    uav.path_index = 0

    blockers = [
        SimpleNamespace(
            current_pipe_id=target_pipe.pipe_id,
            current_lane_id=lane,
            pos_1d=100.0,
        )
        for lane in range(10)
    ]

    uav.step(blockers)

    assert uav.state == "FLYING"
    assert uav.current_pipe_id == target_pipe.pipe_id
    assert 10 <= uav.current_lane_id < target_pipe.num_lanes
    assert uav.current_region_id == target_pipe.region_id
