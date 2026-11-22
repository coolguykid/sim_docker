from uav_sim.airspace import Airspace
from uav_sim.app import generate_uavs


def test_regional_airspace_spawns_10000_uavs_in_first_batch_without_lane_reuse():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    top_pipes = [pipe.pipe_id for pipe in env.pipelines if pipe.height == 600]
    config = {
        "num_uavs": 10000,
        "capacity_weights": {0: 100, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0},
        "dest_pipe_ids": top_pipes[:5],
        "batch_interval": 50,
    }

    uavs = generate_uavs(env, config)

    assert len(uavs) == 10000
    assert {uav.spawn_turn for uav in uavs} == {0}
    assert all(0 <= uav.start_lane_id < 30 for uav in uavs)
    assert len({(uav.start_pipe_id, uav.start_lane_id) for uav in uavs}) == 10000
    assert len({uav.start_region_id for uav in uavs}) == 36
