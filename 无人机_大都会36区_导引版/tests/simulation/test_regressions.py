import numpy as np
from uav_sim.airspace import Airspace, PickupPoint
from uav_sim.distributed.orchestrator import SimulationOrchestrator
from uav_sim.entities.uav import UAV


def test_distributed_kl_uses_pickup_demand_distribution():
    env = Airspace(inter_dynamic_pct=0.0, intra_dynamic_pct=0.0)
    pickup = PickupPoint(0, pipe_id=5, pos_1d=2000.0, capacity=1)
    pickup_pos = (
        env.pipelines[pickup.pipe_id].start_pt
        + env.pipelines[pickup.pipe_id].direction * pickup.pos_1d
    )
    orchestrator = SimulationOrchestrator(
        env=env, uav_groups=[], registry=None, total_uav_count=1, pickup_points=[pickup]
    )
    states = {
        0: {
            "pipe_id": pickup.pipe_id,
            "pos_1d": pickup.pos_1d,
            "state": "FLYING",
            "gaussian_intensity": 1.0,
        }
    }
    kl = orchestrator._compute_kl_from_uav_updates(states)
    assert np.isclose(
        kl, 0.0, atol=1e-09
    ), f"matching supply/demand should have KL near 0, got {kl}"


def test_apply_path_keeps_direct_destination_as_motion_target():
    env = Airspace(inter_dynamic_pct=0.0, intra_dynamic_pct=0.0)
    uav = UAV(uav_id=0, start_pipe_id=0, lane_id=0, dest_pipe_id=0, env=env, capacity=0)
    uav.state = "FLYING"
    uav.phase = "TO_DEST"
    dest_pos = env.pipelines[0].length
    uav._apply_path(["V_START", "V_END"], dest_pos)
    uav.step([])
    assert uav.state == "FLYING"
    assert uav.pos_1d > 0.0
    assert uav.pos_1d < dest_pos
