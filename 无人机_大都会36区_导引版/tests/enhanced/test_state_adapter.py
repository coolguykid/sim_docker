import json
from dataclasses import replace

from tests.enhanced.test_uav_partition_motion import fixture
from uav_sim.entities.trajectory_profile import build_prefix_profile
from uav_sim.visualization.frame_exporter import FrameJsonExporter


def test_lazy_profile_export_uses_sampled_actual_pipe_and_coordinate():
    from uav_sim.visualization.state_adapter import partition_update
    from uav_sim.entities.uav import UAV

    env, graph, goal = fixture()
    u = UAV(0, 0, 0, 1, env, capacity=0)
    u.configure_partition_motion(graph, "a", goal)
    u.set_partition_route(("ab", "bc", "cd", "de", "ef"))
    u._prefix_profile = build_prefix_profile(u._road_state, graph, stop_at_end=False)
    sample = u._prefix_profile.sample(u._prefix_profile.end_time - 0.1)
    update = partition_update(u, env, u._prefix_profile.end_time - 0.1)
    assert update["pipe_id"] == 1
    assert update["pos_3d"] == sample.position
    assert update["speed"] == sample.speed
    assert update["last_acceleration"] == sample.last_acceleration
    assert update["goal_node_id"] == sample.goal.node_id


def test_failed_export_manifest_cannot_claim_complete(tmp_path):
    from uav_sim.airspace.environment import Airspace

    env = Airspace(mode="standard", include_dynamic=False)
    exporter = FrameJsonExporter(tmp_path, env, "test", "1")
    exporter.close(error="simulation failed")
    assert json.loads((tmp_path / "manifest.json").read_text())["complete"] is False


def test_enhanced_scene_exports_the_same_36_management_regions_as_live_frames(tmp_path):
    from uav_sim.airspace.environment import Airspace
    from uav_sim.simulation.metropolis_runtime import MetropolisSession
    env = Airspace(mode='metropolis', num_pipes_per_layer=2, include_dynamic=False)
    cfg = dict(num_uavs=1, max_batch_size=1, max_sim_time_seconds=1,
               drain_timeout_seconds=1, run_output_dir=str(tmp_path), frame_export_enabled=True)
    with MetropolisSession(env,cfg) as session:
        scene = json.loads((tmp_path/'scene.json').read_text(encoding='utf-8'))
        assert len(env.regions) == 1  # The original physical generator stays one world.
        assert len(scene['regions']) == len(session.graph.regions) == 36
        assert {r['region_id'] for r in scene['regions']} == set(range(36))
        assert len(scene['goals']) == 10
        assert len(scene['pipelines']) == 12
        assert any(len(p['management_region_ids']) > 1 for p in scene['pipelines'])
