import time
from uav_sim.airspace import Airspace


def test_balanced_directed_topology_builds_with_expected_scale():
    started = time.perf_counter()
    env = Airspace(
        mode="regional_6x6",
        include_dynamic=True,
        topology_profile="balanced_sparse_v1",
        inter_dynamic_level="low",
        intra_dynamic_level="low",
        random_seed=42,
    )
    elapsed = time.perf_counter() - started
    assert len(env.pipelines) == 2160
    assert len(env.ramps) == 14040
    assert len(env.node_info) == 32400
    assert env.ramp_stats["inter_static_pairs"] == 1080
    assert env.ramp_stats["inter_dynamic_pairs"] == 3240
    assert all((ramp.length > 0.0 for ramp in env.ramps))
    assert all((ramp.ramp_id == index for index, ramp in enumerate(env.ramps)))
    assert elapsed < 30.0


def test_legacy_dense_directed_v2_seed_42_scale_is_stable():
    env = Airspace(
        mode="regional_6x6",
        include_dynamic=True,
        topology_profile="legacy_dense_directed_v2",
        inter_dynamic_level="low",
        intra_dynamic_level="low",
        random_seed=42,
    )
    # Verified against the unchanged original working source on 2026-10-24.
    assert len(env.ramps) == 55102
    assert len(env.node_info) == 114524
    assert env.ramp_stats["inter_static_pairs"] == 10827
    assert env.ramp_stats["inter_dynamic_pairs"] == 12987
