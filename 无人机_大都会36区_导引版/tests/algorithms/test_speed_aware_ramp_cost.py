import pytest

from uav_sim.airspace import Airspace
from uav_sim.airspace.hierarchical_router import HierarchicalRegionalRouter


def test_switch_edge_cost_changes_with_uav_speed():
    env = Airspace(include_dynamic=False, topology_profile="balanced_sparse_v1")
    ramp = next(r for r in env.ramps if r.ramp_type == "inter_layer")
    source_node = f"P{ramp.source_pipe_id}_R{ramp.ramp_id}"
    target_node = f"P{ramp.target_pipe_id}_R{ramp.ramp_id}"

    slow = env.edge_cost(source_node, target_node, uav_speed=10.0)
    fast = env.edge_cost(source_node, target_node, uav_speed=20.0)

    assert slow == pytest.approx(ramp.length / 10.0)
    assert fast == pytest.approx(ramp.length / 20.0)
    assert fast < slow


def test_hierarchical_route_cache_separates_speed_buckets():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    router = HierarchicalRegionalRouter(env.get_readonly_snapshot())
    start = next(pipe for pipe in env.pipelines if pipe.region_id == 0)
    destination = next(
        pipe for pipe in env.pipelines
        if pipe.region_id == 5 and pipe.height == 600
    )

    router.find_path(
        start.pipe_id,
        0.0,
        destination.pipe_id,
        destination.length,
        uav_speed=10.2,
    )
    router.find_path(
        start.pipe_id,
        0.0,
        destination.pipe_id,
        destination.length,
        uav_speed=19.8,
    )

    assert len(router._route_cache) == 2
