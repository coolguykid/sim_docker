from types import SimpleNamespace

import pytest

from uav_sim.airspace import Airspace
from uav_sim.airspace.hierarchical_router import HierarchicalRegionalRouter
from uav_sim.airspace.region_graph import RegionGraph
from uav_sim.airspace.regional_matrix import (
    CrossRegionPortal,
    RegionalAirspaceMatrix,
)


def _switch_edges(graph, source_node, target_node):
    return [
        edge for edge in graph.get(source_node, ())
        if edge[0] == target_node and edge[2] == "SWITCH"
    ]


def test_inter_layer_ramp_adds_only_source_to_target_switch_edge():
    env = Airspace(
        include_dynamic=False,
        topology_profile="balanced_sparse_v1",
    )
    ramp = next(r for r in env.ramps if r.ramp_type == "inter_layer")
    source_node = f"P{ramp.source_pipe_id}_R{ramp.ramp_id}"
    target_node = f"P{ramp.target_pipe_id}_R{ramp.ramp_id}"

    assert len(_switch_edges(env.graph, source_node, target_node)) == 1
    assert _switch_edges(env.graph, target_node, source_node) == []


def test_region_local_graph_enforces_the_same_ramp_direction():
    env = Airspace(
        include_dynamic=False,
        topology_profile="balanced_sparse_v1",
    )
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())
    ramp = next(r for r in env.ramps if r.ramp_type == "inter_layer")
    region = matrix.get_region(env.pipelines[ramp.source_pipe_id].region_id)
    source_node = f"P{ramp.source_pipe_id}_R{ramp.ramp_id}"
    target_node = f"P{ramp.target_pipe_id}_R{ramp.ramp_id}"

    assert len(_switch_edges(region.graph, source_node, target_node)) == 1
    assert _switch_edges(region.graph, target_node, source_node) == []


def test_explicit_bidirectional_legacy_ramp_still_adds_reverse_edge():
    env = Airspace(
        include_dynamic=False,
        topology_profile="legacy_dense_bidirectional_v1",
    )
    ramp = next(r for r in env.ramps if r.ramp_type == "inter_layer")
    source_node = f"P{ramp.source_pipe_id}_R{ramp.ramp_id}"
    target_node = f"P{ramp.target_pipe_id}_R{ramp.ramp_id}"

    assert len(_switch_edges(env.graph, source_node, target_node)) == 1
    assert len(_switch_edges(env.graph, target_node, source_node)) == 1


def test_region_portal_indexes_only_allowed_direction():
    portal = CrossRegionPortal(
        ramp_id=10,
        region_a_id=0,
        region_b_id=1,
        pipe_a_id=3,
        pipe_b_id=4,
        dist_a=20.0,
        dist_b=30.0,
        layer_height=100,
        local_layer_index=0,
        side_a="east",
        side_b="west",
        time_penalty=2.0,
        node_a="P3_R10",
        node_b="P4_R10",
        is_bidirectional=False,
    )
    matrix = RegionalAirspaceMatrix.__new__(RegionalAirspaceMatrix)
    matrix.cross_region_portals = [portal]
    index = matrix._build_portal_index()

    assert index[(0, 1)] == [portal]
    assert (1, 0) not in index

    regions = [
        {"region_id": 0, "center_x": 0.0, "center_y": 0.0},
        {"region_id": 1, "center_x": 100.0, "center_y": 0.0},
    ]
    graph = RegionGraph(regions, [portal])
    assert graph.shortest_region_path(0, 1).region_ids == [0, 1]
    assert graph.shortest_region_path(1, 0).region_ids == []


def test_hierarchical_router_rejects_reverse_use_of_directed_portal():
    portal = SimpleNamespace(
        region_a_id=0,
        region_b_id=1,
        node_a="A",
        node_b="B",
        pipe_a_id=3,
        pipe_b_id=4,
        dist_a=10.0,
        dist_b=20.0,
        is_bidirectional=False,
    )

    with pytest.raises(ValueError, match="方向"):
        HierarchicalRegionalRouter._portal_direction(portal, 1, 0)
