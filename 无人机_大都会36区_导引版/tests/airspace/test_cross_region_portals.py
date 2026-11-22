from collections import Counter

import numpy as np

from uav_sim.airspace import Airspace
from uav_sim.airspace.regional_matrix import RegionalAirspaceMatrix


def test_cross_region_portals_index_adjacent_region_pairs_by_layer():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())

    undirected_pairs = {
        tuple(sorted((portal.region_a_id, portal.region_b_id)))
        for portal in matrix.cross_region_portals
    }
    counts = Counter(
        tuple(sorted((portal.region_a_id, portal.region_b_id)))
        for portal in matrix.cross_region_portals
    )

    assert len(undirected_pairs) == 60
    assert set(counts.values()) == {18}
    assert len(matrix.cross_region_portals) == 1080

    for portal in matrix.cross_region_portals[:100]:
        pipe_a = matrix.pipelines[portal.pipe_a_id]
        pipe_b = matrix.pipelines[portal.pipe_b_id]
        assert pipe_a.height == pipe_b.height == portal.layer_height
        assert pipe_a.region_id != pipe_b.region_id
        assert pipe_b.region_id in matrix.neighbor_region_ids(pipe_a.region_id)
        assert portal.node_a == f"P{portal.pipe_a_id}_R{portal.ramp_id}"
        assert portal.node_b == f"P{portal.pipe_b_id}_R{portal.ramp_id}"


def test_cross_region_portal_lookup_is_bidirectional():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())

    eastbound = matrix.cross_portals_between(0, 1)
    westbound = matrix.cross_portals_between(1, 0)

    assert len(eastbound) == 18
    assert len(westbound) == 18
    assert {portal.ramp_id for portal in eastbound} == {portal.ramp_id for portal in westbound}


def test_cross_region_portals_choose_boundary_near_pipe_pairings():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())

    portals = [
        portal for portal in matrix.cross_portals_between(14, 20)
        if portal.layer_height == 300
    ]
    distances = [
        float(np.linalg.norm(
            env.ramps[portal.ramp_id].pos_a
            - env.ramps[portal.ramp_id].pos_b
        ))
        for portal in portals
    ]

    assert len(distances) == 3
    assert max(distances) < 300.0
