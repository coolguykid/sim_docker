from uav_sim.airspace import Airspace
from uav_sim.airspace.region_graph import RegionGraph
from uav_sim.airspace.regional_matrix import RegionalAirspaceMatrix


def test_region_graph_finds_adjacent_path_across_6x6_matrix():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())
    graph = RegionGraph(matrix.regions, matrix.cross_region_portals)

    plan = graph.shortest_region_path(0, 35)

    assert plan.region_ids[0] == 0
    assert plan.region_ids[-1] == 35
    assert len(plan.region_ids) == 11
    for region_a, region_b in zip(plan.region_ids, plan.region_ids[1:]):
        assert region_b in matrix.neighbor_region_ids(region_a)


def test_region_graph_returns_single_region_when_origin_matches_destination():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())
    graph = RegionGraph(matrix.regions, matrix.cross_region_portals)

    plan = graph.shortest_region_path(12, 12)

    assert plan.region_ids == [12]
    assert plan.estimated_cost == 0.0


def test_region_graph_avoids_neighbor_edge_when_all_its_portals_are_closed():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())
    graph = RegionGraph(matrix.regions, matrix.cross_region_portals)
    states = {ramp.ramp_id: True for ramp in env.ramps}
    for portal in matrix.cross_portals_between(0, 1):
        states[portal.ramp_id] = False

    plan = graph.shortest_region_path(0, 1, ramp_states=states)

    assert plan.region_ids[0] == 0
    assert plan.region_ids[-1] == 1
    assert plan.region_ids != [0, 1]
