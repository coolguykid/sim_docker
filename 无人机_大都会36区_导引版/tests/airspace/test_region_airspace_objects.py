from collections import Counter

from uav_sim.airspace import Airspace
from uav_sim.airspace.regional_matrix import RegionalAirspaceMatrix, RegionAirspace


def test_regional_matrix_instantiates_36_region_airspaces():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())

    assert len(matrix.region_airspaces) == 36
    assert all(isinstance(region, RegionAirspace) for region in matrix.region_airspaces.values())
    assert all(len(region.pipe_ids) == 60 for region in matrix.region_airspaces.values())

    counts = Counter(
        (region_id, matrix.pipelines[pipe_id].height)
        for region_id, region in matrix.region_airspaces.items()
        for pipe_id in region.pipe_ids
    )
    for region_id in range(36):
        for height in [100, 200, 300, 400, 500, 600]:
            assert counts[(region_id, height)] == 10


def test_region_airspace_local_graph_contains_only_region_pipe_nodes():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())
    region = matrix.region_airspaces[0]

    assert region.graph
    assert region.node_info
    assert set(region.pipe_nodes).issubset(set(region.pipe_ids))
    for info in region.node_info.values():
        assert info["region_id"] == 0


def test_region_airspace_finds_local_path_inside_single_region():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())
    region = matrix.region_airspaces[0]

    start_pipe = next(p for p in matrix.pipelines if p.region_id == 0 and p.height == 100)
    dest_pipe = next(p for p in matrix.pipelines if p.region_id == 0 and p.height == 600)
    result = region.find_local_path(
        start_pipe.pipe_id,
        0.0,
        dest_pipe.pipe_id,
        dest_pipe.length,
    )

    assert result.raw_path[0] == "V_START"
    assert result.raw_path[-1] == "V_END"
    assert result.cost < float("inf")
    assert result.node_info_patch
