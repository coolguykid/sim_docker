from collections import Counter

from uav_sim.airspace import Airspace


def test_regional_airspace_builds_36_regions_with_30_lane_pipes():
    env = Airspace(mode="regional_6x6", include_dynamic=False)

    assert env.region_rows == 6
    assert env.region_cols == 6
    assert len(env.regions) == 36
    assert len(env.pipelines) == 36 * 6 * 10
    assert all(pipe.num_lanes == 30 for pipe in env.pipelines)

    by_region_layer = Counter((pipe.region_id, pipe.height) for pipe in env.pipelines)
    assert set(pipe.region_id for pipe in env.pipelines) == set(range(36))
    for region_id in range(36):
        for height in [100, 200, 300, 400, 500, 600]:
            assert by_region_layer[(region_id, height)] == 10


def test_regional_airspace_cross_region_ramps_connect_adjacent_same_layer_pipes():
    env = Airspace(mode="regional_6x6", include_dynamic=False)

    cross_ramps = [
        ramp for ramp in env.ramps
        if getattr(ramp, "ramp_type", "") == "cross_region_intra_layer"
    ]
    assert len(cross_ramps) == 60 * 6 * 3

    portal_pipe_pairs = set()
    for ramp in cross_ramps:
        pipe_a = env.pipelines[ramp.pipe_a_id]
        pipe_b = env.pipelines[ramp.pipe_b_id]
        assert pipe_a.height == pipe_b.height
        assert pipe_a.region_id != pipe_b.region_id
        assert abs(pipe_a.region_row - pipe_b.region_row) + abs(pipe_a.region_col - pipe_b.region_col) == 1
        assert getattr(ramp, "is_cross_region", False) is True
        portal_pipe_pairs.add((
            min(pipe_a.region_id, pipe_b.region_id),
            max(pipe_a.region_id, pipe_b.region_id),
            pipe_a.height,
            pipe_a.pipe_id,
            pipe_b.pipe_id,
        ))
    assert len(portal_pipe_pairs) == len(cross_ramps)


def test_regional_dijkstra_path_can_cross_regions():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    start_pipe = next(p for p in env.pipelines if p.region_id == 0 and p.height == 100)
    dest_pipe = next(p for p in env.pipelines if p.region_id == 35 and p.height == 600)

    path = env.find_shortest_path(
        start_pipe.pipe_id,
        0.0,
        dest_pipe.pipe_id,
        dest_pipe.length,
    )

    assert path
    ramp_ids = [
        env._extract_ramp_id_from_node(node)
        for node in path
        if env._extract_ramp_id_from_node(node) is not None
    ]
    assert any(env.ramps[rid].ramp_type == "cross_region_intra_layer" for rid in ramp_ids)
    regions = {
        env.node_info[node]["region_id"]
        for node in path
        if node in env.node_info and "region_id" in env.node_info[node]
    }
    assert len(regions) > 1
