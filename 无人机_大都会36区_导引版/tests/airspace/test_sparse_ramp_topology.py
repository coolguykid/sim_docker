from collections import defaultdict

from uav_sim.airspace import Airspace


def _reachable(start, adjacency):
    seen = {start}
    stack = [start]
    while stack:
        node = stack.pop()
        for neighbor in adjacency.get(node, ()):
            if neighbor not in seen:
                seen.add(neighbor)
                stack.append(neighbor)
    return seen


def _assert_pipe_transition_graph_strongly_connected(env):
    nodes = {pipe.pipe_id for pipe in env.pipelines}
    adjacency = {pipe_id: [] for pipe_id in nodes}
    for ramp in env.ramps:
        adjacency[ramp.source_pipe_id].append(ramp.target_pipe_id)
        if ramp.is_bidirectional:
            adjacency[ramp.target_pipe_id].append(ramp.source_pipe_id)
    reverse = defaultdict(list)
    for node, neighbors in adjacency.items():
        for neighbor in neighbors:
            reverse[neighbor].append(node)

    start = next(iter(nodes))
    assert _reachable(start, adjacency) == nodes
    assert _reachable(start, reverse) == nodes


def test_balanced_sparse_regional_topology_has_expected_backbone():
    env = Airspace(
        mode="regional_6x6",
        include_dynamic=False,
        topology_profile="balanced_sparse_v1",
    )

    assert env.ramp_stats == {
        "intra_static": 2160,
        "inter_static": 2160,
        "inter_static_pairs": 1080,
        "inter_dynamic": 0,
        "inter_dynamic_pairs": 0,
        "intra_dynamic": 0,
        "cross_region": 1080,
    }
    assert len(env.ramps) == 5400
    assert len(env.node_info) == 15120


def test_sparse_regional_route_graph_is_strongly_connected():
    env = Airspace(
        mode="regional_6x6",
        include_dynamic=False,
        topology_profile="balanced_sparse_v1",
    )

    _assert_pipe_transition_graph_strongly_connected(env)


def test_dynamic_ramps_still_exist_and_have_index_compatible_ids():
    env = Airspace(
        mode="regional_6x6",
        include_dynamic=True,
        topology_profile="balanced_sparse_v1",
        inter_dynamic_level="low",
        intra_dynamic_level="low",
    )

    assert any(ramp.is_dynamic for ramp in env.ramps)
    assert all(ramp.ramp_id == idx for idx, ramp in enumerate(env.ramps))
    assert env.ramp_stats["inter_dynamic"] > 0
    assert env.ramp_stats["intra_dynamic"] > 0


def test_balanced_sparse_cross_region_count_is_three_per_layer():
    env = Airspace(
        mode="regional_6x6",
        include_dynamic=False,
        topology_profile="balanced_sparse_v1",
    )
    pair_layer_counts = defaultdict(int)
    for ramp in env.ramps:
        if not ramp.is_cross_region:
            continue
        region_pair = tuple(sorted((ramp.region_a_id, ramp.region_b_id)))
        height = env.pipelines[ramp.pipe_a_id].height
        pair_layer_counts[(region_pair, height)] += 1

    assert len(pair_layer_counts) == 60 * 6
    assert set(pair_layer_counts.values()) == {3}


def test_legacy_dense_reference_profile_has_exact_historical_size_for_any_seed():
    for seed in (7, 99):
        env = Airspace(
            mode="regional_6x6",
            include_dynamic=True,
            topology_profile="legacy_dense_bidirectional_v1",
            inter_dynamic_level="low",
            intra_dynamic_level="low",
            random_seed=seed,
        )

        assert len(env.ramps) == 31744
        assert len(env.node_info) == 67808
        assert all(
            ramp.is_bidirectional
            for ramp in env.ramps
            if ramp.ramp_type == "inter_layer"
        )
        assert all(ramp.ramp_id == index for index, ramp in enumerate(env.ramps))


def test_legacy_dense_alias_selects_directed_v2_profile():
    env = Airspace(
        include_dynamic=False,
        topology_profile="legacy_dense",
    )

    assert env.topology_profile.name == "legacy_dense_directed_v2"
    assert all(
        not ramp.is_bidirectional
        for ramp in env.ramps
        if ramp.ramp_type == "inter_layer"
    )
