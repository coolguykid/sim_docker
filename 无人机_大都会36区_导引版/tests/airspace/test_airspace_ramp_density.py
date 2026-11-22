from collections import Counter

import numpy as np

from uav_sim.airspace import Airspace


def _ramp_incidence_counts(ramps):
    counts = Counter()
    for ramp in ramps:
        counts[ramp.pipe_a_id] += 1
        counts[ramp.pipe_b_id] += 1
    return counts


def _is_cross_layer(env, ramp):
    return abs(env.pipelines[ramp.pipe_a_id].height - env.pipelines[ramp.pipe_b_id].height) > 100


def test_static_airspace_uses_sparse_connected_ramp_backbone():
    env = Airspace(include_dynamic=False)

    static_ramps = [r for r in env.ramps if not r.is_dynamic]
    intra_static = [r for r in static_ramps if r.ramp_type == "intra_layer"]
    inter_static = [r for r in static_ramps if r.ramp_type == "inter_layer"]

    assert len(env.ramps) == len(static_ramps)
    assert len(intra_static) == 60
    assert len(inter_static) >= 300

    per_pipe = _ramp_incidence_counts(static_ramps)
    assert set(per_pipe) == {p.pipe_id for p in env.pipelines}
    assert min(per_pipe.values()) >= 11

    for ramp in intra_static:
        pipe_a = env.pipelines[ramp.pipe_a_id]
        pipe_b = env.pipelines[ramp.pipe_b_id]
        assert np.allclose(ramp.pos_a, pipe_a.end_pt)
        assert np.allclose(ramp.pos_b, pipe_b.start_pt)

    assert env.find_shortest_path(0, 0.0, 50, env.pipelines[50].length)


def test_static_inter_layer_ramps_give_each_pipe_8_adjacent_and_2_cross_connections():
    env = Airspace(include_dynamic=False)

    adjacent_counts = Counter()
    cross_counts = Counter()
    counted_pairs = set()
    for ramp in env.ramps:
        if ramp.is_dynamic or ramp.ramp_type != "inter_layer":
            continue
        if ramp.pair_id in counted_pairs:
            continue
        counted_pairs.add(ramp.pair_id)
        a = env.pipelines[ramp.pipe_a_id]
        b = env.pipelines[ramp.pipe_b_id]
        layer_gap = abs(a.height - b.height) // 100
        counts = adjacent_counts if layer_gap == 1 else cross_counts
        counts[a.pipe_id] += 1
        counts[b.pipe_id] += 1

    for pipe in env.pipelines:
        assert adjacent_counts[pipe.pipe_id] >= 8
        assert cross_counts[pipe.pipe_id] >= 2


def test_metropolis_mode_has_50_pipes_per_layer():
    env = Airspace(mode="metropolis", include_dynamic=False)

    per_layer = Counter(p.height for p in env.pipelines)
    assert per_layer == {100: 50, 200: 50, 300: 50, 400: 50, 500: 50, 600: 50}


def test_dynamic_density_levels_create_expected_counts():
    env_none = Airspace(inter_dynamic_level="none", intra_dynamic_level="none")
    assert not [r for r in env_none.ramps if r.is_dynamic]

    env_low = Airspace(inter_dynamic_level="low", intra_dynamic_level="low")
    inter_dynamic = [
        r for r in env_low.ramps
        if r.is_dynamic and r.ramp_type == "inter_layer"
    ]
    intra_dynamic = [
        r for r in env_low.ramps
        if r.is_dynamic and r.ramp_type == "intra_layer"
    ]

    logical_static_total = (
        env_low.ramp_stats["intra_static"]
        + env_low.ramp_stats["inter_static_pairs"]
    )
    assert len(inter_dynamic) == 2 * logical_static_total
    assert 0 < len(intra_dynamic) <= 60

    per_layer = Counter(env_low.pipelines[r.pipe_a_id].height for r in intra_dynamic)
    assert set(per_layer) == {100, 200, 300, 400, 500, 600}
    assert max(per_layer.values()) <= 10


def test_ultra_inter_layer_density_covers_every_inter_layer_pipe_pair():
    env = Airspace(inter_dynamic_level="ultra", intra_dynamic_level="none")

    covered_pairs = {
        tuple(sorted((r.pipe_a_id, r.pipe_b_id)))
        for r in env.ramps
        if r.ramp_type == "inter_layer"
    }
    expected_pairs = {
        (a.pipe_id, b.pipe_id)
        for i, a in enumerate(env.pipelines)
        for b in env.pipelines[i + 1:]
        if a.height != b.height
    }

    assert covered_pairs == expected_pairs


def test_ramp_ids_remain_index_compatible():
    env = Airspace(inter_dynamic_level="medium", intra_dynamic_level="medium")

    assert all(ramp.ramp_id == idx for idx, ramp in enumerate(env.ramps))
    assert all(env.ramps[ramp.ramp_id] is ramp for ramp in env.ramps)

