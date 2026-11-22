from collections import defaultdict

import numpy as np
import pytest

from uav_sim.airspace import Airspace


@pytest.fixture(scope="module")
def static_env():
    return Airspace(
        mode="regional_6x6",
        include_dynamic=False,
        topology_profile="balanced_sparse_v1",
    )


@pytest.fixture(scope="module")
def dynamic_env():
    return Airspace(
        mode="regional_6x6",
        include_dynamic=True,
        topology_profile="balanced_sparse_v1",
        inter_dynamic_level="low",
        intra_dynamic_level="low",
    )


def _inter_layer_pairs(env, dynamic=None):
    pairs = defaultdict(list)
    for ramp in env.ramps:
        if ramp.ramp_type != "inter_layer":
            continue
        if dynamic is not None and ramp.is_dynamic is not dynamic:
            continue
        pairs[ramp.pair_id].append(ramp)
    return pairs


def test_each_logical_inter_layer_link_expands_to_two_directed_ramps(static_env):
    inter = [r for r in static_env.ramps if r.ramp_type == "inter_layer"]

    assert len(inter) == 2160
    assert all(not r.is_bidirectional for r in inter)

    by_pair = _inter_layer_pairs(static_env)
    assert None not in by_pair
    assert {len(items) for items in by_pair.values()} == {2}
    for first, second in by_pair.values():
        assert first.ramp_id != second.ramp_id
        assert first.source_pipe_id == second.target_pipe_id
        assert first.target_pipe_id == second.source_pipe_id
        assert not (
            np.allclose(first.pos_a, second.pos_b)
            and np.allclose(first.pos_b, second.pos_a)
        )


def test_balanced_sparse_directed_counts_are_stable(dynamic_env):
    assert dynamic_env.ramp_stats["inter_static"] == 2160
    assert dynamic_env.ramp_stats["inter_static_pairs"] == 1080
    assert dynamic_env.ramp_stats["inter_dynamic"] == 6480
    assert dynamic_env.ramp_stats["inter_dynamic_pairs"] == 3240
    assert len(dynamic_env.ramps) == 14040
    assert len(dynamic_env.node_info) == 32400


def test_opposite_dynamic_ramp_directions_can_be_switched_independently(dynamic_env):
    dynamic_pairs = _inter_layer_pairs(dynamic_env, dynamic=True)
    first, second = next(iter(dynamic_pairs.values()))

    first.is_active = False

    assert first.is_active is False
    assert second.is_active is True
