"""Goals are reproducible physical lane points, independent of routing state."""
import copy
import math
import random
from types import SimpleNamespace

import pytest


def _environment(top_count=50, length=12000.0, lanes=20):
    def pipe(pipe_id, height):
        return SimpleNamespace(pipe_id=pipe_id, height=height, length=length,
                               num_lanes=lanes)

    return SimpleNamespace(pipelines=[pipe(0, 100), pipe(1, 500)] +
                           [pipe(100 + i, 600) for i in range(top_count)])


def _pool(*args, **kwargs):
    from uav_sim.simulation.destination_pool import DestinationPool
    return DestinationPool(*args, **kwargs)


def _points(specs):
    return {(item["pipe_id"], item["lane_id"], item["distance_m"]) for item in specs}


def test_default_is_ten_distinct_original_sixth_layer_interior_lane_points():
    env = _environment()
    specs = _pool(env).specs()
    assert len(specs) == 10
    assert len({s["pipe_id"] for s in specs}) == 10
    assert len({s["node_id"] for s in specs}) == 10
    assert len(_points(specs)) == 10
    for index, spec in enumerate(specs):
        assert set(spec) == {"node_id", "pipe_id", "lane_id", "distance_m"}
        assert spec["node_id"] == "GOAL_FIXED_{}".format(index)
        assert 100 <= spec["pipe_id"] < 150
        assert 0 <= spec["lane_id"] < 20
        assert 0.0 < spec["distance_m"] < 12000.0


def test_fixed_pool_reuses_the_same_points_for_every_batch():
    pool = _pool(_environment(), seed=81)
    expected = pool.specs(0)
    assert pool.specs(99) == expected
    assert pool.all_specs(5) == expected
    assert _pool(_environment(), seed=81).specs() == expected
    assert _pool(_environment(), seed=82).specs() != expected


def test_seeded_generation_does_not_touch_process_random_state():
    before = random.getstate()
    pool = _pool(_environment(), policy="per_batch")
    pool.specs(5)
    pool.all_specs(7)
    assert random.getstate() == before


def test_per_batch_generation_is_reproducible_and_independent_of_request_order():
    env = _environment()
    pool = _pool(env, count=7, seed=13, policy="per_batch")
    third = pool.specs(3)
    first = pool.specs(0)
    comparison = _pool(env, count=7, seed=13, policy="per_batch")
    assert comparison.specs(0) == first
    assert comparison.specs(3) == third
    assert {s["node_id"] for s in third} == {"GOAL_BATCH_3_{}".format(i) for i in range(7)}
    assert not (_points(first) & _points(third))
    combined = pool.all_specs(4)
    assert len(combined) == 28
    assert len({s["node_id"] for s in combined}) == 28


def test_pipe_selection_is_independent_of_environment_container_order():
    env = _environment()
    reversed_env = SimpleNamespace(pipelines=list(reversed(env.pipelines)))
    assert _pool(env, seed=31).specs() == _pool(reversed_env, seed=31).specs()


def test_more_goals_than_top_pipes_have_distinct_points_and_node_ids():
    specs = _pool(_environment(top_count=2), count=47).specs()
    assert len(specs) == len(_points(specs)) == 47
    assert len({s["node_id"] for s in specs}) == 47
    assert {s["pipe_id"] for s in specs} == {100, 101}


def test_missions_are_balanced_round_robin_with_stable_zero_based_ids():
    pool = _pool(_environment(), count=3, policy="per_batch")
    goals = pool.specs(2)
    assigned = [pool.for_mission(i, batch_index=2) for i in range(8)]
    assert assigned == [goals[i % 3] for i in range(8)]


def test_outputs_are_defensive_copies_including_manifest_and_mission():
    pool = _pool(_environment())
    expected = copy.deepcopy(pool.specs())
    pool.specs()[0]["distance_m"] = -100.0
    pool.all_specs(2)[0]["lane_id"] = -1
    pool.for_mission(0)["node_id"] = "corrupted"
    manifest = pool.manifest()
    manifest["batches"][0]["specs"][0]["pipe_id"] = -1
    assert pool.specs() == expected


def test_manifest_records_only_generated_batches_in_sorted_order():
    pool = _pool(_environment(), count=2, seed=19, policy="per_batch")
    pool.specs(5)
    pool.specs(2)
    manifest = pool.manifest()
    assert manifest["policy"] == "per_batch"
    assert manifest["seed"] == 19
    assert manifest["count"] == 2
    assert manifest["manual"] is False
    assert [batch["batch_index"] for batch in manifest["batches"]] == [2, 5]
    assert manifest["batches"][0]["specs"] == list(pool.specs(2))


def test_fixed_manifest_records_a_single_generated_pool():
    pool = _pool(_environment())
    pool.specs(3)
    pool.specs(5)
    assert [batch["batch_index"] for batch in pool.manifest()["batches"]] == [0]


@pytest.mark.parametrize("count", [0, -1, 1.5, True, "10", None])
def test_count_must_be_a_positive_integer(count):
    with pytest.raises(ValueError, match="count"):
        _pool(_environment(), count=count)


@pytest.mark.parametrize("seed", [1.5, True, "42", None, float("nan")])
def test_seed_must_be_an_integer(seed):
    with pytest.raises(ValueError, match="seed"):
        _pool(_environment(), seed=seed)


@pytest.mark.parametrize("policy", ["new", "FIXED", "", None])
def test_policy_is_strict(policy):
    with pytest.raises(ValueError, match="policy"):
        _pool(_environment(), policy=policy)


@pytest.mark.parametrize("batch_index", [-1, 0.5, True, "0"])
def test_batch_index_must_be_a_nonnegative_integer(batch_index):
    pool = _pool(_environment())
    with pytest.raises(ValueError, match="batch_index"):
        pool.specs(batch_index)


@pytest.mark.parametrize("batch_count", [-1, 1.5, True, "2"])
def test_all_specs_validates_batch_count(batch_count):
    with pytest.raises(ValueError, match="batch_count"):
        _pool(_environment()).all_specs(batch_count)


def test_zero_batches_has_no_specs():
    assert _pool(_environment()).all_specs(0) == ()


@pytest.mark.parametrize("uav_id", [-1, 0.5, True, "0"])
def test_mission_id_must_be_a_nonnegative_integer(uav_id):
    with pytest.raises(ValueError, match="uav_id"):
        _pool(_environment()).for_mission(uav_id)


def test_missing_top_layer_does_not_fall_back_to_the_highest_available_pipe():
    with pytest.raises(ValueError, match="600"):
        _pool(_environment(top_count=0))


def test_empty_environment_is_rejected():
    with pytest.raises(ValueError, match="600"):
        _pool(SimpleNamespace(pipelines=[]))


@pytest.mark.parametrize("length", [0.0, -1.0, 1.e-12, float("nan"), float("inf")])
def test_invalid_or_too_short_top_pipe_is_rejected(length):
    with pytest.raises(ValueError, match="length|short"):
        _pool(_environment(top_count=1, length=length))


@pytest.mark.parametrize("lanes", [0, -1, 2.5, True])
def test_invalid_top_lane_count_is_rejected(lanes):
    with pytest.raises(ValueError, match="lane"):
        _pool(_environment(top_count=1, lanes=lanes))


def test_manual_goals_allow_legacy_lower_layers_and_preserve_supplied_ids():
    manual = [{"pipe_id": 0, "lane_id": 0, "distance_m": 123.456, "node_id": "legacy"},
              {"pipe_id": 1, "lane_id": 19, "distance_m": 800.0}]
    pool = _pool(_environment(top_count=0), manual=manual)
    goals = pool.specs()
    assert len(goals) == 2
    assert goals[0] == manual[0]
    assert goals[1]["pipe_id"] == 1
    assert goals[1]["node_id"].startswith("GOAL_")
    manual[0]["distance_m"] = -1
    assert pool.specs()[0]["distance_m"] == 123.456
    assert pool.manifest()["manual"] is True
    assert pool.manifest()["count"] == 2


@pytest.mark.parametrize("changes", [
    {"pipe_id": 999}, {"pipe_id": 0.5}, {"lane_id": -1}, {"lane_id": 20},
    {"lane_id": 0.5}, {"lane_id": True}, {"distance_m": -1},
    {"distance_m": 12000.1}, {"distance_m": float("nan")},
    {"distance_m": float("inf")}, {"node_id": ""}, {"node_id": None},
])
def test_invalid_manual_goal_is_rejected(changes):
    spec = dict(pipe_id=0, lane_id=0, distance_m=10.0)
    spec.update(changes)
    with pytest.raises(ValueError):
        _pool(_environment(), manual=[spec])


def test_manual_endpoint_selection_keeps_the_exact_requested_pipe_end():
    env = _environment()
    goals = _pool(env, manual=[dict(pipe_id=0, lane_id=0, distance_m=12000.)]).specs()
    assert goals[0]['distance_m'] == 12000.


@pytest.mark.parametrize("manual", [[], [{"pipe_id": 0}], [None], ["pipe0"]])
def test_manual_selection_must_be_nonempty_complete_records(manual):
    with pytest.raises(ValueError):
        _pool(_environment(), manual=manual)


def test_manual_duplicate_coordinates_and_ids_are_rejected():
    first = dict(pipe_id=0, lane_id=0, distance_m=100.0, node_id="same")
    for second in (dict(first, node_id="other"), dict(first, distance_m=200.0)):
        with pytest.raises(ValueError, match="duplicate"):
            _pool(_environment(), manual=[first, second])


def test_real_metropolis_compile_preserves_specifications_and_exact_lane_coordinates():
    import numpy as np
    from uav_sim.airspace.environment import Airspace
    from uav_sim.airspace.partition_graph import compile_partition, PartitionMaps

    env = Airspace(mode="metropolis", include_dynamic=False, num_pipes_per_layer=4)
    pool = _pool(env, count=10)
    specs = pool.all_specs(3)
    original = copy.deepcopy(specs)
    maps = PartitionMaps(compile_partition(env, destinations=specs))
    assert specs == original
    for spec in specs:
        pipe = env.pipelines[spec["pipe_id"]]
        destination = maps.destination(spec["node_id"])
        expected = pipe.lanes[spec["lane_id"]].start_pt + pipe.direction * spec["distance_m"]
        assert pipe.height == 600
        assert np.allclose(destination.position, expected, atol=1.e-9)
        assert math.isfinite(destination.distance_m)
