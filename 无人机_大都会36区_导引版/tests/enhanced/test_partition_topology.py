"""Regression requirements for partitions of one original metropolis."""
import math
from collections import defaultdict
from types import SimpleNamespace

import numpy as np
import pytest

from uav_sim.airspace.environment import Airspace, Pipeline, Ramp


def test_metropolis_preserves_one_world_and_original_lane_geometry():
    env = Airspace(mode="metropolis", include_dynamic=False)
    assert len(env.pipelines) == 300
    assert len(env.regions) == 1
    assert [p.pipe_id for p in env.pipelines] == list(range(300))
    assert [env.pipelines[i * 50].max_speed for i in range(6)] == [19, 40, 60, 80, 100, 120]
    assert sum(p.num_lanes for p in env.pipelines if p.height == 100) == 1000
    assert env.topology_profile.name == "metropolis_partition_v1"
    assert 1200 <= len(env.ramps) < 4000
    for pipe in env.pipelines:
        assert pipe.num_lanes == 20
        normal = np.array([-pipe.direction[1], pipe.direction[0], 0.])
        for lane in pipe.lanes:
            shift = normal * (lane.lane_id - 9.5) * 10.
            assert np.allclose(lane.start_pt, pipe.start_pt + shift)
            assert np.allclose(lane.end_pt, pipe.end_pt + shift)
    # The physical directed graph remains strongly connected, not merely its pipe abstraction.
    def reach(adjacency, start):
        seen, stack = {start}, [start]
        while stack:
            for neighbor in adjacency.get(stack.pop(), ()):
                if neighbor not in seen:
                    seen.add(neighbor)
                    stack.append(neighbor)
        return seen
    adjacency, reverse = defaultdict(list), defaultdict(list)
    for source, edges in env.graph.items():
        for target, *_ in edges:
            adjacency[source].append(target)
            reverse[target].append(source)
    assert reach(adjacency, "P0_START") == set(env.node_info)
    assert reach(reverse, "P0_START") == set(env.node_info)


def test_compile_splits_pipes_ramps_and_interior_goal_without_length_loss():
    from uav_sim.airspace.partition_graph import compile_partition
    first = Pipeline(0, [100., 100., 100.], [29000., 100., 100.], 100, 19, 20)
    second = Pipeline(1, [29000., 29000., 200.], [100., 29000., 200.], 200, 40, 20)
    ramp = Ramp(0, first, second, pos_a_3d=first.end_pt, pos_b_3d=second.start_pt)
    env = SimpleNamespace(pipelines=[first, second], ramps=[ramp], x_min=0., x_max=30000.,
                          y_min=0., y_max=30000., topology_version=0, continuation_edges=[])
    goal = {"node_id": "goal", "pipe_id": 1, "distance_m": 12345.6789, "lane_id": 19}
    graph = compile_partition(env, [goal])
    assert len(graph.regions) == 36
    assert graph.metadata["original_pipe_count"] == 2
    assert graph.metadata["pipe_endpoints"][0] == {"start": "P0_START", "end": "P0_END"}
    assert graph.metadata["pipes"][0]["speed_limit_mps"] == 19
    assert graph.nodes["goal"].distance_m == goal["distance_m"]
    for pipe in env.pipelines:
        edges = [e for e in graph.edges.values() if e.kind == "FLY" and e.pipe_id == pipe.pipe_id]
        assert sum(e.length_m for e in edges) == pytest.approx(pipe.length)
        assert all(np.dot(np.subtract(e.points[-1], e.points[0]), pipe.direction) >= 0 for e in edges)
        assert all(e.lane_count == 20 and len(e.offsets_start) == 20 for e in edges)
    ramp_edges = [e for e in graph.edges.values() if e.kind == "RAMP"]
    assert len({e.region_id for e in ramp_edges}) == 6
    assert sum(e.length_m for e in ramp_edges) == pytest.approx(ramp.length)
    assert any(e.kind == "BOUNDARY" and e.ramp_id == 0 for e in graph.edges.values())
    fragments = graph.metadata["ramp_fragments"]
    for edge in graph.edges.values():
        if edge.ramp_id is None:
            continue
        fragment = fragments[edge.edge_id]
        assert fragment["ramp_id"] == 0
        assert fragment["direction"] == "forward"
        assert 0. <= fragment["fraction_start"] <= fragment["fraction_end"] <= 1.
        assert len(fragment["offsets_at_entry"]) == len(fragment["offsets_at_exit"]) == 20
        if edge.kind == "BOUNDARY":
            assert fragment["fraction_start"] == fragment["fraction_end"]
    assert all(e.length_m == 0 for e in graph.edges.values() if e.kind == "BOUNDARY")
    assert all(0 <= e.region_id < 36 for e in graph.edges.values())
    assert len(env.pipelines) == 2  # compilation did not replace physical objects


def test_destination_lane_zero_and_rounded_neighbor_have_distinct_exact_nodes():
    from uav_sim.airspace.partition_graph import compile_partition, PartitionMaps
    env = Airspace(mode="metropolis", include_dynamic=False, num_pipes_per_layer=4)
    pipe = env.pipelines[20]
    goals = [dict(node_id="goal_left", pipe_id=pipe.pipe_id, distance_m=1234.00001, lane_id=0),
             dict(node_id="goal_right", pipe_id=pipe.pipe_id, distance_m=1234.00002, lane_id=19)]
    graph = compile_partition(env, goals)
    maps = PartitionMaps(graph)
    left, right = maps.destination("goal_left"), maps.destination("goal_right")
    assert left.node_id != right.node_id
    assert left.distance_m != right.distance_m
    expected = pipe.lanes[0].start_pt + pipe.direction * left.distance_m
    assert np.allclose(left.position, expected)
    assert math.dist(left.position, right.position) == pytest.approx(190.)


@pytest.mark.parametrize("pipe_count", [8, 50])
def test_compiled_physical_endpoints_are_reachable_from_every_bottom_start(pipe_count):
    from uav_sim.airspace.partition_graph import compile_partition
    env = Airspace(mode="metropolis", include_dynamic=False, num_pipes_per_layer=pipe_count)
    graph = compile_partition(env)
    adjacency, reverse = defaultdict(list), defaultdict(list)
    for edge in graph.edges.values():
        adjacency[edge.source].append(edge.target)
        reverse[edge.target].append(edge.source)
    for pipe in env.pipelines[:pipe_count]:
        seen, stack = {"P{}_START".format(pipe.pipe_id)}, ["P{}_START".format(pipe.pipe_id)]
        while stack:
            for target in adjacency[stack.pop()]:
                if target not in seen:
                    seen.add(target)
                    stack.append(target)
        assert set(graph.nodes) == seen
    seen, stack = {"P0_START"}, ["P0_START"]
    while stack:
        for source in reverse[stack.pop()]:
            if source not in seen:
                seen.add(source)
                stack.append(source)
    assert set(graph.nodes) == seen


def test_metropolis_overrides_are_optional_and_standard_stays_compatible():
    env = Airspace(mode="metropolis", include_dynamic=False, num_pipes_per_layer=4, num_lanes_per_pipe=3)
    assert len(env.pipelines) == 24
    assert all(p.num_lanes == 3 for p in env.pipelines)
    standard = Airspace(mode="standard", include_dynamic=False)
    assert len(standard.pipelines) == 60
    assert standard.pipelines[0].max_speed == 20
    assert standard.pipelines[0].num_lanes == 10


@pytest.mark.parametrize("kind,setting", [("intra_layer", "intra_dynamic_level"), ("inter_layer", "inter_dynamic_level")])
def test_metropolis_dynamic_density_levels_change_real_roads_without_changing_static_backbone(kind, setting):
    counts = []
    static_reference = None
    for level, quota in (("none", 0), ("low", 1), ("medium", 2), ("high", 4), ("ultra", 8)):
        kwargs = dict(mode="metropolis", num_pipes_per_layer=4, include_dynamic=True,
                      intra_dynamic_level="none", inter_dynamic_level="none")
        kwargs[setting] = level
        env = Airspace(**kwargs)
        selected = [ramp for ramp in env.ramps if ramp.is_dynamic and ramp.ramp_type == kind]
        expected_per_quota = 6 * 4 if kind == "intra_layer" else 2 * 5 * 4
        assert len(selected) == expected_per_quota * quota
        counts.append(len(selected))
        signature = lambda ramp: (ramp.pipe_a_id, ramp.pipe_b_id, tuple(ramp.pos_a), tuple(ramp.pos_b))
        assert len({signature(ramp) for ramp in selected}) == len(selected)
        static = {signature(ramp) for ramp in env.ramps if not ramp.is_dynamic}
        if static_reference is None:
            static_reference = static
        assert static == static_reference
        assert not ({signature(ramp) for ramp in selected} & static)
    assert counts[0] == 0 < counts[1] < counts[2] < counts[3] < counts[4]


def test_metropolis_include_dynamic_false_overrides_both_density_quotas():
    env = Airspace(mode="metropolis", num_pipes_per_layer=4, include_dynamic=False,
                   intra_dynamic_level="ultra", inter_dynamic_level="ultra")
    assert not any(ramp.is_dynamic for ramp in env.ramps)


@pytest.mark.parametrize("lane_count", [20, 40])
def test_actual_metropolis_lane_endpoints_keep_full_corridors_inside_world(lane_count):
    env = Airspace(mode="metropolis", include_dynamic=False, num_lanes_per_pipe=lane_count)
    assert len(env.pipelines) == 300
    for pipe in env.pipelines:
        expected_angle = math.radians(env.layer_configs[pipe.layer_index]["angle"])
        original_direction = np.array([math.cos(expected_angle), math.sin(expected_angle), 0.])
        assert abs(np.dot(pipe.direction, original_direction)) == pytest.approx(1.)
        for lane in pipe.lanes:
            for point in (lane.start_pt, lane.end_pt):
                assert env.x_min <= point[0] <= env.x_max
                assert env.y_min <= point[1] <= env.y_max


def test_rejects_dense_bottom_launch_slots_that_violate_lane_clearance():
    with pytest.raises(ValueError, match="launch.*clearance"):
        Airspace(mode="metropolis", include_dynamic=False, num_pipes_per_layer=301)


@pytest.mark.parametrize("maximum_index", [9, 49, 300])
def test_candidate_index_normalization_scales_with_actual_layer(maximum_index):
    from uav_sim.airspace.ramp_topology import select_spatially_diverse_candidates
    def pipe(pid, index):
        return SimpleNamespace(pipe_id=pid, local_layer_index=index, length=10000.)
    candidates = [
        dict(name="origin", pipe_a=pipe(0, 0), pipe_b=pipe(100, 0), sa=0., sb=0., xy_dist=0.),
        dict(name="index", pipe_a=pipe(1, maximum_index), pipe_b=pipe(100, 0), sa=0., sb=0., xy_dist=1.),
        dict(name="position", pipe_a=pipe(2, 0), pipe_b=pipe(101, 0), sa=10000., sb=10000., xy_dist=1.),
    ]
    selected = select_spatially_diverse_candidates(candidates, 2)
    assert [candidate["name"] for candidate in selected] == ["origin", "position"]
