import math
from dataclasses import replace

import pytest

from uav_sim.airspace.partition_contracts import (
    Destination,
    PartitionGraph,
    RoadEdge,
    RoadNode,
)
from uav_sim.entities.partition_motion import PhysicalInvariantError, RoadState
from uav_sim.entities.trajectory_profile import build_prefix_profile


def straight(lengths=(100.0,), caps=(20.0,), regions=None):
    regions = regions or [0] * (len(lengths) + 1)
    x = 0.0
    nodes = {"n0": RoadNode("n0", regions[0], (0.0, 0.0, 100.0), 0, 0.0)}
    edges = {}
    for i, (length, cap) in enumerate(zip(lengths, caps)):
        previous = x
        x += length
        source, target, eid = "n%d" % i, "n%d" % (i + 1), "e%d" % i
        nodes[target] = RoadNode(target, regions[i + 1], (x, 0.0, 100.0), 0, x)
        edges[eid] = RoadEdge(
            eid,
            source,
            target,
            regions[i],
            "BOUNDARY" if length == 0 else "FLY",
            length,
            cap,
            ((previous, 0.0, 100.0), (x, 0.0, 100.0)),
            pipe_id=0,
            lane_count=1,
        )
    graph = PartitionGraph(nodes, edges, {r: {} for r in regions})
    goal = Destination(
        "n%d" % len(lengths), regions[-1], {}, pipe_id=0, position=(x, 0.0, 100.0)
    )
    state = RoadState(
        7,
        "n0",
        regions[0],
        0,
        (0.0, 0.0, 100.0),
        goal,
        route=tuple(edges),
        visited_regions=(regions[0],),
    )
    return state, graph


def test_100_meter_trapezoid_has_analytic_acceleration_cruise_and_braking():
    state, graph = straight()
    profile = build_prefix_profile(state, graph)
    assert profile.duration == pytest.approx(7.0)
    assert [(p.t0, p.t1) for p in profile.pieces] == [
        (0.0, 2.0),
        (2.0, 5.0),
        (5.0, 7.0),
    ]
    assert profile.sample(1).position == pytest.approx((5, 0, 100))
    assert profile.sample(3).position == pytest.approx((40, 0, 100))
    assert profile.sample(6).position == pytest.approx((95, 0, 100))
    assert profile.sample(6).speed == pytest.approx(10)
    assert profile.end_state.status == "ARRIVED"
    assert profile.end_state.speed == 0
    assert profile.end_state.arrival_time == pytest.approx(7.0)
    assert state.status == "WAITING" and state.position == (0.0, 0.0, 100.0)


def test_goal_can_be_reached_at_positive_speed_without_artificial_braking():
    state, graph = straight()
    profile = build_prefix_profile(state, graph, stop_at_end=False)
    assert profile.duration == pytest.approx(6.0)
    assert profile.end_state.status == "ARRIVED"
    assert profile.end_state.speed == pytest.approx(20.0)


def test_backward_envelope_brakes_before_lower_speed_entry():
    state, graph = straight((100.0, 20.0), (20.0, 5.0))
    profile = build_prefix_profile(state, graph)
    entering_second = [p for p in profile.pieces if p.position0[0] >= 100.0 - 1e-10][0]
    sampled = profile.sample(entering_second.t0)
    assert sampled.edge_id == "e1"
    assert sampled.speed == pytest.approx(5.0)
    assert any(p.acceleration[0] < 0 and p.position0[0] < 100 for p in profile.pieces)
    for i in range(1, len(profile.pieces)):
        a, b = profile.pieces[i - 1 : i + 1]
        assert a.position_at(a.t1) == pytest.approx(b.position0, abs=1e-10)
        assert a.velocity_at(a.t1) == pytest.approx(b.velocity0, abs=1e-10)


def test_short_edges_and_zero_boundaries_are_all_preserved():
    state, graph = straight(
        (0.0, 0.001, 0.0, 0.001), (20.0, 20.0, 20.0, 20.0), (0, 1, 1, 2, 2)
    )
    profile = build_prefix_profile(state, graph)
    assert profile.duration == pytest.approx(2 * math.sqrt(0.002 / 10))
    assert profile.end_state.position == pytest.approx((0.002, 0, 100))
    assert profile.end_state.route_index == 4
    assert profile.end_state.visited_regions == (0, 1, 2)
    assert profile.sample(0).region_id == 1


def test_non_goal_prefix_stops_at_boundary_and_keeps_flying_status():
    state, graph = straight((10.0, 0.0, 10.0), (20.0, 20.0, 20.0), (0, 0, 1, 1))
    profile = build_prefix_profile(state, graph, edge_ids=("e0", "e1"))
    assert profile.end_state.node_id == "n2"
    assert profile.end_state.region_id == 1
    assert profile.end_state.status == "FLYING"
    assert profile.end_state.speed == 0
    assert profile.end_state.arrival_time is None
    assert profile.end_state.visited_regions == (0, 1)


def test_fractional_departure_shift_and_fixed_grid_samples_are_exact():
    state, graph = straight()
    profile = build_prefix_profile(state, graph, start_time=0.137)
    before = profile.sample(0.1)
    assert before.status == "WAITING" and before.position == state.position
    assert profile.sample(0.137).status == "FLYING"
    shifted = profile.shifted(2.731)
    assert shifted.duration == pytest.approx(profile.duration)
    assert shifted.end_state.arrival_time == pytest.approx(9.731)
    for tick in range(71):
        dt = tick / 10
        old = profile.sample(0.137 + dt)
        new = shifted.sample(2.731 + dt)
        assert old.position == pytest.approx(new.position, abs=1e-10)
        assert old.speed == pytest.approx(new.speed, abs=1e-10)
        if tick % 10 == 0:
            coarse = profile.sample(0.137 + tick // 10)
            assert old.position == pytest.approx(coarse.position)
    assert shifted.sample(100).status == "ARRIVED"


def test_resuming_inside_edge_uses_remaining_distance_and_route_index():
    state, graph = straight((100.0, 100.0), (20.0, 20.0))
    state = replace(
        state,
        status="FLYING",
        node_id="n1",
        position=(120.0, 0.0, 100.0),
        speed=10.0,
        route_index=1,
        edge_id="e1",
        progress=20.0,
    )
    profile = build_prefix_profile(state, graph, start_time=5)
    assert profile.pieces[0].position0 == state.position
    assert profile.pieces[0].velocity0 == (10.0, 0.0, 0.0)
    assert profile.sample(5).route_index == 1
    assert profile.sample(5).progress == 20.0
    assert profile.end_state.route_index == 2
    assert profile.end_state.position == (200.0, 0.0, 100.0)


def ramp_fixture():
    points = {
        "a": (0.0, 0.0, 100.0),
        "b": (5.0, 0.0, 105.0),
        "c": (5.0, 0.0, 105.0),
        "d": (10.0, 0.0, 110.0),
    }
    nodes = {
        name: RoadNode(
            name,
            0 if name in "ab" else 1,
            p,
            1 if name == "d" else (0 if name == "a" else None),
        )
        for name, p in points.items()
    }
    entry, exit = ((0.0, 0.0, 0.0), (0.0, 2.0, 0.0)), ((0.0, 0.0, 0.0), (0.0, 6.0, 0.0))
    edges, metadata = {}, {}
    for eid, source, target, low, high, kind in [
        ("r0", "a", "b", 0.0, 0.5, "RAMP"),
        ("b", "b", "c", 0.5, 0.5, "BOUNDARY"),
        ("r1", "c", "d", 0.5, 1.0, "RAMP"),
    ]:
        edges[eid] = RoadEdge(
            eid,
            source,
            target,
            nodes[source].region_id,
            kind,
            math.dist(points[source], points[target]),
            20.0,
            (points[source], points[target]),
            ramp_id=9,
            lane_count=2,
        )
        metadata[eid] = dict(
            ramp_id=9,
            direction="reverse",
            fraction_start=low,
            fraction_end=high,
            offsets_at_entry=entry,
            offsets_at_exit=exit,
        )
    graph = PartitionGraph(
        nodes,
        edges,
        {0: {}, 1: {}},
        {"ramp_fragments": metadata},
        ramp_states={9: True},
    )
    goal = Destination("d", 1, {}, pipe_id=1, lane_id=1, position=(10.0, 6.0, 110.0))
    state = RoadState(
        7,
        "a",
        0,
        0,
        (0.0, 0.0, 100.0),
        goal,
        route=("r0", "b", "r1"),
        visited_regions=(0,),
    )
    return state, graph


def test_ramp_lane_transfer_fraction_metadata_survives_region_boundary():
    state, graph = ramp_fixture()
    profile = build_prefix_profile(state, graph)
    half_time = profile.duration / 2
    middle = profile.sample(half_time)
    assert middle.position == pytest.approx((5, 3, 105), abs=1e-10)
    assert middle.region_id == 1
    assert middle.transit_ramp == 9
    assert middle.entry_lane == 0 and middle.exit_lane == 1
    assert middle.transit_direction == "reverse"
    assert profile.end_state.lane_id == 1
    assert profile.end_state.transit_ramp is None
    assert profile.end_state.position == (10.0, 6.0, 110.0)
    assert profile.end_state.visited_regions == (0, 1)
    # True lane geometry is longer than the centerline.
    assert profile.duration == pytest.approx(2 * math.sqrt(math.sqrt(236.0) / 10))


def test_ramp_prefix_can_resume_in_transit_after_next_region_is_planned():
    state, graph = ramp_fixture()
    first = build_prefix_profile(state, graph, edge_ids=("r0", "b"))
    assert first.end_state.transit_ramp == 9
    assert first.end_state.exit_lane == 1
    second = build_prefix_profile(
        first.end_state, graph, edge_ids=("r1",), start_time=first.end_time
    )
    assert second.pieces[0].position0 == pytest.approx((5, 3, 105))
    assert second.end_state.position == (10.0, 6.0, 110.0)
    assert second.end_state.lane_id == 1


@pytest.mark.parametrize("speed", [20.0001, -1.0, math.nan])
def test_invalid_initial_speed_is_rejected_without_clipping(speed):
    state, graph = straight()
    with pytest.raises((ValueError, PhysicalInvariantError)):
        build_prefix_profile(replace(state, speed=speed), graph)


def test_insufficient_braking_distance_is_explicit():
    state, graph = straight((1.0,), (20.0,))
    with pytest.raises(PhysicalInvariantError):
        build_prefix_profile(replace(state, speed=20.0), graph)


def test_closed_ramp_entry_and_wrong_goal_lane_are_rejected():
    state, graph = ramp_fixture()
    graph.ramp_states[9] = False
    with pytest.raises(PhysicalInvariantError):
        build_prefix_profile(state, graph)
    state, graph = straight()
    with pytest.raises(PhysicalInvariantError):
        build_prefix_profile(replace(state, goal=replace(state.goal, lane_id=1)), graph)


def test_samples_are_independent_state_copies():
    state, graph = straight()
    profile = build_prefix_profile(state, graph)
    sample = profile.sample(1.0)
    sample.speed = 999
    end = profile.end_state
    end.position = (999.0, 0.0, 0.0)
    assert profile.sample(1.0).speed == 10.0
    assert profile.end_state.position == (100.0, 0.0, 100.0)


@pytest.mark.parametrize("with_alias", [False, True])
def test_heading_change_has_a_physical_stop_without_velocity_jump(with_alias):
    state, graph = straight(
        (100.0, 0.0, 100.0) if with_alias else (100.0, 100.0),
        (20.0, 20.0, 20.0) if with_alias else (20.0, 20.0),
    )
    last = "e2" if with_alias else "e1"
    target = graph.edges[last].target
    graph.nodes[target] = replace(graph.nodes[target], position=(100.0, 100.0, 100.0))
    graph.edges[last] = replace(
        graph.edges[last], points=((100.0, 0.0, 100.0), (100.0, 100.0, 100.0))
    )
    state = replace(state, goal=replace(state.goal, position=(100.0, 100.0, 100.0)))
    profile = build_prefix_profile(state, graph)
    corner = [p for p in profile.pieces if p.position0 == (100.0, 0.0, 100.0)][0]
    assert corner.velocity0 == pytest.approx((0.0, 0.0, 0.0))
    assert profile.duration == pytest.approx(14.0)
    for before, after in zip(profile.pieces, profile.pieces[1:]):
        assert before.velocity_at(before.t1) == pytest.approx(
            after.velocity0, abs=1e-10
        )
    from uav_sim.algorithms.prefix_reservations import PrefixReservations

    assert (
        PrefixReservations()
        .reserve(state.uav_id, profile.pieces, 0, cold=True)
        .accepted
    )


def test_short_segment_at_high_entry_speed_has_energy_scaled_roundoff_tolerance():
    length = 0.9088887818278824
    state, graph = straight((length,), (60.0,))
    state = replace(state, speed=34.641016151376434, status="FLYING")
    profile = build_prefix_profile(state, graph, stop_at_end=False)
    expected_speed = math.sqrt(state.speed**2 + 20 * length)
    assert profile.end_state.speed == pytest.approx(expected_speed, abs=1e-12)
    assert profile.end_state.position == (length, 0.0, 100.0)
    assert profile.pieces[-1].position_at(profile.end_time) == pytest.approx(
        profile.end_state.position, abs=1e-12
    )


def test_distance_limit_stops_inside_the_real_edge_without_false_arrival():
    state, graph = straight()
    profile = build_prefix_profile(
        state, graph, distance_limit_m=25.0, stop_at_end=False
    )
    end = profile.end_state
    assert end.position == (25.0, 0.0, 100.0)
    assert end.node_id == "n0" and end.edge_id == "e0"
    assert end.route_index == 0 and end.progress == 25.0
    assert end.region_id == 0 and end.goal == state.goal
    assert end.status == "FLYING" and end.arrival_time is None and end.speed == 0
    assert profile.duration == pytest.approx(2 * math.sqrt(25 / 10))
    resumed = build_prefix_profile(end, graph, start_time=profile.end_time)
    assert resumed.pieces[0].position0 == end.position
    assert resumed.pieces[0].velocity0 == (0.0, 0.0, 0.0)
    assert resumed.end_state.status == "ARRIVED"
    assert resumed.end_state.position == state.goal.position
    queued_again = build_prefix_profile(end, graph, distance_limit_m=25.0)
    assert queued_again.end_state.progress == 50.0
    assert queued_again.end_state.position == (50.0, 0.0, 100.0)
    assert queued_again.end_state.edge_id == "e0"


def test_distance_limit_consumes_zero_aliases_and_stops_in_later_edge():
    state, graph = straight((10.0, 0.0, 100.0), (20.0, 20.0, 20.0), (0, 0, 1, 1))
    profile = build_prefix_profile(state, graph, distance_limit_m=25.0)
    end = profile.end_state
    assert end.node_id == "n2" and end.edge_id == "e2"
    assert end.route_index == 2 and end.progress == 15.0
    assert end.position == (25.0, 0.0, 100.0)
    assert end.region_id == 1 and end.visited_regions == (0, 1)
    assert end.status == "FLYING" and end.speed == 0
    exact_boundary = build_prefix_profile(state, graph, distance_limit_m=10.0)
    assert exact_boundary.end_state.position == (10.0, 0.0, 100.0)
    assert exact_boundary.end_state.region_id == 1
    assert exact_boundary.end_state.visited_regions == (0, 1)
    assert exact_boundary.end_state.node_id == "n2"


def test_distance_limit_beyond_goal_preserves_normal_profile():
    state, graph = straight()
    normal = build_prefix_profile(state, graph, stop_at_end=False)
    limited = build_prefix_profile(
        state, graph, stop_at_end=False, distance_limit_m=200.0
    )
    assert limited.pieces == normal.pieces
    assert limited.end_state == normal.end_state


def test_distance_limit_inside_lane_changing_ramp_resumes_exactly():
    state, graph = ramp_fixture()
    first = build_prefix_profile(state, graph, distance_limit_m=5.0)
    end = first.end_state
    assert end.edge_id == "r0" and end.progress == 5.0
    assert end.node_id == "a" and end.transit_ramp == 9
    assert end.entry_lane == 0 and end.exit_lane == 1
    second = build_prefix_profile(end, graph)
    assert second.pieces[0].position0 == end.position
    assert second.end_state.position == state.goal.position
    assert second.end_state.lane_id == 1
    assert second.end_state.status == "ARRIVED"


@pytest.mark.parametrize("distance", [0.0, -1.0, math.nan, math.inf])
def test_distance_limit_requires_positive_finite_value(distance):
    state, graph = straight()
    with pytest.raises(ValueError, match="distance_limit_m"):
        build_prefix_profile(state, graph, distance_limit_m=distance)


def test_region_events_and_boundary_stops_have_exact_physical_times():
    state, graph = straight((10.0, 0.0, 10.0), (20.0, 20.0, 20.0), (0, 0, 1, 1))
    profile = build_prefix_profile(state, graph, stop_at_boundaries=True)
    assert profile.duration == pytest.approx(4.0)
    assert len(profile.region_events) == 1
    when, crossed = profile.region_events[0]
    assert when == pytest.approx(2.0)
    assert crossed.node_id == "n2" and crossed.region_id == 1
    assert crossed.route_index == 2 and crossed.edge_id == ""
    assert crossed.position == (10.0, 0.0, 100.0) and crossed.speed == 0.0
    assert crossed.visited_regions == (0, 1)
    for tick in range(41):
        fine = profile.sample(tick / 10)
        if tick % 10 == 0:
            coarse = profile.sample(tick // 10)
            assert fine.position == coarse.position
    # Existing default retains the straight-line through speed.
    normal = build_prefix_profile(state, graph)
    normal_time, normal_crossed = normal.region_events[0]
    assert normal_time == pytest.approx(math.sqrt(2.0))
    assert normal_crossed.speed == pytest.approx(math.sqrt(200.0))


def test_turn_boundary_events_shift_and_are_independent_copies():
    state, graph = straight((10.0, 0.0, 10.0), (20.0, 20.0, 20.0), (0, 0, 1, 1))
    graph.nodes["n3"] = replace(graph.nodes["n3"], position=(10.0, 10.0, 100.0))
    graph.edges["e2"] = replace(
        graph.edges["e2"], points=((10.0, 0.0, 100.0), (10.0, 10.0, 100.0))
    )
    state = replace(state, goal=replace(state.goal, position=(10.0, 10.0, 100.0)))
    profile = build_prefix_profile(state, graph, stop_at_boundaries=True)
    shifted = profile.shifted(5.137)
    event_time, event = shifted.region_events[0]
    assert event_time == pytest.approx(7.137)
    assert event.speed == 0.0 and event.position == (10.0, 0.0, 100.0)
    event.region_id = 999
    assert shifted.region_events[0][1].region_id == 1
    assert profile.region_events[0][0] == pytest.approx(2.0)


def test_partial_prefix_exposes_only_region_events_actually_traversed():
    state, graph = straight((10.0, 0.0, 10.0), (20.0, 20.0, 20.0), (0, 0, 1, 1))
    before = build_prefix_profile(
        state, graph, distance_limit_m=5.0, stop_at_boundaries=True
    )
    after = build_prefix_profile(
        state, graph, distance_limit_m=15.0, stop_at_boundaries=True
    )
    assert before.region_events == ()
    assert len(after.region_events) == 1
    assert after.region_events[0][0] == pytest.approx(2.0)
    assert after.end_state.node_id == "n2" and after.end_state.progress == 5.0
    assert after.end_state.status == "FLYING"
