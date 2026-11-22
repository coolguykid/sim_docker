"""Continuous 3D safety tests, including independent scalar minimization oracles."""

import math
from time import perf_counter

import numpy as np
import pytest
from scipy.optimize import minimize_scalar

from uav_sim.algorithms.continuous_safety import (
    TrajectoryPiece, audit_trajectories, min_distance,
)


def piece(uid, position, velocity=(0, 0, 0), acceleration=(0, 0, 0), start=0, end=1):
    return TrajectoryPiece(uid, start, end, position, velocity, acceleration)


def test_piece_uses_time_since_its_own_start():
    p = piece(1, (1, 2, 3), (4, 5, 6), (2, 0, -2), start=10, end=12)
    assert p.position_at(11) == pytest.approx((6, 7, 8))
    assert p.velocity_at(11) == pytest.approx((6, 5, 4))
    with pytest.raises(ValueError):
        p.position_at(9)


def test_overtaking_inside_step_is_detected_when_endpoints_are_safe():
    a = piece(1, (0, 0, 0), (100, 0, 0))
    b = piece(2, (50, 0, 0))
    distance, time = min_distance(a, b)
    assert distance == pytest.approx(0)
    assert time == pytest.approx(0.5)
    conflicts = audit_trajectories([a, b], 8)
    assert len(conflicts) == 1
    assert conflicts[0].time_s == pytest.approx(0.5)


@pytest.mark.parametrize("p2,v2", [
    ((0, -10, 0), (0, 20, 0)),  # perpendicular crossing
    ((-10, 10, 0), (20, -20, 0)),  # lane crossing
    ((0, 0, -10), (0, 0, 20)),  # ramp/height crossing
])
def test_intersections_are_detected_in_three_dimensions(p2, v2):
    a = piece("a", (-10, 0, 0), (20, 0, 0))
    b = piece("b", p2, v2)
    assert min_distance(a, b) == pytest.approx((0, 0.5))


def test_no_temporal_overlap_has_no_conflict():
    a = piece(1, (0, 0, 0), start=0, end=1)
    b = piece(2, (0, 0, 0), start=2, end=3)
    assert min_distance(a, b) == (math.inf, None)
    assert audit_trajectories([a, b], 8) == []


def test_touching_time_and_distance_threshold_are_inclusive():
    a = piece(1, (0, 0, 0), start=0, end=1)
    b = piece(2, (8, 0, 0), start=1, end=2)
    conflicts = audit_trajectories([a, b], 8)
    assert len(conflicts) == 1
    assert conflicts[0].distance_m == 8
    assert conflicts[0].time_s == 1


def test_pair_uses_larger_height_threshold_and_deduplicates_pieces():
    trajectories = [piece("low", (0, 0, 0)), piece("high", (14, 0, 0)),
                    piece("low", (0, 0, 0), start=1, end=2),
                    piece("high", (13, 0, 0), start=1, end=2)]
    conflicts = audit_trajectories(trajectories, {"low": 8, "high": 15})
    assert len(conflicts) == 1
    assert {conflicts[0].uav_id_a, conflicts[0].uav_id_b} == {"low", "high"}
    assert conflicts[0].required_distance_m == 15
    assert conflicts[0].distance_m == 13
    assert conflicts[0].time_s == 1


def test_same_vehicle_is_never_a_conflict():
    assert audit_trajectories([piece(1, (0, 0, 0)), piece(1, (0, 0, 0))], 8) == []


def test_stationary_occupants_report_earliest_time():
    assert min_distance(piece(1, (0, 0, 0)), piece(2, (3, 4, 0))) == (5.0, 0.0)


def test_zero_length_occupancy_interval_is_valid():
    a = piece(1, (0, 0, 0), start=0.3, end=0.3)
    b = piece(2, (0, 0, 0))
    assert min_distance(a, b) == (0, 0.3)
    assert len(audit_trajectories([a, b], 0)) == 1


def test_accelerating_tangent_repeated_root_is_found():
    # x(t)=(t-.5)^2; squared-distance derivative has a triple root at .5.
    a = piece(1, (0.25, 0, 0), (-1, 0, 0), (2, 0, 0))
    b = piece(2, (0, 0, 0))
    assert min_distance(a, b) == pytest.approx((0, 0.5), abs=1e-12)


def test_tiny_acceleration_retains_near_linear_minimum():
    a = piece(1, (-1, 1, 0), (2, 0, 0), (1e-22, 0, 0))
    assert min_distance(a, piece(2, (0, 0, 0))) == pytest.approx((1, 0.5))


def test_swept_bounds_include_interior_parabola_extremum():
    a = piece(1, (25, 0, 0), (-100, 0, 0), (200, 0, 0))
    b = piece(2, (0, 0, 0))
    assert len(audit_trajectories([a, b], 0)) == 1


def test_random_accelerated_minima_match_independent_bracket_oracle():
    rng = np.random.default_rng(1789)
    for index in range(60):
        start_a, start_b = rng.uniform(-0.5, 0, 2)
        position_a, position_b = rng.uniform(-30, 30, (2, 3))
        velocity_a, velocity_b = rng.uniform(-40, 40, (2, 3))
        acceleration_a, acceleration_b = rng.uniform(-50, 50, (2, 3))
        a = piece("a", position_a, velocity_a, acceleration_a, start_a, 1)
        b = piece("b", position_b, velocity_b, acceleration_b, start_b, 1)
        left = max(start_a, start_b)
        def squared_distance(t):
            # Independent position formula, without position_at or polynomial roots.
            da, db = t - start_a, t - start_b
            r = (position_a + velocity_a * da + 0.5 * acceleration_a * da**2
                 - position_b - velocity_b * db - 0.5 * acceleration_b * db**2)
            return float(r @ r)
        grid = np.linspace(left, 1, 101)
        values = [squared_distance(t) for t in grid]
        candidates = [(values[0], left), (values[-1], 1)]
        # A minimum can sit between an endpoint and its nearest grid sample.
        for bracket in ((grid[0], grid[1]), (grid[-2], grid[-1])):
            result = minimize_scalar(squared_distance, bounds=bracket, method="bounded",
                                     options={"xatol": 1e-13})
            candidates.append((result.fun, result.x))
        for i in range(1, 100):
            if values[i] <= min(values[i - 1], values[i + 1]):
                result = minimize_scalar(squared_distance, bounds=(grid[i-1], grid[i+1]), method="bounded",
                                         options={"xatol": 1e-13})
                candidates.append((result.fun, result.x))
        expected_sq, expected_time = min(candidates)
        distance, time = min_distance(a, b)
        assert distance == pytest.approx(math.sqrt(expected_sq), abs=1e-7), index
        assert time == pytest.approx(expected_time, abs=1e-6), index


def test_ten_thousand_sparse_vehicles_do_not_require_pairwise_root_solves(monkeypatch):
    import uav_sim.algorithms.continuous_safety as safety
    def unexpected_pair(*_):
        pytest.fail("spatially separate trajectories reached narrow-phase root solver")
    monkeypatch.setattr(safety, "min_distance", unexpected_pair)
    trajectories = [piece(i, (0, 0, i * 100)) for i in range(10000)]
    started = perf_counter()
    assert safety.audit_trajectories(trajectories, 30) == []
    assert perf_counter() - started < 5.0


def test_broadphase_matches_all_pairs_oracle_for_accelerated_fleet():
    rng = np.random.default_rng(1469)
    trajectories = [piece(i, *rng.uniform(-30, 30, (3, 3))) for i in range(40)]
    thresholds = {i: float(rng.uniform(0, 15)) for i in range(40)}
    expected = set()
    for i, a in enumerate(trajectories):
        for b in trajectories[i + 1:]:
            distance, _ = min_distance(a, b)
            if distance <= max(thresholds[a.uav_id], thresholds[b.uav_id]):
                expected.add(frozenset((a.uav_id, b.uav_id)))
    actual = {frozenset((c.uav_id_a, c.uav_id_b))
              for c in audit_trajectories(trajectories, thresholds)}
    assert actual == expected


def test_one_second_and_ten_subinterval_safety_have_same_closest_approach():
    a = piece(1, (-5, 1, 0), (20, 0, 0), (-10, 0, 0))
    b = piece(2, (0, 0, 0))
    expected = audit_trajectories([a, b], 8)[0]
    subintervals = []
    for i in range(10):
        t0, t1 = i / 10, (i + 1) / 10
        for original in (a, b):
            subintervals.append(piece(original.uav_id, original.position_at(t0), original.velocity_at(t0),
                                      original.acceleration, t0, t1))
    actual = audit_trajectories(subintervals, 8)
    assert len(actual) == 1
    assert actual[0].distance_m == pytest.approx(expected.distance_m, abs=1e-12)
    assert actual[0].time_s == pytest.approx(expected.time_s, abs=1e-12)


@pytest.mark.parametrize("kwargs", [
    {"start": math.nan}, {"end": math.inf}, {"start": 2, "end": 1},
    {"position": (math.inf, 0, 0)}, {"position": (0, 0)},
    {"velocity": (0, math.nan, 0)}, {"acceleration": (0, 0, math.inf)},
])
def test_trajectory_rejects_nonfinite_or_malformed_input(kwargs):
    arguments = {"uid": 1, "position": (0, 0, 0)}
    arguments.update(kwargs)
    with pytest.raises(ValueError):
        piece(**arguments)


@pytest.mark.parametrize("threshold", [-1, math.inf, math.nan, {1: 8}, {1: 8, 2: -1}])
def test_thresholds_are_explicit_and_validated(threshold):
    with pytest.raises(ValueError):
        audit_trajectories([piece(1, (0, 0, 0)), piece(2, (100, 0, 0))], threshold)
