"""Physical examples derived independently from the implementation."""

import math

import pytest

from uav_sim.algorithms.continuous_motion import integrate_motion, time_to_distance


def test_accelerates_to_cap_then_keeps_remaining_time():
    result = integrate_motion(9.0, 4.0, 1.0, 10.0)
    assert result.distance_m == pytest.approx(9.875)
    assert result.speed_mps == 10.0
    assert len(result.pieces) == 2
    assert [(p.t0, p.t1) for p in result.pieces] == [(0.0, 0.25), (0.25, 1.0)]
    assert result.pieces[1].distance0_m == pytest.approx(2.375)
    assert result.pieces[1].accel_mps2 == 0.0


def test_brakes_to_stop_then_retains_stationary_time():
    result = integrate_motion(3.0, -10.0, 1.0, 20.0)
    assert result.distance_m == pytest.approx(0.45)
    assert result.speed_mps == 0.0
    assert [(p.t0, p.t1) for p in result.pieces] == [(0.0, 0.3), (0.3, 1.0)]
    assert result.pieces[1].speed0_mps == 0.0


@pytest.mark.parametrize("speed,acceleration,cap", [(3.0, 10.0, 8.0), (3.0, -10.0, 8.0), (7.0, 0.0, 8.0)])
def test_one_second_matches_ten_substeps(speed, acceleration, cap):
    whole = integrate_motion(speed, acceleration, 1.0, cap)
    distance = 0.0
    for _ in range(10):
        part = integrate_motion(speed, acceleration, 0.1, cap)
        speed = part.speed_mps
        distance += part.distance_m
    assert distance == pytest.approx(whole.distance_m, abs=1e-12)
    assert speed == pytest.approx(whole.speed_mps, abs=1e-12)


def test_arbitrary_event_duration_is_not_rounded_to_tick():
    result = integrate_motion(2.0, 3.0, 0.137, 8.0)
    assert result.distance_m == pytest.approx(2 * 0.137 + 1.5 * 0.137**2)
    assert result.speed_mps == pytest.approx(2 + 3 * 0.137)


@pytest.mark.parametrize("speed,accel,distance,cap,expected", [
    (0, 2, 9, 10, 3), (9, 4, 9.875, 10, 1),
    (3, -10, 0.45, 20, 0.3), (3, -10, 0.46, 20, math.inf),
    (0, 0, 1, 10, math.inf), (0, 2, 1, 0, math.inf),
    (4, 0, 2, 10, 0.5), (0, 0, 0, 0, 0),
])
def test_time_to_distance_handles_cap_stop_and_unreachable(speed, accel, distance, cap, expected):
    assert time_to_distance(speed, accel, distance, cap) == pytest.approx(expected)


def test_zero_duration_has_no_fake_motion_piece():
    result = integrate_motion(2, 3, 0, 4)
    assert result.distance_m == 0.0
    assert result.speed_mps == 2.0
    assert result.pieces == ()


@pytest.mark.parametrize("args", [
    (-1, 1, 1, 5), (6, -1, 1, 5), (5.0000000001, -1, 1, 5),
    (1, 1, -1, 5), (1, 1, 1, -1),
    (math.nan, 1, 1, 5), (1, math.inf, 1, 5), (1, 1, math.nan, 5),
    (1, 1, 1, math.inf),
])
def test_motion_rejects_invalid_physics(args):
    with pytest.raises(ValueError):
        integrate_motion(*args)


@pytest.mark.parametrize("args", [(1, 1, -1, 5), (6, -1, 1, 5), (1, 1, math.nan, 5)])
def test_arrival_time_rejects_invalid_physics(args):
    with pytest.raises(ValueError):
        time_to_distance(*args)


def test_arrival_time_at_integrated_stop_is_not_lost_to_roundoff():
    distance = integrate_motion(3, -10, 1, 20).distance_m
    assert time_to_distance(3, -10, distance, 20) == pytest.approx(0.3, abs=1e-14)


@pytest.mark.parametrize("speed", [6.0, 3.0, 1e-8, 1e-12])
@pytest.mark.parametrize("direction", [-math.inf, math.inf])
def test_arrival_within_32_stop_distance_ulps_reaches_exact_stop(speed, direction):
    stop_time = speed / 10.0
    stop_distance = 0.5 * speed * stop_time
    distance = stop_distance
    for _ in range(32):
        distance = math.nextafter(distance, direction)
    arrival = time_to_distance(speed, -10, distance, 20)
    assert arrival == stop_time
    assert integrate_motion(speed, -10, arrival, 20).speed_mps == 0


def test_stop_tolerance_does_not_snap_goal_ten_micrometers_before_stop():
    arrival = time_to_distance(6, -10, 1.8 - 1e-5, 20)
    result = integrate_motion(6, -10, arrival, 20)
    assert arrival < 0.6
    assert result.speed_mps == pytest.approx(math.sqrt(2 * 10 * 1e-5), rel=1e-8)
    assert result.distance_m == pytest.approx(1.8 - 1e-5, abs=1e-14)


def test_stop_tolerance_remains_relative_for_physically_tiny_stopping_distance():
    speed = 1e-8
    stop_time = speed / 10
    stop_distance = 0.5 * speed * stop_time
    arrival = time_to_distance(speed, -10, 0.25 * stop_distance, 20)
    result = integrate_motion(speed, -10, arrival, 20)
    assert arrival == pytest.approx(stop_time * (1 - math.sqrt(0.75)), rel=1e-12)
    assert result.speed_mps == pytest.approx(speed * math.sqrt(0.75), rel=1e-12)
