"""The optional compiled solver must retain the reference continuous test."""
import math
import random

import pytest

from uav_sim.algorithms.continuous_safety import TrajectoryPiece, min_distance


def test_compiled_closest_approach_matches_seeded_reference_quadratics():
    from uav_sim.algorithms.fast_continuous_safety import fast_min_distance

    rng = random.Random(20903)
    for index in range(3000):
        scale = 10.0 ** rng.uniform(-8, 8)
        begin = rng.uniform(-100, 100)
        duration = 10.0 ** rng.uniform(-4, 3)

        def make(uid):
            return TrajectoryPiece(
                uid,
                begin + rng.uniform(-0.1, 0.1) * duration,
                begin + duration,
                tuple(rng.uniform(-30, 30) * scale for _ in range(3)),
                tuple(rng.uniform(-10, 10) * scale / duration for _ in range(3)),
                tuple(rng.uniform(-3, 3) * scale / duration**2 for _ in range(3)),
            )

        a, b = make(0), make(1)
        from uav_sim.algorithms.continuous_safety import reference_min_distance

        expected = reference_min_distance(a, b)
        actual = fast_min_distance(a, b)
        assert actual[0] == pytest.approx(
            expected[0], rel=3e-13, abs=1e-16 * scale
        ), index
        assert actual[1] == pytest.approx(
            expected[1], rel=1e-12, abs=duration * 1e-10
        ), index


@pytest.mark.parametrize("distance", [0.0, 8.0, 10.0, 15.0, 20.0, 25.0, 30.0])
def test_compiled_solver_preserves_tangency_and_inclusive_safety_boundary(distance):
    from uav_sim.algorithms.fast_continuous_safety import fast_min_distance

    a = TrajectoryPiece(0, 0, 1, (0.25, distance, 0), (-1, 0, 0), (2, 0, 0))
    b = TrajectoryPiece(1, 0, 1, (0, 0, 0), (0, 0, 0), (0, 0, 0))
    value, time = fast_min_distance(a, b)
    assert value <= distance + 16 * math.ulp(max(1.0, distance))
    assert time == pytest.approx(min_distance(a, b)[1], abs=1e-12)
    # A repeated derivative root has a flat fourth-order minimum; the
    # reference may choose an equally rounded point a few microseconds early.
    assert time == pytest.approx(0.5, abs=2e-6)


def test_compiled_solver_retains_tiny_orthogonal_distance_and_no_time_overlap():
    from uav_sim.algorithms.fast_continuous_safety import fast_min_distance

    a = TrajectoryPiece(0, 0, 1, (-1, 1e-200, 0), (2, 0, 0), (0, 0, 0))
    b = TrajectoryPiece(1, 0, 1, (0, 0, 0), (0, 0, 0), (0, 0, 0))
    assert fast_min_distance(a, b)[0] == 1e-200
    c = TrajectoryPiece(2, 2, 3, (0, 0, 0), (0, 0, 0), (0, 0, 0))
    assert fast_min_distance(a, c) == (math.inf, None)


def test_compiled_solver_rejects_overflowing_relative_geometry():
    from uav_sim.algorithms.fast_continuous_safety import fast_min_distance

    a = TrajectoryPiece(0, 0, 1, (1e308, 0, 0), (1e308, 0, 0), (0, 0, 0))
    b = TrajectoryPiece(1, 0, 1, (-1e308, 0, 0), (0, 0, 0), (0, 0, 0))
    with pytest.raises(ValueError):
        fast_min_distance(a, b)


def test_public_solver_keeps_an_independent_python_reference_and_optional_fallback(
    monkeypatch,
):
    from uav_sim.algorithms import continuous_safety as module

    reference = module.reference_min_distance
    a = TrajectoryPiece(0, 0, 1, (-5, 1, 0), (20, 0, 0), (-10, 0, 0))
    b = TrajectoryPiece(1, 0, 1, (0, 0, 0), (0, 0, 0), (0, 0, 0))
    expected = reference(a, b)
    monkeypatch.setattr(module, "COMPILED_AVAILABLE", False)
    assert module.min_distance(a, b) == expected


def test_equal_acceleration_closed_form_matches_reference_at_large_and_small_scales():
    from uav_sim.algorithms.fast_continuous_safety import fast_min_distance
    from uav_sim.algorithms.continuous_safety import reference_min_distance

    rng = random.Random(38191)
    for index in range(1500):
        scale = 10.0 ** rng.uniform(-100, 100)
        acceleration = tuple(rng.uniform(-1, 1) * scale for _ in range(3))
        a = TrajectoryPiece(
            0,
            -0.3,
            1.0,
            tuple(rng.uniform(-20, 20) * scale for _ in range(3)),
            tuple(rng.uniform(-10, 10) * scale for _ in range(3)),
            acceleration,
        )
        b = TrajectoryPiece(
            1,
            0.0,
            2.0,
            tuple(rng.uniform(-20, 20) * scale for _ in range(3)),
            tuple(rng.uniform(-10, 10) * scale for _ in range(3)),
            acceleration,
        )
        expected = reference_min_distance(a, b)
        actual = fast_min_distance(a, b)
        assert actual[0] == pytest.approx(expected[0], rel=1e-13, abs=1e-16 * scale)
        assert actual[1] == pytest.approx(expected[1], abs=1e-12)
