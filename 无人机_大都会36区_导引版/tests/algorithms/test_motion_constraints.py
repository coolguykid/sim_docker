from types import SimpleNamespace

from uav_sim.airspace import Airspace
from uav_sim.algorithms.motion_constraints import (
    DEFAULT_UAV_MAX_ACCEL,
    DEFAULT_UAV_MAX_DECEL,
    braking_distance,
    clamp_speed_change,
    compute_acceleration,
    max_accel_for_uav,
    max_decel_for_uav,
    normalized_acceleration,
)
from uav_sim.entities.uav import UAV


def test_defaults_are_fixed_ten_meters_per_second_squared():
    bare = SimpleNamespace()
    assert DEFAULT_UAV_MAX_ACCEL == 10.0
    assert DEFAULT_UAV_MAX_DECEL == 10.0
    assert max_accel_for_uav(bare) == 10.0
    assert max_decel_for_uav(bare) == 10.0


def test_clamps_acceleration_by_uav_limit_not_pipe_speed():
    next_speed, accel = clamp_speed_change(
        current_speed=20.0,
        desired_speed=100.0,
        pipe_max_speed=100.0,
        max_accel=10.0,
        max_decel=10.0,
        dt=1.0,
    )
    assert next_speed == 30.0
    assert accel == 10.0


def test_clamps_deceleration_by_uav_limit_not_pipe_speed():
    next_speed, accel = clamp_speed_change(
        current_speed=40.0,
        desired_speed=0.0,
        pipe_max_speed=100.0,
        max_accel=10.0,
        max_decel=10.0,
        dt=1.0,
    )
    assert next_speed == 30.0
    assert accel == -10.0


def test_speed_never_exceeds_pipe_limit_after_acceleration():
    next_speed, accel = clamp_speed_change(
        current_speed=95.0,
        desired_speed=120.0,
        pipe_max_speed=100.0,
        max_accel=10.0,
        max_decel=10.0,
        dt=1.0,
    )
    assert next_speed == 100.0
    assert accel == 5.0


def test_braking_distance_uses_kinematic_formula():
    assert braking_distance(20.0, 10.0) == 20.0


def test_compute_acceleration_uses_dt():
    assert compute_acceleration(10.0, 16.0, dt=2.0) == 3.0


def test_normalized_acceleration_supports_negative_braking():
    assert normalized_acceleration(-10.0, max_accel=10.0, max_decel=10.0) == -1.0
    assert normalized_acceleration(5.0, max_accel=10.0, max_decel=10.0) == 0.5


def test_uav_owns_fixed_accel_and_decel_limits():
    env = Airspace(include_dynamic=False)
    uav = UAV(0, 0, 0, 1, env, capacity=0)
    assert uav.max_accel == 10.0
    assert uav.max_decel == 10.0
    assert uav.last_acceleration == 0.0


def test_uav_apply_speed_command_respects_accel_limit():
    env = Airspace(include_dynamic=False)
    uav = UAV(0, 10, 0, 1, env, capacity=0)
    uav.speed = 20.0
    uav.apply_speed_command(100.0)
    assert uav.speed == 30.0
    assert uav.last_acceleration == 10.0


def test_uav_apply_speed_command_respects_decel_limit():
    env = Airspace(include_dynamic=False)
    uav = UAV(0, 30, 0, 1, env, capacity=0)
    uav.speed = 40.0
    uav.apply_speed_command(0.0)
    assert uav.speed == 30.0
    assert uav.last_acceleration == -10.0

