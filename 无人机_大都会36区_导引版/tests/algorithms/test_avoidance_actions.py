from uav_sim.algorithms.avoidance_actions import (
    ACTION_ACCEL_FULL,
    ACTION_ACCEL_LIGHT,
    ACTION_BRAKE_FULL,
    ACTION_BRAKE_LIGHT,
    ACTION_KEEP,
    desired_speed_for_action,
)


def test_acceleration_actions_use_uav_accel_limit():
    assert desired_speed_for_action(
        action=ACTION_ACCEL_LIGHT,
        current_speed=20.0,
        pipe_max_speed=100.0,
        max_accel=10.0,
        max_decel=10.0,
        dt=1.0,
    ) == 25.0
    assert desired_speed_for_action(
        action=ACTION_ACCEL_FULL,
        current_speed=20.0,
        pipe_max_speed=100.0,
        max_accel=10.0,
        max_decel=10.0,
        dt=1.0,
    ) == 30.0


def test_brake_actions_use_uav_decel_limit():
    assert desired_speed_for_action(
        action=ACTION_BRAKE_LIGHT,
        current_speed=20.0,
        pipe_max_speed=100.0,
        max_accel=10.0,
        max_decel=10.0,
        dt=1.0,
    ) == 15.0
    assert desired_speed_for_action(
        action=ACTION_BRAKE_FULL,
        current_speed=20.0,
        pipe_max_speed=100.0,
        max_accel=10.0,
        max_decel=10.0,
        dt=1.0,
    ) == 10.0


def test_keep_action_respects_current_speed_limit():
    assert desired_speed_for_action(
        action=ACTION_KEEP,
        current_speed=30.0,
        pipe_max_speed=100.0,
        max_accel=10.0,
        max_decel=10.0,
        dt=1.0,
        current_speed_limit=18.0,
    ) == 18.0

