"""Atomic movement of stationary wait cycles, without weakening separation."""
from copy import deepcopy

import pytest

from uav_sim.algorithms.continuous_safety import (
    TrajectoryPiece as P,
    audit_trajectories,
)
from uav_sim.algorithms.prefix_reservations import PrefixReservations
from tests.enhanced.test_prefix_reservations import stopped


def cycle():
    a = (
        P("A", 0, 1, (0, 0, 100), (0, 0, 0), (10, 0, 0)),
        P("A", 1, 4, (5, 0, 100), (10, 0, 0), (0, 0, 0)),
        P("A", 4, 5, (35, 0, 100), (10, 0, 0), (-10, 0, 0)),
    )
    b = (
        P("B", 0, 1, (20, 0, 100), (0, 0, 0), (0, 10, 0)),
        P("B", 1, 2, (20, 5, 100), (0, 10, 0), (0, 0, 0)),
        P("B", 2, 3, (20, 15, 100), (0, 10, 0), (0, -10, 0)),
        P("B", 3, 4, (20, 20, 100), (0, 0, 0), (-10, 0, 0)),
        P("B", 4, 5, (15, 20, 100), (-10, 0, 0), (0, 0, 0)),
        P("B", 5, 6, (5, 20, 100), (-10, 0, 0), (10, 0, 0)),
        P("B", 6, 7, (0, 20, 100), (0, 0, 0), (0, -10, 0)),
        P("B", 7, 10, (0, 15, 100), (0, -10, 0), (0, 0, 0)),
        P("B", 10, 11, (0, -15, 100), (0, -10, 0), (0, 10, 0)),
    )
    registry = PrefixReservations()
    registry.reserve("A", (), 0, start_position=(0, 0, 100))
    registry.reserve("B", (), 0, start_position=(20, 0, 100))
    return registry, dict(
        A=dict(pieces=a, start_position=(0, 0, 100)),
        B=dict(pieces=b, start_position=(20, 0, 100)),
    )


def test_safe_concurrent_cycle_succeeds_where_each_serial_request_is_blocked():
    from uav_sim.algorithms.joint_reservations import reserve_joint

    registry, requests = cycle()
    assert not registry.reserve("A", earliest_start=0, **requests["A"]).accepted
    assert not registry.reserve("B", earliest_start=0, **requests["B"]).accepted
    result = reserve_joint(registry, requests, 0)
    assert result.accepted and set(result.results) == {"A", "B"}
    assert not audit_trajectories(
        tuple(p for r in result.results.values() for p in r.pieces)
    )
    assert registry.metrics()["active_holds"] == 2


def test_true_head_on_swap_rejects_and_preserves_every_old_hold():
    from uav_sim.algorithms.joint_reservations import reserve_joint

    registry, _ = cycle()
    before = dict(registry._records)
    cells = deepcopy(registry._cells)
    requests = dict(
        A=dict(pieces=stopped("A", (0, 0, 100), (20, 0, 100))),
        B=dict(pieces=stopped("B", (20, 0, 100), (0, 0, 100))),
    )
    result = reserve_joint(registry, requests, 0)
    assert not result.accepted
    assert registry._records == before and registry._cells == cells
    assert registry.metrics()["active_holds"] == 2


def test_joint_profiles_still_respect_outsider_stationary_holds():
    from uav_sim.algorithms.joint_reservations import reserve_joint

    registry, requests = cycle()
    registry.reserve("OUTSIDE", (), 0, start_position=(40, 0, 100))
    before = dict(registry._records)
    result = reserve_joint(registry, requests, 0)
    assert not result.accepted and "OUTSIDE" in result.blocked_by
    assert registry._records == before


def test_cannot_cancel_an_aircrafts_already_committed_moving_profile():
    from uav_sim.algorithms.joint_reservations import reserve_joint

    registry = PrefixReservations()
    registry.reserve("A", stopped("A", (0, 0, 100), (40, 0, 100)), 0, cold=True)
    before = dict(registry._records)
    with pytest.raises(ValueError, match="stationary"):
        reserve_joint(
            registry, dict(A=dict(pieces=stopped("A", (0, 0, 100), (20, 0, 100)))), 1
        )
    assert registry._records == before


def test_find_only_actual_stationary_wait_cycles_without_python_recursion():
    from uav_sim.algorithms.joint_reservations import wait_cycles

    waiting = {uid: ((uid + 1) % 5000,) for uid in range(5000)}
    assert len(wait_cycles(waiting)) == 1
    assert len(wait_cycles(waiting)[0]) == 5000
    assert wait_cycles(waiting, eligible=set(range(1, 5000))) == ()
    assert wait_cycles({"a": ("b",), "b": ("a",), "c": ("b",), "d": ("e",)}) == (
        ("a", "b"),
    )
    assert len(wait_cycles({None: ("b",), "b": (None,)})) == 1
    assert wait_cycles({"a": (None,), None: ("b",), "b": ()}) == ()


def test_joint_commit_does_not_reuse_ids_still_in_the_retirement_heap():
    from uav_sim.algorithms.joint_reservations import reserve_joint

    registry, requests = cycle()
    registry.reserve(
        "OLD", stopped("OLD", (100, 0, 100), (110, 0, 100), 2), 0, cold=True
    )
    registry.release("OLD")
    result = reserve_joint(registry, requests, 0)
    assert result.accepted
    registry.expire(2.1)
    remaining = registry.nearby(result.results["A"].motion_pieces)
    assert any(
        record.uav_id == "A" and record.t0 == 1 and record.t1 == 4
        for record in remaining
    )
