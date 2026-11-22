"""Exact space-time prefix reservations and atomic boundary-hold replacement."""
import math

import pytest

from uav_sim.airspace.partition_contracts import PartitionGraph
from uav_sim.algorithms.continuous_safety import TrajectoryPiece, audit_trajectories
from uav_sim.distributed.authoritative_control_server import AuthoritativeControlCore


def book(**kwargs):
    controller = AuthoritativeControlCore.for_partition(PartitionGraph({}, {}, {}))
    result = getattr(controller, "prefix_reservations", None)
    assert (
        result is not None
    ), "Existing authoritative core must expose prefix reservations"
    return result if not kwargs else type(result)(**kwargs)


def stopped(uid, start, end, duration=4.0):
    vector = tuple(b - a for a, b in zip(start, end))
    half = duration / 2
    acceleration = tuple(x / (half * half) for x in vector)
    midpoint = tuple((a + b) / 2 for a, b in zip(start, end))
    velocity = tuple(a * half for a in acceleration)
    return (
        TrajectoryPiece(uid, 0, half, start, (0, 0, 0), acceleration),
        TrajectoryPiece(
            uid, half, duration, midpoint, velocity, tuple(-a for a in acceleration)
        ),
    )


def test_crossing_prefixes_receive_continuously_safe_shifted_start_times():
    reservations = book()
    first = reservations.reserve(
        1, stopped(1, (-20, 0, 100), (20, 0, 100)), 0, cold=True
    )
    second = reservations.reserve(
        2, stopped(2, (0, -20, 100), (0, 20, 100)), 0, cold=True
    )
    assert first.accepted and second.accepted
    assert second.start_time > 0
    assert not audit_trajectories(first.pieces + second.pieces, 8)
    assert second.end_time == pytest.approx(second.start_time + 4)


def test_independent_lanes_and_long_ramp_followers_reserve_concurrently():
    reservations = book()
    first = reservations.reserve(
        1, stopped(1, (0, 0, 100), (1000, 0, 100), 20), 0, cold=True
    )
    second = reservations.reserve(
        2, stopped(2, (-100, 0, 100), (900, 0, 100), 20), 0, cold=True
    )
    third = reservations.reserve(
        3, stopped(3, (0, 20, 100), (1000, 20, 100), 20), 0, cold=True
    )
    assert first.accepted and second.accepted and third.accepted
    assert (first.start_time, second.start_time, third.start_time) == (0, 0, 0)


def test_infinite_boundary_hold_blocks_then_successful_next_prefix_replaces_it():
    reservations = book()
    held = reservations.reserve(1, stopped(1, (-20, 0, 100), (0, 0, 100)), 0, cold=True)
    blocked = reservations.reserve(
        2, stopped(2, (0, -20, 100), (0, 20, 100)), 5, cold=True
    )
    assert held.accepted and not blocked.accepted and blocked.deferred
    assert 1 in blocked.blocked_by
    next_prefix = reservations.reserve(1, stopped(1, (0, 0, 100), (20, 0, 100)), 5)
    assert next_prefix.accepted
    retry = reservations.reserve(
        2, stopped(2, (0, -20, 100), (0, 20, 100)), 5, cold=True
    )
    assert retry.accepted
    assert not audit_trajectories(next_prefix.pieces + retry.pieces, 8)


def test_failed_replacement_retains_the_source_hold_atomically():
    reservations = book()
    assert reservations.reserve(1, (), 0, start_position=(0, 0, 100)).accepted
    assert reservations.reserve(3, (), 0, start_position=(20, 0, 100)).accepted
    failed = reservations.reserve(1, stopped(1, (0, 0, 100), (20, 0, 100)), 1)
    assert not failed.accepted
    crossing = reservations.reserve(
        2, stopped(2, (0, -20, 100), (0, 20, 100)), 2, cold=True
    )
    assert not crossing.accepted and 1 in crossing.blocked_by


def test_sink_release_removes_future_hold_and_allows_waiters():
    reservations = book()
    assert reservations.reserve(1, (), 0, start_position=(0, 0, 100)).accepted
    reservations.release(1, now=3)
    accepted = reservations.reserve(
        2, stopped(2, (0, -20, 100), (0, 20, 100)), 3, cold=True
    )
    assert accepted.accepted and accepted.start_time == 3


def test_goal_sink_has_no_stationary_tail_after_arrival():
    reservations = book()
    first = reservations.reserve(
        1, stopped(1, (-20, 0, 100), (0, 0, 100)), 0, end_hold=False, cold=True
    )
    second = reservations.reserve(
        2, stopped(2, (0, -20, 100), (0, 20, 100)), 5, cold=True
    )
    assert first.accepted and second.accepted and second.start_time == 5


def test_airborne_waiting_is_audited_while_cold_waiting_is_outside_airspace():
    reservations = book()
    assert reservations.reserve(
        1, stopped(1, (-20, 0, 100), (20, 0, 100)), 0, cold=True
    ).accepted
    candidate = stopped(2, (0, 0, 100), (0, 20, 100))
    warm = reservations.reserve(2, candidate, 1, cold=False)
    assert not warm.accepted
    cold = reservations.reserve(2, candidate, 1, cold=True)
    assert cold.accepted and cold.start_time > 1


def test_height_threshold_can_vary_by_phase_without_using_distant_goal_height():
    reservations = book()
    a = reservations.reserve(
        1, stopped(1, (0, 0, 100), (100, 0, 100)), 0, cold=True, thresholds=(8, 8)
    )
    b = reservations.reserve(
        2, stopped(2, (0, 10, 100), (100, 10, 100)), 0, cold=True, thresholds=(8, 8)
    )
    assert a.accepted and b.accepted and b.start_time == 0
    high = reservations.reserve(3, (), 0, start_position=(0, 0, 200), threshold=10)
    nearby = reservations.reserve(4, (), 0, start_position=(0, 9, 200), threshold=8)
    assert high.accepted and not nearby.accepted


def test_expiring_past_phases_preserves_stationary_end_occupancy():
    reservations = book()
    assert reservations.reserve(
        1, stopped(1, (-20, 0, 100), (0, 0, 100)), 0, cold=True
    ).accepted
    reservations.update(1, 10)
    assert reservations.metrics()["active_finite_phases"] == 0
    assert reservations.metrics()["active_holds"] == 1
    blocked = reservations.reserve(
        2, stopped(2, (0, -20, 100), (0, 20, 100)), 10, cold=True
    )
    assert not blocked.accepted and blocked.blocked_by == (1,)


def test_nearby_query_ignores_own_old_profile_and_distant_aircraft():
    reservations = book()
    reservations.reserve(1, stopped(1, (-20, 0, 100), (20, 0, 100)), 0, cold=True)
    reservations.reserve(2, stopped(2, (-20, 1000, 100), (20, 1000, 100)), 0, cold=True)
    query = stopped(3, (0, -20, 100), (0, 20, 100))
    found = reservations.nearby(query)
    assert {item.uav_id for item in found} == {1}
    assert not reservations.nearby(query, ignore_uid=1)


def test_maximum_wait_is_bounded_and_pending_requests_have_age_priority():
    reservations = book(max_wait_seconds=0.01)
    reservations.reserve(1, stopped(1, (-20, 0, 100), (20, 0, 100)), 0, cold=True)
    blocked = reservations.reserve(
        2, stopped(2, (0, -20, 100), (0, 20, 100)), 0, cold=True
    )
    assert not blocked.accepted
    assert "wait" in blocked.reason.lower()
    assert reservations.priority_key(2, 100) < reservations.priority_key(3, 100)


def test_hold_requires_an_actual_stationary_endpoint():
    reservations = book()
    moving = (TrajectoryPiece(1, 0, 1, (0, 0, 100), (10, 0, 0), (0, 0, 0)),)
    with pytest.raises(ValueError, match="(?i)(stop|stationary)"):
        reservations.reserve(1, moving, 0)


def test_sparse_diagonal_prefixes_do_not_force_global_candidate_scans():
    reservations = book()
    for uid in range(100):
        offset = uid * 20000.0
        pieces = (
            TrajectoryPiece(
                uid,
                0.0,
                100.0,
                (offset, 0.0, 100.0),
                (50.0, 50.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
        )
        assert reservations.reserve(
            uid, pieces, 0.0, cold=True, end_hold=False
        ).accepted
    metrics = reservations.metrics()
    assert metrics["spatial_candidate_count"] < 100
    assert metrics["large_phase_count"] == 0


def test_curve_cell_index_keeps_intermediate_extrema_and_all_contact_points():
    reservations = book()
    curve = TrajectoryPiece(
        1, 0.0, 100.0, (0.0, 0.0, 100.0), (100.0, 100.0, 0.0), (-2.0, 0.0, 0.0)
    )
    assert reservations.reserve(1, (curve,), 0.0, cold=True, end_hold=False).accepted
    for index in range(101):
        point = curve.position_at(index)
        result = reservations.reserve(index + 2, (), index, start_position=point)
        assert not result.accepted, "missed physical contact at t=%s" % index
        assert 1 in result.blocked_by


def test_wait_search_retimes_only_the_conflicting_phase(monkeypatch):
    import uav_sim.algorithms.prefix_reservations as module

    reservations = book()
    first = reservations.reserve(
        1, stopped(1, (-20, 0, 100), (20, 0, 100)), 0, cold=True, end_hold=False
    )
    candidate = []
    for index in range(20):
        start_y = -20 + index * 40
        for piece in stopped(2, (0, start_y, 100), (0, start_y + 40, 100)):
            candidate.append(
                TrajectoryPiece(
                    2,
                    piece.t0 + index * 4,
                    piece.t1 + index * 4,
                    piece.position0,
                    piece.velocity0,
                    piece.acceleration,
                )
            )
    original = module._swept_bounds
    calls = []

    def counted(piece):
        calls.append(piece)
        return original(piece)

    monkeypatch.setattr(module, "_swept_bounds", counted)
    second = reservations.reserve(2, candidate, 0, cold=True, end_hold=False)
    assert second.accepted and second.start_time > 0
    assert not audit_trajectories(first.pieces + second.pieces, 8)
    assert len(calls) < 200, "wait probes rebuilt unrelated phase bounds"


def test_infinite_hold_is_checked_before_unnecessary_finite_wait_search(monkeypatch):
    reservations = book()
    assert reservations.reserve(
        1, stopped(1, (-20, 0, 100), (20, 0, 100)), 0, cold=True, end_hold=False
    ).accepted
    assert reservations.reserve(3, (), 0, start_position=(0, 20, 100)).accepted
    original = reservations._conflict
    checked = []

    def counted(candidate, other):
        checked.append(other.kind)
        return original(candidate, other)

    monkeypatch.setattr(reservations, "_conflict", counted)
    blocked = reservations.reserve(
        2, stopped(2, (0, -20, 100), (0, 20, 100)), 0, cold=True
    )
    assert not blocked.accepted and blocked.blocked_by == (3,)
    assert checked and set(checked) == {"HOLD"}


def test_clearance_time_covers_registered_finite_flights_without_mutation():
    reservations = book()
    assert reservations.clearance_time() == 0.0
    assert reservations.clearance_time(7.0) == 7.0
    assert reservations.reserve(
        1, stopped(1, (0, 0, 100), (20, 0, 100)), 10.0, cold=True, end_hold=False
    ).accepted
    before = reservations.metrics()
    assert reservations.clearance_time(0.0) == 14.0
    assert reservations.clearance_time(20.0) == 20.0
    assert reservations.metrics() == before
    reservations.expire(20.0)
    assert reservations.clearance_time(20.0) == 20.0


def test_clearance_time_cannot_claim_empty_airspace_while_endpoint_hold_remains():
    reservations = book()
    assert reservations.reserve(1, (), 0.0, start_position=(0, 0, 100)).accepted
    assert reservations.clearance_time(100.0) == math.inf
    reservations.release(1)
    assert reservations.clearance_time(100.0) == 100.0


@pytest.mark.parametrize("now", [math.nan, math.inf, -math.inf])
def test_clearance_time_requires_finite_world_time(now):
    with pytest.raises(ValueError, match="now"):
        book().clearance_time(now)


def test_minimum_search_shift_is_physical_seconds_and_keeps_continuous_safety():
    outcomes = []
    for minimum in (0.0, 0.1):
        reservations = book(min_search_shift_seconds=minimum)
        point = TrajectoryPiece(1, 0.0, 0.0, (0, 0, 100), (0, 0, 0), (0, 0, 0))
        first = reservations.reserve(1, (point,), 0.0, cold=True, end_hold=False)
        flight = TrajectoryPiece(2, 0.0, 1.0, (0, 0, 100), (20, 0, 0), (0, 0, 0))
        second = reservations.reserve(2, (flight,), 0.0, cold=True, end_hold=False)
        assert second.accepted
        assert not audit_trajectories(first.pieces + second.pieces, 8.0)
        outcomes.append(second.start_time)
    assert 0.0 < outcomes[0] < 1e-5
    assert outcomes[1] == 0.1


@pytest.mark.parametrize("minimum", [-0.1, math.inf, math.nan])
def test_minimum_search_shift_requires_finite_nonnegative_seconds(minimum):
    with pytest.raises(ValueError, match="min_search_shift_seconds"):
        book(min_search_shift_seconds=minimum)
def test_exhausted_search_exposes_a_later_retry_without_admitting_unsafe_motion():
    from uav_sim.algorithms.continuous_safety import TrajectoryPiece
    from uav_sim.algorithms.prefix_reservations import PrefixReservations
    manager = PrefixReservations(min_distance_m=1., max_attempts=1)
    a = (TrajectoryPiece(1,0.,10.,(-10.,0.,0.),(2.,0.,0.),(0.,0.,0.)),)
    b = (TrajectoryPiece(2,0.,10.,(0.,-10.,0.),(0.,2.,0.),(0.,0.,0.)),)
    assert manager.reserve(1,a,0.,end_hold=False,cold=True).accepted
    refused = manager.reserve(2,b,0.,end_hold=False,cold=True)
    assert not refused.accepted and refused.retry_start_time > 0.
    approved = manager.reserve(2,b,refused.retry_start_time,end_hold=False,cold=True)
    assert approved.accepted


def test_compiled_finite_wait_search_avoids_python_phase_reconstruction(monkeypatch):
    from uav_sim.algorithms.continuous_safety import TrajectoryPiece, audit_trajectories
    from uav_sim.algorithms.prefix_reservations import PrefixReservations
    from uav_sim.algorithms.fast_continuous_safety import COMPILED_AVAILABLE
    if not COMPILED_AVAILABLE:
        pytest.skip('Optional Numba is unavailable')
    manager = PrefixReservations(min_distance_m=1.)
    a=(TrajectoryPiece(1,0.,10.,(-10.,0.,0.),(2.,0.,0.),(0.,0.,0.)),)
    b=(TrajectoryPiece(2,0.,10.,(0.,-10.,0.),(0.,2.,0.),(0.,0.,0.)),)
    first=manager.reserve(1,a,0.,end_hold=False,cold=True)
    calls=[]
    original=manager._conflict
    def counted(*args):
        calls.append(True)
        return original(*args)
    monkeypatch.setattr(manager,'_conflict',counted)
    second=manager.reserve(2,b,0.,end_hold=False,cold=True)
    assert second.accepted and second.start_time>0
    assert not calls
    assert not audit_trajectories(first.pieces+second.pieces,1.)

