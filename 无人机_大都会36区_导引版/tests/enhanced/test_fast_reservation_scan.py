import math
import random

import pytest

from uav_sim.algorithms.continuous_safety import TrajectoryPiece
from uav_sim.algorithms.prefix_reservations import PrefixReservations


def test_compiled_batch_scan_matches_every_reference_pair_and_earliest_conflict():
    from uav_sim.algorithms.fast_continuous_safety import prepare_finite_scan

    rng = random.Random(7151)
    registry = PrefixReservations()
    candidates = []
    neighbors = []
    for index in range(15):
        p = TrajectoryPiece(
            1,
            0.0,
            3.0,
            tuple(rng.uniform(-20, 20) for _ in range(3)),
            tuple(rng.uniform(-10, 10) for _ in range(3)),
            tuple(rng.uniform(-2, 2) for _ in range(3)),
        )
        candidates.append(registry._record(1, p, 8.0001))
        group = []
        for j in range(30):
            other = TrajectoryPiece(
                j + 2,
                float(j % 4),
                float(j % 4) + 4.0,
                tuple(rng.uniform(-30, 30) for _ in range(3)),
                tuple(rng.uniform(-10, 10) for _ in range(3)),
                tuple(rng.uniform(-2, 2) for _ in range(3)),
            )
            group.append(registry._record(j + 2, other, 10.0001))
        neighbors.append(tuple(group))
    batch = prepare_finite_scan(candidates, neighbors)
    for shift in (0.0, 0.01, 0.5, 2.0, 6.0, 20.0):
        expected = []
        checked_before = registry.metrics()["phase_pairs_checked"]
        for i, base in enumerate(candidates):
            p = base.piece
            shifted = registry._record(
                1,
                TrajectoryPiece(
                    1,
                    p.t0 + shift,
                    p.t1 + shift,
                    p.position0,
                    p.velocity0,
                    p.acceleration,
                ),
                base.threshold,
            )
            for other in neighbors[i]:
                conflict = registry._conflict(shifted, other)
                if conflict is not None:
                    expected.append((conflict[0], i, other, conflict[1]))
        checked = registry.metrics()["phase_pairs_checked"] - checked_before
        actual = batch.scan(shift)
        assert actual[4] <= checked
        if expected:
            first = min(expected, key=lambda x: x[0])
            assert actual[0] == first[1] and actual[1] is first[2]
            assert actual[2] == pytest.approx(first[0], abs=1e-12)
            assert actual[3] == pytest.approx(first[3], abs=1e-12)
        else:
            assert actual[0] == -1 and actual[1] is None


def test_batch_scan_keeps_contact_at_interval_endpoint_and_exact_safe_threshold():
    from uav_sim.algorithms.fast_continuous_safety import prepare_finite_scan

    registry = PrefixReservations(safety_margin_m=0)
    a = registry._record(
        1, TrajectoryPiece(1, 0, 1, (0, 0, 0), (0, 0, 0), (0, 0, 0)), 8.0
    )
    b = registry._record(
        2, TrajectoryPiece(2, 1, 2, (8, 0, 0), (0, 0, 0), (0, 0, 0)), 8.0
    )
    batch = prepare_finite_scan((a,), ((b,),))
    result = batch.scan(0.0)
    assert result[:4] == (0, b, 1.0, 8.0)
    assert batch.scan(3.0)[0] == -1


def test_batch_scan_rejects_holds_instead_of_treating_them_as_finite():
    from uav_sim.algorithms.fast_continuous_safety import prepare_finite_scan

    registry = PrefixReservations()
    held = registry._record(1, None, 8.0, kind="HOLD", position=(0, 0, 0), start=0.0)
    with pytest.raises(ValueError, match="finite"):
        prepare_finite_scan((held,), ((),))


def test_native_reservation_admission_matches_python_pair_loop_and_remains_safe():
    from tests.enhanced.test_prefix_reservations import stopped
    from uav_sim.algorithms.continuous_safety import audit_trajectories

    outcomes = []
    for compiled in (False, True):
        registry = PrefixReservations(use_compiled=compiled)
        values = []
        for uid in range(25):
            if uid % 2:
                pieces = stopped(uid, (-20, uid % 3, 100), (20, uid % 3, 100))
            else:
                pieces = stopped(uid, (uid % 3, -20, 100), (uid % 3, 20, 100))
            result = registry.reserve(uid, pieces, 0.0, cold=True, end_hold=False)
            assert result.accepted
            values.append(result)
        assert not audit_trajectories(
            [p for value in values for p in value.pieces], 8.0
        )
        outcomes.append([value.start_time for value in values])
    assert outcomes[0] == pytest.approx(outcomes[1], abs=1e-7)


def test_metrics_identify_actual_native_path_and_physical_wait_search_policy():
    from uav_sim.algorithms.fast_continuous_safety import COMPILED_AVAILABLE

    registry = PrefixReservations(
        use_compiled=True, max_attempts=4096, min_search_shift_seconds=0.1
    )
    values = registry.metrics()
    assert values["native_scan_enabled"] is COMPILED_AVAILABLE
    assert values["max_attempts"] == 4096
    assert values["min_search_shift_seconds"] == 0.1


def test_static_separation_is_pruned_once_before_departure_search():
    from uav_sim.algorithms.fast_continuous_safety import prepare_finite_scan

    registry = PrefixReservations()

    def occupant(uid, position, start=0.0, end=1.0):
        return registry._record(
            uid, TrajectoryPiece(uid, start, end, position, (0, 0, 0), (0, 0, 0)), 8.0
        )

    candidate = occupant(1, (0, 0, 0))
    nearby = occupant(2, (0, 0, 0))
    far = [occupant(uid + 3, (100.0 + uid, 0, 0)) for uid in range(100)]
    batch = prepare_finite_scan((candidate,), ((nearby, *far),))
    assert len(batch.pairs) == 1
    assert batch.scan(0.0)[:4] == (0, nearby, 0.0, 0.0)
    assert batch.scan(2.0)[0] == -1


def test_earliest_conflict_stops_later_monotone_phases_without_root_solves():
    from uav_sim.algorithms.fast_continuous_safety import prepare_finite_scan

    registry = PrefixReservations()

    def occupant(uid, start, end):
        return registry._record(
            uid, TrajectoryPiece(uid, start, end, (0, 0, 0), (0, 0, 0), (0, 0, 0)), 8.0
        )

    candidates = (occupant(1, 0.0, 1.0), occupant(1, 2.0, 3.0))
    first = occupant(2, 0.2, 0.3)
    later = tuple(occupant(uid + 3, 2.0, 3.0) for uid in range(100))
    batch = prepare_finite_scan(candidates, ((first,), later))
    result = batch.scan(0.0)
    assert result[:4] == (0, first, 0.2, 0.0)
    assert result[4] == 1


def test_earliest_conflict_skips_neighbors_that_only_exist_after_best_time():
    from uav_sim.algorithms.fast_continuous_safety import prepare_finite_scan

    registry = PrefixReservations()

    def occupant(uid, start, end):
        return registry._record(
            uid, TrajectoryPiece(uid, start, end, (0, 0, 0), (0, 0, 0), (0, 0, 0)), 8.0
        )

    candidate = occupant(1, 0.0, 10.0)
    first = occupant(2, 0.2, 0.3)
    later = tuple(occupant(uid + 3, 3.0, 4.0) for uid in range(100))
    batch = prepare_finite_scan((candidate,), ((first, *later),))
    result = batch.scan(0.0)
    assert result[:4] == (0, first, 0.2, 0.0)
    assert result[4] == 1


def test_unsorted_candidate_input_retains_correct_earliest_conflict():
    from uav_sim.algorithms.fast_continuous_safety import prepare_finite_scan

    registry = PrefixReservations()

    def occupant(uid, start):
        return registry._record(
            uid,
            TrajectoryPiece(uid, start, start + 1, (0, 0, 0), (0, 0, 0), (0, 0, 0)),
            8.0,
        )

    candidates = (occupant(1, 2.0), occupant(1, 10.0), occupant(1, 1.0))
    neighbors = tuple(
        (occupant(i + 2, start),) for i, start in enumerate((2.0, 10.0, 1.0))
    )
    result = prepare_finite_scan(candidates, neighbors).scan(0.0)
    assert result[:4] == (2, neighbors[2][0], 1.0, 0.0)
def test_large_shared_neighbor_batch_avoids_a_python_tuple_per_pair():
    import tracemalloc
    from uav_sim.algorithms.fast_continuous_safety import prepare_finite_scan
    registry=PrefixReservations()
    piece=TrajectoryPiece(1,0.,1.,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.))
    candidate=registry._record(1,piece,8.)
    other=registry._record(2,TrajectoryPiece(2,0.,1.,(100.,0.,0.),(0.,0.,0.),(0.,0.,0.)),8.)
    prepare_finite_scan((candidate,),((other,),)).scan(0.)
    neighbors=((other,)*100000,)
    tracemalloc.start()
    try:
        batch=prepare_finite_scan((candidate,),neighbors)
        _,peak=tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    assert batch.scan(0.)[0] == -1
    assert peak < 7_000_000, peak

