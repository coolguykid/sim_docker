"""Exact reusable space-time certificates for regional autopilot prefixes.

Spatial cells index individual quadratic phases and stationary endpoint
holds.  They are a broad phase only: every accepted neighboring pair passes
the analytic continuous 3D closest-approach test.  An endpoint hold lasts
until its owner's next prefix is accepted or the owner explicitly exits.
"""
from collections import defaultdict
from collections.abc import Mapping
from dataclasses import dataclass
import heapq
import itertools
import math

from uav_sim.algorithms.continuous_safety import (
    TrajectoryPiece,
    min_distance,
    _swept_bounds,
)
from .fast_continuous_safety import COMPILED_AVAILABLE, prepare_finite_scan


@dataclass(frozen=True)
class ReservationResult:
    accepted: bool
    start_time: float = math.inf
    end_time: float = math.inf
    pieces: tuple = ()
    motion_pieces: tuple = ()
    thresholds: tuple = ()
    end_hold: bool = True
    end_hold_threshold: float = 8.0
    reason: str = ""
    blocked_by: tuple = ()
    shift_seconds: float = 0.0
    retry_start_time: object = None

    @property
    def deferred(self):
        return not self.accepted

    def __bool__(self):
        return self.accepted


@dataclass(frozen=True)
class ReservedPhase:
    record_id: int
    uav_id: object
    t0: float
    t1: float
    position: tuple
    piece: object
    threshold: float
    lower: tuple
    upper: tuple
    kind: str = "MOTION"

    @property
    def is_hold(self):
        return self.kind == "HOLD"


class PrefixReservations:
    """Atomically reserve exact prefixes while retaining safe source holds.

    ``pieces`` passed to :meth:`reserve` use relative times starting at zero.
    ``thresholds`` optionally gives one original physical separation per
    phase; ``threshold`` overrides the default for the whole prefix.  Waiting
    airborne aircraft occupy their start point; cold requests enter only at
    the accepted start time.  Deferred requests retain their old occupancy.
    """

    def __init__(
        self,
        min_distance_m=8.0,
        safety_margin_m=1e-4,
        cell_size=64.0,
        max_attempts=256,
        max_wait_seconds=100000.0,
        max_cells_per_phase=4096,
        use_compiled=True,
        min_search_shift_seconds=0.0,
    ):
        self.min_distance_m = min_distance_m
        self.safety_margin_m = self._number(safety_margin_m, "safety_margin_m", 0.0)
        self.cell_size = self._number(cell_size, "cell_size", 1e-12)
        self.max_wait_seconds = self._number(max_wait_seconds, "max_wait_seconds", 0.0)
        self.max_attempts = int(max_attempts)
        self.max_cells_per_phase = int(max_cells_per_phase)
        self.use_compiled = bool(use_compiled and COMPILED_AVAILABLE)
        self.min_search_shift_seconds = self._number(
            min_search_shift_seconds, "min_search_shift_seconds", 0.0
        )
        if self.max_attempts < 1 or self.max_cells_per_phase < 1:
            raise ValueError("max_attempts and max_cells_per_phase must be positive")
        self._records = {}
        self._by_uid = defaultdict(set)
        self._cells = defaultdict(set)
        self._record_cells = {}
        self._large = set()
        self._ends = []
        self._next_id = itertools.count()
        self._maximum_threshold = 0.0
        self._expired_before = -math.inf
        self._pending = {}
        self._counts = {
            "reservation_attempts": 0,
            "accepted_count": 0,
            "deferred_count": 0,
            "shifted_count": 0,
            "phase_pairs_checked": 0,
            "spatial_candidate_count": 0,
            "max_shift_seconds": 0.0,
        }

    @staticmethod
    def _number(value, name, minimum=None):
        value = float(value)
        if not math.isfinite(value) or (minimum is not None and value < minimum):
            raise ValueError("%s must be finite and at least %s" % (name, minimum))
        return value

    @staticmethod
    def _position(value, name):
        value = tuple(float(x) for x in value)
        if len(value) != 3 or not all(math.isfinite(x) for x in value):
            raise ValueError("%s requires three finite coordinates" % name)
        return value

    def _threshold(self, uid, override=None):
        value = self.min_distance_m if override is None else override
        if isinstance(value, Mapping):
            if uid not in value:
                raise ValueError("missing minimum distance for UAV %r" % (uid,))
            value = value[uid]
        return self._number(value, "threshold", 0.0) + self.safety_margin_m

    def _cell_keys(self, lower, upper):
        bounds = [
            (math.floor(a / self.cell_size), math.floor(b / self.cell_size))
            for a, b in zip(lower, upper)
        ]
        if math.prod(b - a + 1 for a, b in bounds) > self.max_cells_per_phase:
            return None
        return itertools.product(*(range(a, b + 1) for a, b in bounds))

    def _piece_cell_keys(self, piece, extent=0.0):
        """Conservatively index a long curve without filling its empty box.

        Large diagonal swept boxes contain thousands of cells the path never
        visits. Subdivide only those large boxes into exact quadratic pieces;
        the union of their swept bounds still covers every trajectory point.
        This changes the broad phase only. Analytic closest approach remains
        the acceptance test, and extremely long paths retain the safe global
        fallback when their actual cell cover exceeds the configured budget.
        """
        lower, upper = _swept_bounds(piece)

        def expanded_keys(a, b):
            return self._cell_keys(
                tuple(x - extent for x in a), tuple(x + extent for x in b)
            )

        direct = expanded_keys(lower, upper)
        if direct is not None:
            return direct
        pending, found = [(piece.t0, piece.t1)], set()
        while pending:
            first, last = pending.pop()
            segment = TrajectoryPiece(
                piece.uav_id,
                first,
                last,
                piece.position_at(first),
                piece.velocity_at(first),
                piece.acceleration,
            )
            a, b = _swept_bounds(segment)
            middle = first + (last - first) * 0.5
            if (
                max(y - x for x, y in zip(a, b)) > 8 * self.cell_size
                and first < middle < last
            ):
                pending.extend(((first, middle), (middle, last)))
                continue
            keys = expanded_keys(a, b)
            if keys is None:
                return None
            found.update(keys)
            if len(found) > self.max_cells_per_phase:
                return None
        return found

    def _query(self, lower, upper, threshold, piece=None):
        extent = max(threshold, self._maximum_threshold)
        keys = (
            self._piece_cell_keys(piece, extent)
            if piece is not None
            else self._cell_keys(
                tuple(x - extent for x in lower), tuple(x + extent for x in upper)
            )
        )
        if keys is None:
            return set(self._records)
        found = set(self._large)
        for key in keys:
            found.update(self._cells.get(key, ()))
        return found

    def _insert(self, record):
        self._records[record.record_id] = record
        self._by_uid[record.uav_id].add(record.record_id)
        keys = (
            self._piece_cell_keys(record.piece)
            if record.piece is not None
            else self._cell_keys(record.lower, record.upper)
        )
        if keys is None:
            self._large.add(record.record_id)
        else:
            cells = tuple(keys)
            self._record_cells[record.record_id] = cells
            for key in cells:
                self._cells[key].add(record.record_id)
        self._maximum_threshold = max(self._maximum_threshold, record.threshold)
        if math.isfinite(record.t1):
            heapq.heappush(self._ends, (record.t1, record.record_id))

    def _remove(self, record_id):
        record = self._records.pop(record_id, None)
        if record is None:
            return
        self._by_uid[record.uav_id].discard(record_id)
        if not self._by_uid[record.uav_id]:
            del self._by_uid[record.uav_id]
        self._large.discard(record_id)
        for key in self._record_cells.pop(record_id, ()):
            bucket = self._cells[key]
            bucket.discard(record_id)
            if not bucket:
                del self._cells[key]

    def release(self, uid, now=None):
        """Remove an exited aircraft, including its old infinite endpoint hold."""
        if now is not None:
            self.expire(now)
        for record_id in tuple(self._by_uid.get(uid, ())):
            self._remove(record_id)
        self._pending.pop(uid, None)

    def expire(self, now):
        """Retire strict-past finite phases while preserving contact endpoints."""
        now = self._number(now, "now")
        self._expired_before = max(self._expired_before, now)
        while self._ends and self._ends[0][0] < now:
            _, record_id = heapq.heappop(self._ends)
            self._remove(record_id)

    def update(self, uid, now):
        """Retire elapsed phases; the UID's source/end hold remains reserved."""
        self.expire(now)
        return tuple(self._records[index] for index in self._by_uid.get(uid, ()))

    def clearance_time(self, now=0.0):
        """Earliest conservative time after every registered finite flight.

        This read-only bound supports a cold-ground serialization fallback.
        An infinite endpoint hold has no known clearance, so it yields inf
        rather than silently assuming the waiting aircraft will disappear.
        """
        now = self._number(now, "now")
        return max(
            now, max((record.t1 for record in self._records.values()), default=now)
        )

    def priority_key(self, uid, now):
        """Older deferred requests sort first; callers own ready-queue order."""
        first, attempts = self._pending.get(uid, (float(now), 0))
        return (first, -attempts, repr(uid))

    def metrics(self):
        values = dict(self._counts)
        values.update(
            native_scan_enabled=self.use_compiled,
            max_attempts=self.max_attempts,
            min_search_shift_seconds=self.min_search_shift_seconds,
            active_finite_phases=sum(
                not value.is_hold for value in self._records.values()
            ),
            active_holds=sum(value.is_hold for value in self._records.values()),
            active_uavs=len(self._by_uid),
            spatial_cell_count=len(self._cells),
            large_phase_count=len(self._large),
            pending_count=len(self._pending),
        )
        return values

    def nearby(self, pieces, ignore_uid=None):
        """Return stored phases/holds whose spatial cells neighbor the query."""
        found = set()
        for piece in pieces:
            lower, upper = _swept_bounds(piece)
            found.update(self._query(lower, upper, self._maximum_threshold, piece))
        return tuple(
            self._records[index]
            for index in sorted(found)
            if self._records[index].uav_id != ignore_uid
        )

    @staticmethod
    def _record(uid, piece, threshold, kind="MOTION", position=None, start=None):
        if piece is None:
            return ReservedPhase(
                -1,
                uid,
                start,
                math.inf,
                position,
                None,
                threshold,
                position,
                position,
                kind,
            )
        lower, upper = _swept_bounds(piece)
        return ReservedPhase(
            -1,
            uid,
            piece.t0,
            piece.t1,
            piece.position0,
            piece,
            threshold,
            lower,
            upper,
            kind,
        )

    def _conflict(self, candidate, other):
        start, end = max(candidate.t0, other.t0), min(candidate.t1, other.t1)
        if start > end:
            return None
        threshold = max(candidate.threshold, other.threshold)
        if any(
            candidate.lower[k] - other.upper[k] > threshold
            or other.lower[k] - candidate.upper[k] > threshold
            for k in range(3)
        ):
            return None
        self._counts["phase_pairs_checked"] += 1
        if candidate.is_hold and other.is_hold:
            distance, time = math.dist(candidate.position, other.position), start
        else:
            a = candidate.piece or TrajectoryPiece(
                candidate.uav_id, start, end, candidate.position, (0, 0, 0), (0, 0, 0)
            )
            b = other.piece or TrajectoryPiece(
                other.uav_id, start, end, other.position, (0, 0, 0), (0, 0, 0)
            )
            distance, time = min_distance(a, b)
        return (
            (time, distance)
            if distance <= threshold + 16 * math.ulp(max(1.0, threshold))
            else None
        )

    def reserve(
        self,
        uid,
        pieces,
        earliest_start,
        start_position=None,
        end_position=None,
        end_hold=True,
        cold=False,
        threshold=None,
        thresholds=None,
    ):
        earliest_start = self._number(earliest_start, "earliest_start")
        if earliest_start < self._expired_before - 1e-8:
            raise ValueError("cannot reserve a trajectory before retired world time")
        self._counts["reservation_attempts"] += 1
        first, attempts = self._pending.get(uid, (earliest_start, 0))
        self._pending[uid] = (first, attempts + 1)
        pieces = tuple(pieces)
        if any(not isinstance(piece, TrajectoryPiece) for piece in pieces):
            raise ValueError("prefix pieces must be TrajectoryPiece instances")
        if pieces:
            if abs(pieces[0].t0) > 1e-9:
                raise ValueError("prefix trajectory must start at relative time zero")
            for before, after in zip(pieces, pieces[1:]):
                if (
                    abs(before.t1 - after.t0) > 1e-8
                    or math.dist(before.position_at(before.t1), after.position0) > 1e-5
                ):
                    raise ValueError(
                        "prefix phases must join without time gaps or position jumps"
                    )
                if math.dist(before.velocity_at(before.t1), after.velocity0) > 1e-5:
                    raise ValueError("prefix phases must join without velocity jumps")
            start = pieces[0].position0
            end = pieces[-1].position_at(pieces[-1].t1)
            duration = pieces[-1].t1
            if end_hold and math.hypot(*pieces[-1].velocity_at(duration)) > 1e-6:
                raise ValueError("an endpoint hold requires a stationary physical stop")
        else:
            if start_position is None:
                raise ValueError("an empty prefix requires start_position")
            start = end = self._position(start_position, "start_position")
            duration = 0.0
        for supplied, actual, name in (
            (start_position, start, "start_position"),
            (end_position, end, "end_position"),
        ):
            if (
                supplied is not None
                and math.dist(self._position(supplied, name), actual) > 1e-5
            ):
                raise ValueError("%s differs from actual trajectory endpoint" % name)
        default = self._threshold(uid, threshold)
        if thresholds is None:
            per_phase = (default,) * len(pieces)
        else:
            raw = (
                tuple(thresholds[index] for index in range(len(pieces)))
                if isinstance(thresholds, Mapping)
                else tuple(thresholds)
            )
            if len(raw) != len(pieces):
                raise ValueError(
                    "thresholds requires one distance per trajectory phase"
                )
            per_phase = tuple(self._threshold(uid, value) for value in raw)
        start_threshold = per_phase[0] if per_phase else default
        end_threshold = per_phase[-1] if per_phase else default

        # Geometry and swept bounds do not change when a departure is shifted.
        # Compute them once; wait-search probes retime just the conflicting
        # phase rather than rebuilding every unrelated phase on the prefix.
        base_motion = tuple(self._record(uid, p, t) for p, t in zip(pieces, per_phase))
        motion_offset = int(not cold)
        record_count = motion_offset + len(base_motion) + int(end_hold)

        def candidate_at(index, start_time):
            if not cold and index == 0:
                wait = TrajectoryPiece(
                    uid, earliest_start, start_time, start, (0, 0, 0), (0, 0, 0)
                )
                return ReservedPhase(
                    -1,
                    uid,
                    earliest_start,
                    start_time,
                    start,
                    wait,
                    start_threshold,
                    start,
                    start,
                    "WAIT",
                )
            phase_index = index - motion_offset
            if phase_index < len(base_motion):
                base = base_motion[phase_index]
                p = base.piece
                shifted = TrajectoryPiece(
                    uid,
                    p.t0 + start_time,
                    p.t1 + start_time,
                    p.position0,
                    p.velocity0,
                    p.acceleration,
                )
                return ReservedPhase(
                    -1,
                    uid,
                    shifted.t0,
                    shifted.t1,
                    base.position,
                    shifted,
                    base.threshold,
                    base.lower,
                    base.upper,
                    base.kind,
                )
            return ReservedPhase(
                -1,
                uid,
                start_time + duration,
                math.inf,
                end,
                None,
                end_threshold,
                end,
                end,
                "HOLD",
            )

        def candidates(start_time):
            records = tuple(
                candidate_at(index, start_time) for index in range(record_count)
            )
            motion = tuple(
                record.piece for record in records if record.kind == "MOTION"
            )
            return records, motion

        initial, _ = candidates(earliest_start)
        neighbors = []
        for candidate in initial:
            ids = self._query(
                candidate.lower, candidate.upper, candidate.threshold, candidate.piece
            )
            others = tuple(
                self._records[index]
                for index in ids
                if self._records[index].uav_id != uid
            )
            self._counts["spatial_candidate_count"] += len(others)
            neighbors.append(others)
        hold_neighbors = tuple(
            tuple(other for other in group if other.is_hold) for group in neighbors
        )
        finite_neighbors = tuple(
            tuple(other for other in group if not other.is_hold) for group in neighbors
        )
        # Cold finite journeys have no physical source wait or endpoint hold.
        # Their stable geometry can be scanned as one native batch for every
        # trial departure, instead of millions of Python pair calls. Holds and
        # airborne waits retain the explicit transactional reference path.
        finite_scan = (
            prepare_finite_scan(base_motion, finite_neighbors)
            if self.use_compiled and cold and not end_hold and not any(hold_neighbors)
            else None
        )

        def failure(reason, owners):
            self._counts["deferred_count"] += 1
            return ReservationResult(
                False,
                reason=reason,
                blocked_by=tuple(sorted(set(owners), key=repr)),
                end_hold=end_hold,
                retry_start_time=start_time if reason == 'bounded reservation search exhausted' else None,
            )

        start_time = earliest_start
        for _ in range(self.max_attempts):
            if finite_scan is not None:
                index, other, when, distance, checked = finite_scan.scan(start_time)
                self._counts["phase_pairs_checked"] += checked
                conflicts = [] if index < 0 else [(when, index, other)]
                if not conflicts:
                    current, motion = candidates(start_time)
            else:
                current, motion = candidates(start_time)
                # An infinite hold cannot be cleared by delaying departure.
                # Reject it before solving finite-flight wait windows. Check
                # stationary endpoints before moving phases.
                order = (
                    (len(current) - 1,) + tuple(range(len(current) - 1))
                    if end_hold
                    else range(len(current))
                )
                for index in order:
                    for other in hold_neighbors[index]:
                        if self._conflict(current[index], other) is not None:
                            return failure(
                                "infinite endpoint hold blocks this prefix",
                                (other.uav_id,),
                            )
                conflicts = []
                for index, candidate in enumerate(current):
                    for other in finite_neighbors[index]:
                        conflict = self._conflict(candidate, other)
                        if conflict is not None:
                            if candidate.kind == "WAIT":
                                return failure(
                                    "airborne waiting position conflicts with a reserved flight",
                                    (other.uav_id,),
                                )
                            conflicts.append((conflict[0], index, other))
            if not conflicts:
                # Commit this new certificate atomically, replacing the UID's
                # old future phases and hold only after every check succeeded.
                for old_id in tuple(self._by_uid.get(uid, ())):
                    self._remove(old_id)
                for record in current:
                    self._insert(
                        ReservedPhase(
                            next(self._next_id),
                            record.uav_id,
                            record.t0,
                            record.t1,
                            record.position,
                            record.piece,
                            record.threshold,
                            record.lower,
                            record.upper,
                            record.kind,
                        )
                    )
                self._pending.pop(uid, None)
                self._counts["accepted_count"] += 1
                shift = start_time - earliest_start
                self._counts["shifted_count"] += shift > 1e-8
                self._counts["max_shift_seconds"] = max(
                    self._counts["max_shift_seconds"], shift
                )
                finite = tuple(record.piece for record in current if not record.is_hold)
                finite_thresholds = tuple(
                    record.threshold - self.safety_margin_m
                    for record in current
                    if not record.is_hold
                )
                return ReservationResult(
                    True,
                    start_time,
                    start_time + duration,
                    finite,
                    motion,
                    finite_thresholds,
                    end_hold,
                    end_threshold - self.safety_margin_m,
                    shift_seconds=shift,
                )
            _, index, other = min(conflicts, key=lambda value: value[0])
            candidate = (
                candidate_at(index, start_time)
                if finite_scan is not None
                else current[index]
            )
            relative_start = candidate.t0 - start_time
            upper = max(start_time + 1e-7, other.t1 - relative_start + 1e-7)
            if upper - earliest_start > self.max_wait_seconds:
                # A long phase can have a short crossing window.  Search its
                # continuously safe upper endpoint before applying the cap.
                allowed = earliest_start + self.max_wait_seconds
                if (
                    allowed <= start_time
                    or self._conflict(candidate_at(index, allowed), other) is not None
                ):
                    return failure(
                        "required waiting time exceeds configured bound",
                        (other.uav_id,),
                    )
                upper = allowed
            lower = start_time
            # The upper endpoint clears this finite phase in time.  Every
            # accepted whole-prefix result is subsequently re-audited, even
            # if several disjoint geometric conflict windows exist.
            if finite_scan is not None:
                upper,checked = finite_scan.clearance_after(index,other,lower,upper)
                self._counts['phase_pairs_checked'] += checked
            else:
                for _ in range(28):
                    middle = (lower + upper) * 0.5
                    if self._conflict(candidate_at(index, middle), other) is None:
                        upper = middle
                    else:
                        lower = middle
            # This optional physical-time increment prevents repeated tiny
            # departures along nearly coincident conflict-window boundaries.
            # The entire trajectory is rechecked after the increment; output
            # clock dt has no role in this search or its safety threshold.
            start_time = max(upper + 1e-8, start_time + self.min_search_shift_seconds)
            if start_time - earliest_start > self.max_wait_seconds + 1e-8:
                return failure(
                    "required waiting time exceeds configured bound", (other.uav_id,)
                )
        return failure(
            "bounded reservation search exhausted",
            (other.uav_id for group in neighbors for other in group),
        )
