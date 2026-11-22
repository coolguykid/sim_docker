"""Continuous closest approach of finite, piecewise quadratic 3D paths.

The narrow phase minimizes squared separation analytically: endpoints plus
the real roots of its cubic derivative. A swept AABB sweep precedes it. The
sweep chooses the axis with greatest spatial spread, so sparse fleets avoid
quadratic root solves even when all vehicles share two coordinates. Dense or
very long overlapping swept boxes can still require quadratic candidate work.

Intervals are closed: touching in time or at the required separation is a
conflict. Disjoint intervals have closest approach ``(inf, None)``.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import math
from typing import Hashable, Tuple
from .fast_continuous_safety import COMPILED_AVAILABLE, fast_min_distance

Vector3 = Tuple[float, float, float]


def _finite(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("%s must be a finite number" % name) from error
    if not math.isfinite(value):
        raise ValueError("%s must be a finite number" % name)
    return value


def _vector(value, name):
    try:
        values = tuple(value)
    except TypeError as error:
        raise ValueError("%s must contain exactly three finite numbers" % name) from error
    if len(values) != 3:
        raise ValueError("%s must contain exactly three finite numbers" % name)
    x0 = _finite(values[0], name)
    x1 = _finite(values[1], name)
    x2 = _finite(values[2], name)
    return (x0, x1, x2)


@dataclass(frozen=True)
class TrajectoryPiece:
    uav_id: Hashable
    t0: float
    t1: float
    position0: Vector3
    velocity0: Vector3
    acceleration: Vector3

    def __post_init__(self):
        try:
            hash(self.uav_id)
        except TypeError as error:
            raise ValueError("uav_id must be hashable") from error
        t0 = _finite(self.t0, "t0")
        t1 = _finite(self.t1, "t1")
        object.__setattr__(self, "t0", t0)
        object.__setattr__(self, "t1", t1)
        if t1 < t0:
            raise ValueError("trajectory t1 must not precede t0")
        object.__setattr__(self, "position0", _vector(self.position0, "position0"))
        object.__setattr__(self, "velocity0", _vector(self.velocity0, "velocity0"))
        object.__setattr__(self, "acceleration", _vector(self.acceleration, "acceleration"))

    def _elapsed(self, time_s):
        time_s = _finite(time_s, "time_s")
        if not self.t0 <= time_s <= self.t1:
            raise ValueError("time_s lies outside trajectory interval")
        return time_s - self.t0

    def position_at(self, time_s):
        elapsed = self._elapsed(time_s)
        half_t2 = 0.5 * elapsed * elapsed
        p0, v0, a0 = self.position0, self.velocity0, self.acceleration
        x = p0[0] + v0[0] * elapsed + a0[0] * half_t2
        y = p0[1] + v0[1] * elapsed + a0[1] * half_t2
        z = p0[2] + v0[2] * elapsed + a0[2] * half_t2
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            raise ValueError("trajectory position exceeds finite floating-point range")
        return (x, y, z)

    def velocity_at(self, time_s):
        elapsed = self._elapsed(time_s)
        v0, a0 = self.velocity0, self.acceleration
        vx = v0[0] + a0[0] * elapsed
        vy = v0[1] + a0[1] * elapsed
        vz = v0[2] + a0[2] * elapsed
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(vz)):
            raise ValueError("trajectory velocity exceeds finite floating-point range")
        return (vx, vy, vz)


@dataclass(frozen=True)
class Conflict:
    uav_id_a: Hashable
    uav_id_b: Hashable
    time_s: float
    distance_m: float
    required_distance_m: float


def _quadratic_roots(a, b, c):
    """Stable real quadratic roots; exact degree reduction retains tiny terms."""
    if a == 0:
        return () if b == 0 else (-c / b,)
    scale = max(abs(a), abs(b), abs(c))
    a, b, c = a / scale, b / scale, c / scale
    discriminant = b * b - 4 * a * c
    rounding = 16 * math.ulp(1.0) * (b * b + abs(4 * a * c))
    if discriminant < -rounding:
        return ()
    root = math.sqrt(max(0.0, discriminant))
    q = -0.5 * (b + math.copysign(root, b))
    if q == 0:
        return (-b / (2 * a),)
    return (q / a, c / q)


def _unit_interval_roots(coefficients):
    """Isolate cubic roots between its analytic critical points, then bisect.

    Includes repeated roots explicitly at critical points. All work happens
    on [0,1] after spatial scaling, avoiding ill-conditioned physical units.
    """
    c0, c1, c2, c3 = coefficients
    if c3 == 0:
        return tuple(x for x in _quadratic_roots(c2, c1, c0) if 0 <= x <= 1)
    def polynomial(x):
        return ((c3 * x + c2) * x + c1) * x + c0
    cuts = sorted({0.0, 1.0} | {x for x in _quadratic_roots(3*c3, 2*c2, c1) if 0 < x < 1})
    tolerance = 32 * math.ulp(1.0) * (abs(c0) + abs(c1) + abs(c2) + abs(c3))
    roots = [x for x in cuts if abs(polynomial(x)) <= tolerance]
    for left, right in zip(cuts, cuts[1:]):
        fl, fr = polynomial(left), polynomial(right)
        if fl == 0 or fr == 0 or (fl < 0) == (fr < 0):
            continue
        for _ in range(60):
            midpoint = (left + right) * 0.5
            fm = polynomial(midpoint)
            if fm == 0:
                left = right = midpoint
                break
            if (fm < 0) == (fl < 0):
                left, fl = midpoint, fm
            else:
                right = midpoint
        roots.append((left + right) * 0.5)
    return roots


def reference_min_distance(piece_a, piece_b):
    """Return ``(minimum 3D distance, earliest minimizing absolute time)``."""
    if not isinstance(piece_a, TrajectoryPiece) or not isinstance(piece_b, TrajectoryPiece):
        raise ValueError("closest approach requires TrajectoryPiece inputs")
    start, end = max(piece_a.t0, piece_b.t0), min(piece_a.t1, piece_b.t1)
    if start > end:
        return math.inf, None
    pa, pb = piece_a.position_at(start), piece_b.position_at(start)
    va, vb = piece_a.velocity_at(start), piece_b.velocity_at(start)
    duration = end - start
    aa, ab = piece_a.acceleration, piece_b.acceleration
    half_d2 = 0.5 * duration * duration
    r0, r1, r2 = pa[0] - pb[0], pa[1] - pb[1], pa[2] - pb[2]
    v0, v1, v2 = (va[0] - vb[0]) * duration, (va[1] - vb[1]) * duration, (va[2] - vb[2]) * duration
    q0, q1, q2 = (aa[0] - ab[0]) * half_d2, (aa[1] - ab[1]) * half_d2, (aa[2] - ab[2]) * half_d2
    if not (
        math.isfinite(r0) and math.isfinite(r1) and math.isfinite(r2)
        and math.isfinite(v0) and math.isfinite(v1) and math.isfinite(v2)
        and math.isfinite(q0) and math.isfinite(q1) and math.isfinite(q2)
    ):
        raise ValueError("relative trajectory exceeds finite floating-point range")
    scale = max(abs(r0), abs(r1), abs(r2), abs(v0), abs(v1), abs(v2), abs(q0), abs(q1), abs(q2))
    if scale == 0:
        return 0.0, start
    r = (r0 / scale, r1 / scale, r2 / scale)
    v = (v0 / scale, v1 / scale, v2 / scale)
    q = (q0 / scale, q1 / scale, q2 / scale)
    def dot(a, b):
        return math.fsum((a[0] * b[0], a[1] * b[1], a[2] * b[2]))
    coefficients = (dot(r, v), dot(v, v) + 2*dot(r, q), 3*dot(v, q), 2*dot(q, q))
    candidates = [0.0, 1.0] + list(_unit_interval_roots(coefficients))
    best = min(
        (
            math.hypot(
                r[0] + u * (v[0] + u * q[0]),
                r[1] + u * (v[1] + u * q[1]),
                r[2] + u * (v[2] + u * q[2]),
            ),
            u,
        )
        for u in candidates
    )
    distance = best[0] * scale
    if not math.isfinite(distance):
        raise ValueError("separation exceeds finite floating-point range")
    return distance, start + best[1] * duration


def min_distance(piece_a, piece_b):
    """Closest approach, using the optional native equivalent when available."""
    if not isinstance(piece_a, TrajectoryPiece) or not isinstance(piece_b, TrajectoryPiece):
        raise ValueError("closest approach requires TrajectoryPiece inputs")
    if COMPILED_AVAILABLE:
        return fast_min_distance(piece_a, piece_b)
    return reference_min_distance(piece_a, piece_b)


def _swept_bounds(piece):
    ending = piece.position_at(piece.t1)
    duration = piece.t1 - piece.t0
    p0, v0, a0 = piece.position0, piece.velocity0, piece.acceleration
    low0, high0 = (p0[0], ending[0]) if p0[0] <= ending[0] else (ending[0], p0[0])
    low1, high1 = (p0[1], ending[1]) if p0[1] <= ending[1] else (ending[1], p0[1])
    low2, high2 = (p0[2], ending[2]) if p0[2] <= ending[2] else (ending[2], p0[2])
    if a0[0] != 0:
        st = -v0[0] / a0[0]
        if 0 < st < duration:
            val = p0[0] + v0[0] * st + 0.5 * a0[0] * st * st
            if not math.isfinite(val):
                raise ValueError("swept bounds exceed finite floating-point range")
            if val < low0:
                low0 = val
            elif val > high0:
                high0 = val
    if a0[1] != 0:
        st = -v0[1] / a0[1]
        if 0 < st < duration:
            val = p0[1] + v0[1] * st + 0.5 * a0[1] * st * st
            if not math.isfinite(val):
                raise ValueError("swept bounds exceed finite floating-point range")
            if val < low1:
                low1 = val
            elif val > high1:
                high1 = val
    if a0[2] != 0:
        st = -v0[2] / a0[2]
        if 0 < st < duration:
            val = p0[2] + v0[2] * st + 0.5 * a0[2] * st * st
            if not math.isfinite(val):
                raise ValueError("swept bounds exceed finite floating-point range")
            if val < low2:
                low2 = val
            elif val > high2:
                high2 = val
    return (low0, low1, low2), (high0, high1, high2)


def audit_trajectories(pieces, min_distance_m=8.0):
    """Closest conflict per unordered UAV pair, using max per-UAV threshold.

    A threshold mapping must contain every input UAV ID, even if broad-phase
    screening finds no candidates. Piece boundaries are inclusive. Returned
    pair orientation follows sweep order and needs no ordering of IDs.
    """
    pieces = list(pieces)
    if any(not isinstance(piece, TrajectoryPiece) for piece in pieces):
        raise ValueError("audit requires TrajectoryPiece inputs")
    if isinstance(min_distance_m, Mapping):
        required = {}
        for piece in pieces:
            if piece.uav_id not in min_distance_m:
                raise ValueError("missing minimum distance for UAV %r" % (piece.uav_id,))
            required[piece.uav_id] = _finite(min_distance_m[piece.uav_id], "min_distance_m")
            if required[piece.uav_id] < 0:
                raise ValueError("minimum distance must be nonnegative")
    else:
        threshold = _finite(min_distance_m, "min_distance_m")
        if threshold < 0:
            raise ValueError("minimum distance must be nonnegative")
        required = {piece.uav_id: threshold for piece in pieces}
    if len(pieces) < 2:
        return []
    boxes = [_swept_bounds(piece) for piece in pieces]
    # Select the useful sweep axis rather than degenerating for vertical or
    # horizontal aligned fleets. Min/max bounds avoid overflow-prone centers.
    spreads = [max(box[0][axis] for box in boxes) - min(box[0][axis] for box in boxes)
               for axis in range(3)]
    axis = max(range(3), key=spreads.__getitem__)
    maximum_required = max(required.values())
    order = sorted(range(len(pieces)), key=lambda index: boxes[index][0][axis])
    active = []
    conflicts = {}
    for index in order:
        current = pieces[index]
        lower, upper = boxes[index]
        limit_val = lower[axis] - maximum_required
        active = [other for other in active if boxes[other][1][axis] >= limit_val]
        req_curr = required[current.uav_id]
        for other in active:
            previous = pieces[other]
            if current.uav_id == previous.uav_id:
                continue
            if max(current.t0, previous.t0) > min(current.t1, previous.t1):
                continue
            req_prev = required[previous.uav_id]
            threshold = req_curr if req_curr >= req_prev else req_prev
            other_lower, other_upper = boxes[other]
            if (
                lower[0] - other_upper[0] > threshold or other_lower[0] - upper[0] > threshold
                or lower[1] - other_upper[1] > threshold or other_lower[1] - upper[1] > threshold
                or lower[2] - other_upper[2] > threshold or other_lower[2] - upper[2] > threshold
            ):
                continue
            distance, time = min_distance(previous, current)
            # A few ulps allow an analytically touching pair to remain inclusive
            # after polynomial evaluation without hiding meaningful separation.
            tolerance = 16 * math.ulp(max(1.0, threshold))
            if distance > threshold + tolerance:
                continue
            pair = frozenset((previous.uav_id, current.uav_id))
            candidate = Conflict(previous.uav_id, current.uav_id, time, distance, threshold)
            old = conflicts.get(pair)
            if old is None or (distance, time) < (old.distance_m, old.time_s):
                conflicts[pair] = candidate
        active.append(index)
    return list(conflicts.values())
