"""Optional native compilation of the continuous closest-approach equations.

This is the same normalized cubic-derivative/root-bracketing calculation as
continuous_safety, including repeated roots and sixty bisection iterations.
No fast-math transformations or relaxed distance thresholds are enabled.
Without Numba the reference solver remains available to the application.
"""
import math
import itertools

try:
    from numba import njit
except ImportError:
    COMPILED_AVAILABLE = False

    def _native(function):
        return function

else:
    COMPILED_AVAILABLE = True

    def _native(function):
        return njit(cache=True, nogil=True, fastmath=False)(function)


@_native
def _sum3(a, b, c):
    # Error-free TwoSum steps retain small cancellation terms. Inputs here
    # are products of normalized coordinates, so no intermediate overflow.
    x = a + b
    bb = x - a
    error = (a - (x - bb)) + (b - bb)
    result = x + c
    bb = result - x
    error += (x - (result - bb)) + (c - bb)
    return result + error


@_native
def _norm3(x, y, z):
    scale = max(abs(x), abs(y), abs(z))
    if scale == 0.0:
        return 0.0
    return scale * math.sqrt((x / scale) ** 2 + (y / scale) ** 2 + (z / scale) ** 2)


@_native
def _quadratic(a, b, c):
    if a == 0.0:
        return (math.nan, math.nan) if b == 0.0 else (-c / b, math.nan)
    scale = max(abs(a), abs(b), abs(c))
    a, b, c = a / scale, b / scale, c / scale
    discriminant = b * b - 4 * a * c
    rounding = 16 * 2.220446049250313e-16 * (b * b + abs(4 * a * c))
    if discriminant < -rounding:
        return math.nan, math.nan
    root = math.sqrt(max(0.0, discriminant))
    q = -0.5 * (b + math.copysign(root, b))
    if q == 0.0:
        return -b / (2 * a), math.nan
    return q / a, c / q


@_native
def _value(c0, c1, c2, c3, x):
    return ((c3 * x + c2) * x + c1) * x + c0


@_native
def _linear_minimum(a0, a1, pa, va, aa, b0, b1, pb, vb, ab):
    start, end = max(a0, b0), min(a1, b1)
    if start > end:
        return math.inf, math.nan
    da, db = start - a0, start - b0
    duration = end - start
    rx = (pa[0] + va[0] * da + 0.5 * aa[0] * da * da) - (
        pb[0] + vb[0] * db + 0.5 * ab[0] * db * db
    )
    ry = (pa[1] + va[1] * da + 0.5 * aa[1] * da * da) - (
        pb[1] + vb[1] * db + 0.5 * ab[1] * db * db
    )
    rz = (pa[2] + va[2] * da + 0.5 * aa[2] * da * da) - (
        pb[2] + vb[2] * db + 0.5 * ab[2] * db * db
    )
    vx = ((va[0] + aa[0] * da) - (vb[0] + ab[0] * db)) * duration
    vy = ((va[1] + aa[1] * da) - (vb[1] + ab[1] * db)) * duration
    vz = ((va[2] + aa[2] * da) - (vb[2] + ab[2] * db)) * duration
    if not (
        math.isfinite(rx)
        and math.isfinite(ry)
        and math.isfinite(rz)
        and math.isfinite(vx)
        and math.isfinite(vy)
        and math.isfinite(vz)
    ):
        raise ValueError("relative trajectory exceeds finite floating-point range")
    scale = max(abs(rx), abs(ry), abs(rz), abs(vx), abs(vy), abs(vz))
    if not math.isfinite(scale):
        raise ValueError("relative trajectory exceeds finite floating-point range")
    if scale == 0.0:
        return 0.0, start
    rx, ry, rz = rx / scale, ry / scale, rz / scale
    vx, vy, vz = vx / scale, vy / scale, vz / scale
    denominator = _sum3(vx * vx, vy * vy, vz * vz)
    best = _norm3(rx, ry, rz)
    best_u = 0.0
    ending = _norm3(rx + vx, ry + vy, rz + vz)
    if ending < best:
        best, best_u = ending, 1.0
    if denominator != 0.0:
        u = -_sum3(rx * vx, ry * vy, rz * vz) / denominator
        if 0.0 <= u <= 1.0:
            value = _norm3(rx + u * vx, ry + u * vy, rz + u * vz)
            if value < best or (value == best and u < best_u):
                best, best_u = value, u
    result = best * scale
    if not math.isfinite(result):
        raise ValueError("separation exceeds finite floating-point range")
    return result, start + best_u * duration


@_native
def _minimum(a0, a1, pa, va, aa, b0, b1, pb, vb, ab):
    # Equal accelerations cancel exactly. The derivative is then linear,
    # the same exact degree reduction used by the general cubic solver.
    if aa[0] == ab[0] and aa[1] == ab[1] and aa[2] == ab[2]:
        return _linear_minimum(a0, a1, pa, va, aa, b0, b1, pb, vb, ab)
    start, end = max(a0, b0), min(a1, b1)
    if start > end:
        return math.inf, math.nan
    da, db = start - a0, start - b0
    duration = end - start
    r = [0.0, 0.0, 0.0]
    v = [0.0, 0.0, 0.0]
    q = [0.0, 0.0, 0.0]
    scale = 0.0
    for k in range(3):
        p_a = pa[k] + va[k] * da + 0.5 * aa[k] * da * da
        p_b = pb[k] + vb[k] * db + 0.5 * ab[k] * db * db
        v_a = va[k] + aa[k] * da
        v_b = vb[k] + ab[k] * db
        if not (
            math.isfinite(p_a)
            and math.isfinite(p_b)
            and math.isfinite(v_a)
            and math.isfinite(v_b)
        ):
            raise ValueError(
                "trajectory evaluation exceeds finite floating-point range"
            )
        r[k] = p_a - p_b
        v[k] = (v_a - v_b) * duration
        q[k] = 0.5 * (aa[k] - ab[k]) * duration * duration
        if not (math.isfinite(r[k]) and math.isfinite(v[k]) and math.isfinite(q[k])):
            raise ValueError("relative trajectory exceeds finite floating-point range")
        scale = max(scale, abs(r[k]), abs(v[k]), abs(q[k]))
    if scale == 0.0:
        return 0.0, start
    for k in range(3):
        r[k] /= scale
        v[k] /= scale
        q[k] /= scale
    c0 = _sum3(r[0] * v[0], r[1] * v[1], r[2] * v[2])
    c1 = _sum3(v[0] * v[0], v[1] * v[1], v[2] * v[2]) + 2 * _sum3(
        r[0] * q[0], r[1] * q[1], r[2] * q[2]
    )
    c2 = 3 * _sum3(v[0] * q[0], v[1] * q[1], v[2] * q[2])
    c3 = 2 * _sum3(q[0] * q[0], q[1] * q[1], q[2] * q[2])
    roots = [0.0, 1.0]
    if c3 == 0.0:
        one, two = _quadratic(c2, c1, c0)
        if 0.0 <= one <= 1.0:
            roots.append(one)
        if 0.0 <= two <= 1.0:
            roots.append(two)
    else:
        cuts = [0.0, 1.0]
        one, two = _quadratic(3 * c3, 2 * c2, c1)
        if 0.0 < one < 1.0:
            cuts.append(one)
        if 0.0 < two < 1.0 and two != one:
            cuts.append(two)
        cuts.sort()
        tolerance = 32 * 2.220446049250313e-16 * (abs(c0) + abs(c1) + abs(c2) + abs(c3))
        for x in cuts:
            if abs(_value(c0, c1, c2, c3, x)) <= tolerance:
                roots.append(x)
        for i in range(len(cuts) - 1):
            left, right = cuts[i], cuts[i + 1]
            fl, fr = _value(c0, c1, c2, c3, left), _value(c0, c1, c2, c3, right)
            if fl == 0.0 or fr == 0.0 or (fl < 0.0) == (fr < 0.0):
                continue
            for iteration in range(60):
                midpoint = (left + right) * 0.5
                fm = _value(c0, c1, c2, c3, midpoint)
                if fm == 0.0:
                    left = right = midpoint
                    break
                if (fm < 0.0) == (fl < 0.0):
                    left, fl = midpoint, fm
                else:
                    right = midpoint
            roots.append((left + right) * 0.5)
    best = math.inf
    best_u = 0.0
    for u in roots:
        distance = _norm3(
            r[0] + u * (v[0] + u * q[0]),
            r[1] + u * (v[1] + u * q[1]),
            r[2] + u * (v[2] + u * q[2]),
        )
        if distance < best or (distance == best and u < best_u):
            best, best_u = distance, u
    result = best * scale
    if not math.isfinite(result):
        raise ValueError("separation exceeds finite floating-point range")
    return result, start + best_u * duration


def fast_min_distance(a, b):
    """Return exact-in-time closest approach for validated trajectory pieces."""
    distance, time = _minimum(
        a.t0,
        a.t1,
        a.position0,
        a.velocity0,
        a.acceleration,
        b.t0,
        b.t1,
        b.position0,
        b.velocity0,
        b.acceleration,
    )
    return distance, None if math.isnan(time) else time


@_native
def _prune_static_pairs(candidate_data, other_data, pairs):
    """Remove time-independent AABB misses in place, keeping input order."""
    kept = 0
    for pair_index in range(len(pairs)):
        index, other_index = pairs[pair_index, 0], pairs[pair_index, 1]
        threshold = max(candidate_data[index, 17], other_data[other_index, 17])
        separated = False
        for axis in range(3):
            if (
                candidate_data[index, 11 + axis] - other_data[other_index, 14 + axis]
                > threshold
                or other_data[other_index, 11 + axis] - candidate_data[index, 14 + axis]
                > threshold
            ):
                separated = True
                break
        if not separated:
            pairs[kept, 0], pairs[kept, 1] = index, other_index
            kept += 1
    return kept


@_native
def _scan_finite(candidate_data, other_data, pairs, shift, monotone):
    best_time = math.inf
    best_distance = math.inf
    best_candidate = -1
    best_other = -1
    checked = 0
    for pair_index in range(len(pairs)):
        index, other_index = pairs[pair_index, 0], pairs[pair_index, 1]
        a, b = candidate_data, other_data
        a0, a1 = a[index, 0] + shift, a[index, 1] + shift
        b0, b1 = b[other_index, 0], b[other_index, 1]
        # A rejected trial needs only its earliest closest-approach conflict.
        # A later phase/pair cannot improve that time. An accepted trial has
        # no best conflict and therefore still evaluates every relevant pair.
        if monotone and a0 > best_time:
            break
        if max(a0, b0) > best_time:
            continue
        if max(a0, b0) > min(a1, b1):
            continue
        threshold = max(a[index, 17], b[other_index, 17])
        # Spatial misses were already removed by _prune_static_pairs; the
        # world-space boxes do not change with this departure-time shift.
        checked += 1
        distance, time = _minimum(
            a0,
            a1,
            (a[index, 2], a[index, 3], a[index, 4]),
            (a[index, 5], a[index, 6], a[index, 7]),
            (a[index, 8], a[index, 9], a[index, 10]),
            b0,
            b1,
            (b[other_index, 2], b[other_index, 3], b[other_index, 4]),
            (b[other_index, 5], b[other_index, 6], b[other_index, 7]),
            (b[other_index, 8], b[other_index, 9], b[other_index, 10]),
        )
        _, exponent = math.frexp(max(1.0, threshold))
        tolerance = 16 * math.ldexp(1.0, exponent - 53)
        if distance <= threshold + tolerance and time < best_time:
            best_time, best_distance = time, distance
            best_candidate, best_other = index, other_index
    return best_candidate, best_other, best_time, best_distance, checked


@_native
def _clear_pair(a, b, index, other_index, lower, upper):
    """The same 28 bisections, without allocating validated phase objects."""
    threshold=max(a[index,17], b[other_index,17])
    _,exponent=math.frexp(max(1.,threshold))
    tolerance=16*math.ldexp(1.,exponent-53)
    checked=0
    for _ in range(28):
        middle=(lower+upper)*.5
        a0,a1=a[index,0]+middle,a[index,1]+middle
        b0,b1=b[other_index,0],b[other_index,1]
        conflict=False
        if max(a0,b0)<=min(a1,b1):
            checked+=1
            distance,_=_minimum(a0,a1,
                (a[index,2],a[index,3],a[index,4]),
                (a[index,5],a[index,6],a[index,7]),
                (a[index,8],a[index,9],a[index,10]),
                b0,b1,
                (b[other_index,2],b[other_index,3],b[other_index,4]),
                (b[other_index,5],b[other_index,6],b[other_index,7]),
                (b[other_index,8],b[other_index,9],b[other_index,10]))
            conflict=distance<=threshold+tolerance
        if conflict:
            lower=middle
        else:
            upper=middle
    return upper,checked


class _FiniteScan:
    def __init__(self, candidates, neighbors):
        import numpy as np

        candidates = tuple(candidates)
        neighbors = tuple(tuple(group) for group in neighbors)
        if len(candidates) != len(neighbors):
            raise ValueError("finite scan needs one neighbor group per candidate")

        def data(record):
            p = record.piece
            if p is None or not math.isfinite(record.t1):
                raise ValueError("finite scan cannot include an infinite hold")
            return (
                p.t0,
                p.t1,
                *p.position0,
                *p.velocity0,
                *p.acceleration,
                *record.lower,
                *record.upper,
                record.threshold,
            )

        # Keep pair traversal order, but build the two integer columns in
        # NumPy rather than allocating one Python tuple for every neighbor.
        flat_records=list(itertools.chain.from_iterable(neighbors))
        counts=np.fromiter(map(len,neighbors),dtype=np.int64,count=len(neighbors))
        identities=np.fromiter(map(id,flat_records),dtype=np.int64,count=len(flat_records))
        _,first,inverse=np.unique(identities,return_index=True,return_inverse=True)
        self.records=tuple(flat_records[int(i)] for i in first)
        self._record_indexes={id(record):i for i,record in enumerate(self.records)}
        self.pairs=np.empty((len(flat_records),2),dtype=np.int64)
        self.pairs[:,0]=np.repeat(np.arange(len(candidates),dtype=np.int64),counts)
        self.pairs[:,1]=inverse
        del flat_records,identities,first,inverse,counts
        self.candidates = np.asarray(
            [data(record) for record in candidates], dtype=np.float64
        ).reshape((-1, 18))
        self.others = np.asarray(
            [data(record) for record in self.records], dtype=np.float64
        ).reshape((-1, 18))
        kept = _prune_static_pairs(
            memoryview(self.candidates), memoryview(self.others), memoryview(self.pairs)
        )
        if kept < len(self.pairs):
            self.pairs = self.pairs[:kept].copy()
        self.monotone = all(a.t0 <= b.t0 for a, b in zip(candidates, candidates[1:]))

    def scan(self, start_time):
        shift = float(start_time)
        if not math.isfinite(shift):
            raise ValueError("finite scan start time must be finite")
        # PEP 3118 buffers avoid reliance on a NumPy C-ABI unbox adapter;
        # geometry is still packed once and passed without copying.
        i, j, time, distance, checked = _scan_finite(
            memoryview(self.candidates),
            memoryview(self.others),
            memoryview(self.pairs),
            shift,
            self.monotone,
        )
        return i, self.records[j] if j >= 0 else None, time, distance, checked

    def clearance_after(self,index,other,lower,upper):
        return _clear_pair(memoryview(self.candidates),memoryview(self.others),index,
                           self._record_indexes[id(other)],lower,upper)


def prepare_finite_scan(candidates, neighbors):
    """Pack stable geometry once; each retiming scans neighbors in native code.

    The caller must exclude its own UID and all infinite holds first. Input
    order is retained, including earliest-conflict tie order. This performs
    the same temporal, AABB and analytic checks as the Python pair loop.
    """
    return _FiniteScan(candidates, neighbors)
