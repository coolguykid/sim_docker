"""Exact scalar motion with finite acceleration, a speed cap, and a stop.

Times in returned pieces are relative to the start of this integration. A cap
or stop changes acceleration at the event; all remaining time is preserved.
Initial overspeed is an error, including small overspeed: callers must resolve
their own numerical tolerances before invoking these physical primitives.
"""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass(frozen=True)
class MotionPiece:
    t0: float
    t1: float
    distance0_m: float
    speed0_mps: float
    accel_mps2: float


@dataclass(frozen=True)
class MotionResult:
    distance_m: float
    speed_mps: float
    pieces: Tuple[MotionPiece, ...]


def _finite(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("%s must be a finite number" % name) from error
    if not math.isfinite(result):
        raise ValueError("%s must be a finite number" % name)
    return result


def _inputs(speed, acceleration, extent, cap, extent_name):
    speed = _finite(speed, "speed_mps")
    acceleration = _finite(acceleration, "accel_mps2")
    extent = _finite(extent, extent_name)
    cap = _finite(cap, "speed_cap_mps")
    if speed < 0 or cap < 0 or extent < 0:
        raise ValueError("speed, speed cap, and %s must be nonnegative" % extent_name)
    if speed > cap:
        raise ValueError("initial speed exceeds speed cap; instantaneous clipping is forbidden")
    return speed, acceleration, extent, cap


def integrate_motion(speed_mps, accel_mps2, duration_seconds, speed_cap_mps):
    """Integrate constant requested acceleration with exact cap/stop events."""
    speed, acceleration, duration, cap = _inputs(
        speed_mps, accel_mps2, duration_seconds, speed_cap_mps, "duration_seconds")
    if duration == 0:
        return MotionResult(0.0, speed, ())
    if (acceleration > 0 and speed == cap) or (acceleration < 0 and speed == 0):
        acceleration = 0.0
    event_time = math.inf
    if acceleration > 0:
        event_time = (cap - speed) / acceleration
    elif acceleration < 0:
        event_time = -speed / acceleration
    active_time = min(duration, event_time)
    distance = speed * active_time + 0.5 * acceleration * active_time * active_time
    next_speed = speed + acceleration * active_time
    pieces = []
    if active_time > 0:
        pieces.append(MotionPiece(0.0, active_time, 0.0, speed, acceleration))
    if event_time <= duration:
        next_speed = cap if acceleration > 0 else 0.0
        if active_time < duration:
            pieces.append(MotionPiece(active_time, duration, distance, next_speed, 0.0))
            distance += next_speed * (duration - active_time)
    # The exact event calculation above handles physical constraints; this bound
    # only removes the last floating-point ulp in a non-event velocity sum.
    next_speed = min(cap, max(0.0, next_speed))
    if not math.isfinite(distance) or not math.isfinite(next_speed):
        raise ValueError("motion exceeds finite floating-point range")
    return MotionResult(distance, next_speed, tuple(pieces))


def time_to_distance(speed_mps, accel_mps2, distance_m, cap_mps):
    """Earliest arrival under the same cap/stop law, or infinity if unreachable."""
    speed, acceleration, distance, cap = _inputs(
        speed_mps, accel_mps2, distance_m, cap_mps, "distance_m")
    if distance == 0:
        return 0.0
    if cap == 0:
        return math.inf
    if acceleration == 0 or (acceleration > 0 and speed == cap):
        return distance / speed if speed > 0 else math.inf
    if acceleration < 0 and speed == 0:
        return math.inf
    event_speed = cap if acceleration > 0 else 0.0
    event_time = (event_speed - speed) / acceleration
    event_distance = (0.5 * speed + 0.5 * event_speed) * event_time
    if not math.isfinite(event_distance):
        raise ValueError("event distance exceeds finite floating-point range")
    # Edge-length subtraction and prior integration can each contribute a few
    # ulps. Near a stop the inverse quadratic amplifies that distance error
    # into a spurious residual speed, so retain the physical stop event within
    # 32 local distance ulps. There is deliberately no absolute metre floor:
    # physically earlier goals remain earlier even for tiny stopping distances.
    event_tolerance = (32 if acceleration < 0 else 4) * math.ulp(event_distance)
    if abs(distance - event_distance) <= event_tolerance:
        return event_time
    if distance > event_distance:
        if acceleration < 0:
            return math.inf
        result = event_time + (distance - event_distance) / cap
    elif distance == event_distance:
        result = event_time
    else:
        discriminant = speed * speed + 2.0 * acceleration * distance
        if not math.isfinite(discriminant):
            raise ValueError("arrival calculation exceeds finite floating-point range")
        # The stable quadratic form avoids subtracting two nearly equal speeds.
        result = 2.0 * distance / (speed + math.sqrt(max(0.0, discriminant)))
    if not math.isfinite(result):
        raise ValueError("arrival time exceeds finite floating-point range")
    return result
