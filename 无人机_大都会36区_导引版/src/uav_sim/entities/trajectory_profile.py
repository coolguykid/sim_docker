"""Cached exact motion over an already selected regional road prefix.

This module chooses no route. It computes a backward braking envelope and a
forward acceleration envelope, then stores at most three physical phases per
positive-length edge. Arbitrary-time samples use a binary search and one
quadratic evaluation; they never replay previous edges or integration steps.
"""

from bisect import bisect_right
from dataclasses import dataclass, replace
import math
from typing import Tuple

from uav_sim.algorithms.continuous_safety import TrajectoryPiece
from .partition_motion import PhysicalInvariantError, RoadState, endpoints


def _finite(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("%s must be finite" % name) from error
    if not math.isfinite(value):
        raise ValueError("%s must be finite" % name)
    return value


def _add_distance(position, unit, distance):
    return tuple(p + u * distance for p, u in zip(position, unit))


def _retime(state, delta):
    return replace(
        state,
        arrival_time=None if state.arrival_time is None else state.arrival_time + delta,
    )


@dataclass(frozen=True)
class _PhaseState:
    state: RoadState
    acceleration: float


@dataclass(frozen=True)
class PrefixTrajectory:
    start_time: float
    end_time: float
    pieces: Tuple[TrajectoryPiece, ...]
    _source_state: RoadState
    _end_state: RoadState
    _phase_states: Tuple[_PhaseState, ...]
    _phase_starts: Tuple[float, ...]
    _region_events: Tuple[Tuple[float, RoadState], ...] = ()

    @property
    def duration(self):
        return self.end_time - self.start_time

    @property
    def end_state(self):
        # RoadState is mutable in the original UAV interface. Never expose a
        # cached snapshot directly to callers that commit or edit that state.
        return replace(self._end_state)

    @property
    def region_events(self):
        """Exact physical crossings, with independent mutable state copies."""
        return tuple((when, replace(state)) for when, state in self._region_events)

    def sample(self, abs_time):
        """Return a fresh RoadState at any finite absolute physical time."""
        time = _finite(abs_time, "abs_time")
        if time < self.start_time:
            return replace(self._source_state)
        if time >= self.end_time or not self.pieces:
            return self.end_state
        index = bisect_right(self._phase_starts, time) - 1
        piece, metadata = self.pieces[index], self._phase_states[index]
        state, acceleration = metadata.state, metadata.acceleration
        elapsed = time - piece.t0
        distance = state.speed * elapsed + 0.5 * acceleration * elapsed * elapsed
        return replace(
            state,
            position=piece.position_at(time),
            progress=state.progress + distance,
            speed=max(0.0, state.speed + acceleration * elapsed),
            last_acceleration=acceleration,
        )

    def shifted(self, start_time):
        """Retime this profile without rebuilding its geometry or speed plan."""
        start = _finite(start_time, "start_time")
        delta = start - self.start_time
        pieces = tuple(
            TrajectoryPiece(
                p.uav_id,
                start + (p.t0 - self.start_time),
                start + (p.t1 - self.start_time),
                p.position0,
                p.velocity0,
                p.acceleration,
            )
            for p in self.pieces
        )
        end = start + self.duration
        if not math.isfinite(end):
            raise ValueError("shifted profile exceeds finite time range")
        final = _retime(self._end_state, delta)
        if final.status == "ARRIVED":
            final = replace(final, arrival_time=end)
        states = tuple(
            _PhaseState(_retime(p.state, delta), p.acceleration)
            for p in self._phase_states
        )
        return PrefixTrajectory(
            start,
            end,
            pieces,
            _retime(self._source_state, delta),
            final,
            states,
            tuple(p.t0 for p in pieces),
            tuple(
                (start + (when - self.start_time), _retime(state, delta))
                for when, state in self._region_events
            ),
        )


@dataclass(frozen=True)
class _Geometry:
    before: RoadState
    after: RoadState
    position: tuple
    unit: tuple
    length: float
    cap: float


def _at_goal(state):
    if state.node_id != state.goal.node_id:
        return False
    if (
        state.lane_id != state.goal.lane_id
        or math.dist(state.position, state.goal.position) > 1e-5
    ):
        raise PhysicalInvariantError(
            "Goal node reached in the wrong physical lane or position"
        )
    return True


def _geometry(state, graph, route, route_index):
    result = []
    current = replace(
        state, route=route, route_index=route_index, status="FLYING", arrival_time=None
    )
    if _at_goal(current):
        return result, current
    for index in range(route_index, len(route)):
        edge_id = route[index]
        if edge_id not in graph.edges:
            raise PhysicalInvariantError("Unknown route edge: %r" % (edge_id,))
        edge = graph.edges[edge_id]
        if edge.source != current.node_id:
            raise PhysicalInvariantError("Route edge is disconnected from current node")
        if current.edge_id and current.edge_id != edge.edge_id:
            raise PhysicalInvariantError(
                "Current edge differs from the selected prefix"
            )
        if edge.ramp_id is not None:
            if (
                not graph.ramp_states.get(edge.ramp_id, True)
                and current.transit_ramp != edge.ramp_id
            ):
                raise PhysicalInvariantError("Closed ramp entry rejected")
            if current.transit_ramp != edge.ramp_id:
                current = replace(
                    current,
                    transit_ramp=edge.ramp_id,
                    entry_lane=current.lane_id,
                    exit_lane=min(current.goal.lane_id, edge.lane_count - 1),
                )
            fragment = graph.metadata.get("ramp_fragments", {}).get(edge.edge_id, {})
            current = replace(
                current,
                transit_direction=fragment.get("direction", current.transit_direction),
            )
        current = replace(current, edge_id=edge.edge_id, route_index=index)
        a, b = endpoints(edge, current, graph)
        if (
            len(a) != 3
            or len(b) != 3
            or not all(math.isfinite(float(x)) for x in a + b)
        ):
            raise ValueError(
                "Edge geometry must contain finite three-dimensional points"
            )
        length = math.dist(a, b)
        progress = _finite(current.progress, "progress")
        if progress < 0 or progress > length + 32 * math.ulp(length):
            raise PhysicalInvariantError(
                "Current progress lies outside its physical edge"
            )
        progress = min(progress, length)
        unit = (
            tuple((end - start) / length for start, end in zip(a, b))
            if length
            else (0.0, 0.0, 0.0)
        )
        position = _add_distance(a, unit, progress)
        if math.dist(position, current.position) > 1e-5:
            raise PhysicalInvariantError(
                "Route would require a discontinuous position change"
            )
        current = replace(current, position=position, progress=progress)
        cap = _finite(edge.speed_limit_mps, "edge speed limit")
        remaining = length - progress
        if cap < 0 or (remaining > 0 and cap == 0):
            raise PhysicalInvariantError(
                "Positive-length edge requires a positive speed limit"
            )
        after = replace(
            current,
            node_id=edge.target,
            edge_id="",
            progress=0.0,
            route_index=index + 1,
            position=b,
        )
        node = graph.nodes[edge.target]
        terminal = graph.metadata.get("goal_specs", {}).get(edge.target)
        if terminal is not None:
            after = replace(after, lane_id=int(terminal.get("lane_id", 0)))
        if edge.ramp_id is not None and node.pipe_id is not None:
            after = replace(
                after, lane_id=after.exit_lane, transit_ramp=None, transit_direction=""
            )
        if node.region_id != after.region_id:
            after = replace(
                after,
                region_id=node.region_id,
                visited_regions=after.visited_regions + (node.region_id,),
            )
        result.append(_Geometry(current, after, position, unit, remaining, cap))
        current = after
        if _at_goal(current):
            break
    return result, current


def _endpoint_speeds(
    geometry, initial_speed, accel, decel, must_stop, stop_at_boundaries=False
):
    count = len(geometry)
    if not count:
        if initial_speed != 0:
            raise PhysicalInvariantError(
                "Moving aircraft lacks an executable continuation"
            )
        return (0.0,)
    caps = [part.cap for part in geometry]
    limits = (
        [caps[0]] + [min(caps[i - 1], caps[i]) for i in range(1, count)] + [caps[-1]]
    )
    # The road geometry consists of straight segments. A nonzero scalar
    # speed through a corner would instantaneously rotate the velocity vector
    # (infinite acceleration). Stop before changing heading, including when
    # zero-length portal aliases separate the two real segments. Collinear
    # fragments retain their through speed. The tiny unit-vector tolerance
    # covers only roundoff from independently split copies of the same line.
    previous = None
    for index, part in enumerate(geometry):
        if part.length == 0:
            continue
        if (
            previous is not None
            and math.dist(geometry[previous].unit, part.unit) > 1e-12
        ):
            for boundary in range(previous + 1, index + 1):
                limits[boundary] = 0.0
        previous = index
    if stop_at_boundaries:
        for index, part in enumerate(geometry):
            if part.before.region_id != part.after.region_id:
                limits[index + 1] = 0.0
    if must_stop:
        limits[-1] = 0.0
    backward = limits[:]
    for i in range(count - 1, -1, -1):
        reachable = math.sqrt(backward[i + 1] ** 2 + 2 * decel * geometry[i].length)
        backward[i] = min(backward[i], reachable)
    if initial_speed > caps[0]:
        raise PhysicalInvariantError(
            "Initial speed exceeds edge cap; instantaneous clipping is forbidden"
        )
    if initial_speed > backward[0] + 32 * math.ulp(max(initial_speed, backward[0])):
        raise PhysicalInvariantError(
            "Insufficient distance for physical braking along this prefix"
        )
    speeds = [initial_speed]
    for i, part in enumerate(geometry):
        reachable = math.sqrt(speeds[-1] ** 2 + 2 * accel * part.length)
        speeds.append(min(backward[i + 1], reachable))
    return tuple(speeds)


def build_prefix_profile(
    state,
    graph,
    edge_ids=None,
    start_time=0.0,
    max_accel=10.0,
    max_decel=10.0,
    stop_at_end=True,
    distance_limit_m=None,
    stop_at_boundaries=False,
):
    """Plan an exact trapezoidal trajectory over the supplied selected edges.

    Omitting edge_ids uses state.route from state.route_index, including any
    current edge progress. Explicit edge_ids replaces that remaining prefix.
    An unknown continuation always ends at rest. At a physically valid goal,
    stop_at_end=False permits arrival at the maximum reachable terminal speed.
    Closed ramps may be completed by aircraft already in that same ramp.
    A positive distance_limit_m ends the certificate at an actual stationary
    point on the selected route. It retains the current edge and progress for
    later continuation, without declaring a portal crossing or an arrival.
    stop_at_boundaries additionally requires a physical stop at each actual
    region crossing. region_events records these crossings at exact times.
    """
    start = _finite(start_time, "start_time")
    accel, decel = _finite(max_accel, "max_accel"), _finite(max_decel, "max_decel")
    if accel <= 0 or decel <= 0:
        raise ValueError("max_accel and max_decel must be positive")
    if distance_limit_m is not None:
        distance_limit_m = _finite(distance_limit_m, "distance_limit_m")
        if distance_limit_m <= 0:
            raise ValueError("distance_limit_m must be positive")
    speed = _finite(state.speed, "initial speed")
    if speed < 0:
        raise ValueError("initial speed must be nonnegative")
    try:
        position = tuple(_finite(x, "initial position") for x in state.position)
    except TypeError as error:
        raise ValueError(
            "initial position must contain three finite coordinates"
        ) from error
    if len(position) != 3:
        raise ValueError("initial position must contain three finite coordinates")
    source = replace(state, position=position)
    if edge_ids is None:
        route, index = tuple(state.route), state.route_index
        if (
            isinstance(index, bool)
            or not isinstance(index, int)
            or not 0 <= index <= len(route)
        ):
            raise ValueError("route_index lies outside the selected route")
    else:
        route, index = tuple(edge_ids), 0
    if source.edge_id and (index >= len(route) or route[index] != source.edge_id):
        raise PhysicalInvariantError(
            "Current edge must be first in the remaining selected prefix"
        )
    geometry, final = _geometry(source, graph, route, index)
    if distance_limit_m is not None:
        selected, remaining = [], distance_limit_m
        for part in geometry:
            if part.length <= remaining:
                selected.append(part)
                remaining -= part.length
                continue
            # This endpoint is an interior physical point, not edge.target.
            # Keeping the source node plus edge/progress permits exact resume.
            # Zero-length aliases preceding the cut were already consumed.
            final = replace(
                part.before,
                position=_add_distance(part.position, part.unit, remaining),
                progress=part.before.progress + remaining,
                speed=0.0,
                status="FLYING",
                arrival_time=None,
                last_acceleration=0.0,
            )
            selected.append(replace(part, after=final, length=remaining))
            geometry = selected
            break
    at_goal = _at_goal(final)
    speeds = _endpoint_speeds(
        geometry,
        speed,
        accel,
        decel,
        bool(stop_at_end) or not at_goal,
        bool(stop_at_boundaries),
    )
    pieces, phase_states = [], []
    region_events = []
    elapsed = 0.0

    def append_crossing(part, index):
        if part.before.region_id != part.after.region_id:
            goal_reached = _at_goal(part.after)
            when = start + elapsed
            state_at_crossing = replace(
                part.after,
                speed=speeds[index + 1],
                status="ARRIVED" if goal_reached else "FLYING",
                arrival_time=when if goal_reached else None,
                last_acceleration=0.0,
            )
            region_events.append((when, state_at_crossing))

    for i, part in enumerate(geometry):
        if part.length == 0:
            append_crossing(part, i)
            continue
        initial, terminal = speeds[i : i + 2]
        peak_squared = (
            2 * part.length + initial**2 / accel + terminal**2 / decel
        ) / (1 / accel + 1 / decel)
        peak = min(part.cap, math.sqrt(max(0.0, peak_squared)))
        minimum_peak = max(initial, terminal)
        if peak < minimum_peak:
            if minimum_peak - peak > 32 * math.ulp(minimum_peak):
                raise PhysicalInvariantError("Infeasible per-edge speed envelope")
            peak = minimum_peak
        accelerating_distance = (peak - initial) * ((peak + initial) / (2 * accel))
        braking_distance = (peak - terminal) * ((peak + terminal) / (2 * decel))
        cruising_distance = part.length - accelerating_distance - braking_distance
        tolerance = 64 * math.ulp(
            max(part.length, accelerating_distance, braking_distance)
        )
        # Endpoint speeds come from sqrt(v²+2*a*s). On a short edge at
        # substantial entry speed, rounding is set by the kinetic-energy
        # terms, not by s alone. Account for those input ULPs explicitly;
        # a fixed metre tolerance would hide real infeasibility at tiny scale.
        tolerance += 8 * (
            math.ulp(initial * initial / (2 * accel))
            + math.ulp(peak * peak / (2 * accel))
            + math.ulp(terminal * terminal / (2 * decel))
            + math.ulp(peak * peak / (2 * decel))
        )
        if cruising_distance < -tolerance:
            raise PhysicalInvariantError(
                "Speed phases require more than the physical edge length"
            )
        cruising_distance = max(0.0, cruising_distance)
        phases = (
            (initial, accel, (peak - initial) / accel),
            (peak, 0.0, cruising_distance / peak if peak else 0.0),
            (peak, -decel, (peak - terminal) / decel),
        )
        travelled = 0.0
        for phase_speed, acceleration, duration in phases:
            if duration <= 0:
                continue
            ending = math.fsum((elapsed, duration))
            t0, t1 = start + elapsed, start + ending
            if not math.isfinite(t1):
                raise ValueError("profile exceeds finite time range")
            origin = _add_distance(part.position, part.unit, travelled)
            snapshot = replace(
                part.before,
                position=origin,
                progress=part.before.progress + travelled,
                speed=phase_speed,
                status="FLYING",
                last_acceleration=acceleration,
            )
            pieces.append(
                TrajectoryPiece(
                    source.uav_id,
                    t0,
                    t1,
                    origin,
                    tuple(x * phase_speed for x in part.unit),
                    tuple(x * acceleration for x in part.unit),
                )
            )
            phase_states.append(_PhaseState(snapshot, acceleration))
            travelled += (
                phase_speed * duration + 0.5 * acceleration * duration * duration
            )
            elapsed = ending
        append_crossing(part, i)
    end = start + elapsed
    final = replace(
        final,
        speed=speeds[-1],
        status="ARRIVED" if at_goal else "FLYING",
        arrival_time=end if at_goal else None,
        last_acceleration=0.0,
    )
    return PrefixTrajectory(
        start,
        end,
        tuple(pieces),
        source,
        final,
        tuple(phase_states),
        tuple(p.t0 for p in pieces),
        tuple(region_events),
    )
