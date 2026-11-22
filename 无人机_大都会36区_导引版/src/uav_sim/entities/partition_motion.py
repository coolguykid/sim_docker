"""Exact proposed motion for the existing UAV class, with no route selection."""
from dataclasses import dataclass, replace
import math
from uav_sim.algorithms.continuous_motion import integrate_motion, time_to_distance
from uav_sim.algorithms.continuous_safety import TrajectoryPiece


class PhysicalInvariantError(RuntimeError):
    pass


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mul(a, b):
    return (a[0] * b, a[1] * b, a[2] * b)


def norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def lerp(a, b, t):
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


@dataclass
class RoadState:
    uav_id: int
    node_id: str
    region_id: int
    lane_id: int
    position: tuple
    goal: object
    speed: float = 0.0
    status: str = "WAITING"
    route: tuple = ()
    route_index: int = 0
    progress: float = 0.0
    edge_id: str = ""
    transit_ramp: object = None
    transit_direction: str = ""
    entry_lane: int = 0
    exit_lane: int = 0
    arrival_time: object = None
    visited_regions: tuple = ()
    last_acceleration: float = 0.0


@dataclass(frozen=True)
class PreparedMotion:
    state: RoadState
    pieces: tuple
    crossed_edges: tuple


def endpoints(edge, state, graph):
    a, b = edge.points[0], edge.points[-1]
    metadata = graph.metadata.get("ramp_fragments", {}).get(edge.edge_id)
    if metadata and state.transit_ramp is not None:
        initial = metadata["offsets_at_entry"][
            min(state.entry_lane, len(metadata["offsets_at_entry"]) - 1)
        ]
        final = metadata["offsets_at_exit"][
            min(state.exit_lane, len(metadata["offsets_at_exit"]) - 1)
        ]
        return add(a, lerp(initial, final, metadata["fraction_start"])), add(
            b, lerp(initial, final, metadata["fraction_end"])
        )
    lane = state.lane_id
    if edge.offsets_start:
        a = add(a, edge.offsets_start[min(lane, len(edge.offsets_start) - 1)])
    if edge.offsets_end:
        b = add(b, edge.offsets_end[min(lane, len(edge.offsets_end) - 1)])
    return a, b


def node_position(graph, node_id, lane):
    node = graph.nodes[node_id]
    for edge in graph.edges.values():
        offsets = (
            edge.offsets_start
            if edge.source == node_id
            else edge.offsets_end
            if edge.target == node_id
            else ()
        )
        if offsets:
            return add(node.position, offsets[min(lane, len(offsets) - 1)])
    return node.position


def advance(state, graph, acceleration, now, dt, next_region=None):
    """Pure candidate step. Maps may be requested at physical boundary events.

    Geometry stays in the simulated world; route choices come from the selected
    original route-server/worker, never from this motion integrator.
    """
    s = replace(state)
    pieces = []
    crossed = []
    elapsed = 0.0
    hops = 0
    if s.status in ("ARRIVED", "NO_ROUTE", "BLOCKED"):
        return PreparedMotion(s, (), ())
    s.status = "FLYING"
    s.last_acceleration = float(acceleration)
    while elapsed < dt - 1e-12 and s.status != "ARRIVED":
        hops += 1
        if hops > 10000:
            raise PhysicalInvariantError("Zero-length road cycle")
        if s.route_index >= len(s.route):
            if s.node_id == s.goal.node_id:
                if (
                    s.lane_id != s.goal.lane_id
                    or norm(sub(s.position, s.goal.position)) > 1e-5
                ):
                    raise PhysicalInvariantError(
                        "Goal node reached in the wrong physical lane"
                    )
                s.status = "ARRIVED"
                s.arrival_time = now + elapsed
                break
            if next_region is not None:
                s.route = tuple(next_region(s, now + elapsed))
                s.route_index = 0
            if not s.route or s.route_index >= len(s.route):
                if s.speed > 1e-8:
                    raise PhysicalInvariantError(
                        "Moving aircraft lacks a safe executable continuation"
                    )
                s.status = "NO_ROUTE"
                break
        edge = graph.edges[s.route[s.route_index]]
        if not s.edge_id:
            if (
                edge.ramp_id is not None
                and not graph.ramp_states.get(edge.ramp_id, True)
                and s.transit_ramp != edge.ramp_id
            ):
                raise PhysicalInvariantError("Closed ramp entry rejected")
            s.edge_id = edge.edge_id
            if edge.kind == "RAMP" and s.transit_ramp != edge.ramp_id:
                s.transit_ramp = edge.ramp_id
                s.entry_lane = s.lane_id
                s.exit_lane = min(s.goal.lane_id, edge.lane_count - 1)
        a, b = endpoints(edge, s, graph)
        vector = sub(b, a)
        length = norm(vector)
        remaining = max(0.0, length - s.progress)
        if remaining > 1e-9:
            cap = edge.speed_limit_mps
            if s.speed > cap + 1e-7:
                raise PhysicalInvariantError(
                    "Lower-speed entry without physical braking"
                )
            speed = min(s.speed, cap)  # numerical root drift only, bounded above
            event = time_to_distance(speed, acceleration, remaining, cap)
            duration = min(dt - elapsed, event)
            result = integrate_motion(speed, acceleration, duration, cap)
            unit = mul(vector, 1.0 / length)
            origin = add(a, mul(unit, s.progress))
            for phase in result.pieces:
                pieces.append(
                    TrajectoryPiece(
                        s.uav_id,
                        now + elapsed + phase.t0,
                        now + elapsed + phase.t1,
                        add(origin, mul(unit, phase.distance0_m)),
                        mul(unit, phase.speed0_mps),
                        mul(unit, phase.accel_mps2),
                    )
                )
            s.progress += result.distance_m
            s.position = add(a, mul(unit, s.progress))
            s.speed = result.speed_mps
            s.last_acceleration = acceleration
            elapsed += duration
            if event > duration + 1e-9:
                break
        s.node_id = edge.target
        s.edge_id = ""
        s.progress = 0.0
        s.route_index += 1
        s.position = b
        crossed.append(edge.edge_id)
        node = graph.nodes[s.node_id]
        terminal = graph.metadata.get("goal_specs", {}).get(s.node_id)
        if terminal is not None:
            # GOAL_APPROACH has already flown the continuous lateral geometry.
            # Updating the lane label here does not change the actual position.
            s.lane_id = int(terminal.get("lane_id", 0))
        if edge.ramp_id is not None and node.pipe_id is not None:
            s.lane_id = s.exit_lane
            s.transit_ramp = None
        if node.region_id != s.region_id:
            s.region_id = node.region_id
            s.visited_regions += (s.region_id,)
        if s.node_id == s.goal.node_id:
            if (
                s.lane_id != s.goal.lane_id
                or norm(sub(s.position, s.goal.position)) > 1e-5
            ):
                raise PhysicalInvariantError(
                    "Arrival position differs from requested target"
                )
            s.status = "ARRIVED"
            s.arrival_time = now + elapsed
    if s.status != "ARRIVED" and elapsed < dt - 1e-12:
        pieces.append(
            TrajectoryPiece(
                s.uav_id,
                now + elapsed,
                now + dt,
                s.position,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
        )
    return PreparedMotion(s, tuple(pieces), tuple(crossed))
