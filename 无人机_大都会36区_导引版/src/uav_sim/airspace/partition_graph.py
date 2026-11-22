"""Virtual 5 km partitions of the existing physical pipes and ramps.

The compiler does not mutate Airspace. Detailed region views and the shared
portal summary are cached per topology version; aircraft can share them.
"""
import heapq
import math
from collections import defaultdict
from copy import copy

from uav_sim.airspace.partition_contracts import (
    Destination, PartitionGraph, PortalArc, PortalSummary, RegionMap, RoadEdge, RoadNode,
)


class FrozenDict(dict):
    """Read-only, pickle-compatible mapping (unlike MappingProxyType)."""

    def _immutable(self, *args, **kwargs):
        raise TypeError("partition views are immutable")

    __setitem__ = __delitem__ = clear = pop = popitem = setdefault = update = _immutable
    __ior__ = _immutable

    def __reduce__(self):
        return (FrozenDict, (dict(self),))


def edge_cost(edge):
    return float(edge.length_m) / max(float(edge.speed_limit_mps), 1.e-12)


def shortest_paths(adjacency, start):
    """Exact node-key Dijkstra, returning distances and incoming edge IDs."""
    distances, previous = {start: 0.}, {}
    queue = [(0., start)]
    while queue:
        distance, node = heapq.heappop(queue)
        if distance != distances[node]:
            continue
        for target, cost, edge_id in adjacency.get(node, ()):
            candidate = distance + cost
            if candidate < distances.get(target, float("inf")):
                distances[target] = candidate
                previous[target] = (node, edge_id)
                heapq.heappush(queue, (candidate, target))
    return distances, previous


def _point(value):
    return tuple(float(v) for v in value)


def _lerp(start, end, t):
    return tuple(a + (b - a) * t for a, b in zip(start, end))


def compile_partition(env, destinations=()):
    """Compile original pipe/ramp centerlines, inserting optional exact goals.

    IDs of original START/END/ramp nodes remain available. A crossing has two
    co-located nodes with a directed zero-length BOUNDARY edge; all movement
    edges are owned by the region containing their centerline midpoint.
    """
    side = 5000.
    xmin, ymin = float(env.x_min), float(env.y_min)
    xmax, ymax = float(env.x_max), float(env.y_max)
    cols = max(1, int(math.ceil((xmax - xmin) / side)))
    rows = max(1, int(math.ceil((ymax - ymin) / side)))
    regions = {}
    for row in range(rows):
        for col in range(cols):
            rid = row * cols + col
            regions[rid] = dict(region_id=rid, row=row, col=col,
                                x_min=xmin + col * side, x_max=min(xmax, xmin + (col + 1) * side),
                                y_min=ymin + row * side, y_max=min(ymax, ymin + (row + 1) * side))

    def owner(position):
        col = max(0, min(cols - 1, int(math.floor((position[0] - xmin) / side))))
        row = max(0, min(rows - 1, int(math.floor((position[1] - ymin) / side))))
        return row * cols + col

    pipes = {int(p.pipe_id): p for p in env.pipelines}
    nodes, edges, ramp_fragments = {}, {}, {}
    pipe_nodes = defaultdict(list)
    goal_specs, goal_anchors = {}, {}

    def pipe_node(node_id, pipe, distance):
        distance = float(distance)
        position = _point(pipe.start_pt + pipe.direction * distance)
        value = RoadNode(str(node_id), owner(position), position, int(pipe.pipe_id), distance)
        if node_id in nodes and nodes[node_id] != value:
            raise ValueError("conflicting node ID: {}".format(node_id))
        nodes[node_id] = value
        pipe_nodes[pipe.pipe_id].append((distance, str(node_id)))

    for pid, pipe in pipes.items():
        pipe_node("P{}_START".format(pid), pipe, 0.)
        pipe_node("P{}_END".format(pid), pipe, pipe.length)
    for ramp in env.ramps:
        for pid, position in ((int(ramp.pipe_a_id), ramp.pos_a), (int(ramp.pipe_b_id), ramp.pos_b)):
            pipe = pipes[pid]
            distance = sum((float(p) - float(s)) * float(d) for p, s, d in zip(position, pipe.start_pt, pipe.direction))
            # Dot products can put an endpoint ramp a few picometers before
            # END, leaving END with no exit. Preserve the endpoint identity
            # explicitly; interior goal distances are never rounded or merged.
            if math.dist(position, pipe.start_pt) < 1.e-7:
                distance = 0.
            elif math.dist(position, pipe.end_pt) < 1.e-7:
                distance = float(pipe.length)
            pipe_node("P{}_R{}".format(pid, ramp.ramp_id), pipe, max(0., min(float(pipe.length), distance)))
    for spec in destinations:
        spec = dict(spec)
        node_id, pid = str(spec["node_id"]), int(spec["pipe_id"])
        pipe, distance = pipes[pid], float(spec["distance_m"])
        lane = int(spec.get("lane_id", 0))
        if not 0. <= distance <= float(pipe.length) or not 0 <= lane < int(pipe.num_lanes):
            raise ValueError("destination lies outside its pipe or lane")
        # A goal is a lane-specific terminal, not an ordinary pass-through
        # centerline node. All traffic may keep traversing the original pipe;
        # only a route to this terminal selects its physical lane approach.
        position = _point(pipe.start_pt + pipe.direction * distance)
        terminal = RoadNode(node_id, owner(position), position, pid, distance)
        if node_id in nodes and nodes[node_id] != terminal:
            raise ValueError("conflicting goal node ID: {}".format(node_id))
        nodes[node_id] = terminal
        start_distance = max(0., distance - 200.)
        anchors = []
        for label, value in (("APPROACH", start_distance), ("POINT", distance)):
            anchor_id = "GOAL:{}:{}".format(node_id, label)
            if anchor_id in nodes:
                raise ValueError("conflicting goal anchor ID: {}".format(anchor_id))
            pipe_node(anchor_id, pipe, value)
            anchors.append(anchor_id)
        goal_anchors[node_id] = (start_distance, anchors[1])
        goal_specs[node_id] = dict(node_id=node_id, pipe_id=pid, distance_m=distance, lane_id=lane)

    def offsets(pipe, distance):
        fraction = distance / float(pipe.length) if pipe.length else 0.
        result = []
        center = _lerp(pipe.start_pt, pipe.end_pt, fraction)
        for lane in pipe.lanes:
            position = _lerp(lane.start_pt, lane.end_pt, fraction)
            result.append(tuple(a - b for a, b in zip(position, center)))
        return tuple(result)

    def split(edge_id, source, target, kind, speed, pipe_id=None, ramp_id=None,
              offsets_start=(), offsets_end=(), ramp_direction="forward"):
        start, end = nodes[source].position, nodes[target].position
        length = math.dist(start, end)
        cut_values = [0., 1.]
        for axis, minimum, count in ((0, xmin, cols), (1, ymin, rows)):
            delta = end[axis] - start[axis]
            if abs(delta) > 1.e-12:
                for boundary in range(1, count):
                    value = (minimum + boundary * side - start[axis]) / delta
                    if 0. < value < 1.:
                        cut_values.append(value)
        cuts = []
        for value in sorted(cut_values):
            if not cuts or abs(value - cuts[-1]) > 1.e-12:
                cuts.append(value)
        lane_count = min(len(offsets_start), len(offsets_end)) or 1

        def interpolated(fraction):
            return tuple(_lerp(a, b, fraction) for a, b in zip(offsets_start, offsets_end))

        def ramp_metadata(name, low, high):
            if ramp_id is not None:
                ramp_fragments[name] = dict(ramp_id=ramp_id, direction=ramp_direction,
                                            fraction_start=low, fraction_end=high,
                                            offsets_at_entry=offsets_start, offsets_at_exit=offsets_end)

        def anchor(index, rid, fraction):
            name = "{}:C{}:R{}".format(edge_id, index, rid)
            distance = None
            if pipe_id is not None:
                distance = nodes[source].distance_m + (nodes[target].distance_m - nodes[source].distance_m) * fraction
            nodes[name] = RoadNode(name, rid, _lerp(start, end, fraction), pipe_id, distance)
            return name

        def boundary(index, left, right, fraction):
            name = "{}:B{}".format(edge_id, index)
            position = _lerp(start, end, fraction)
            shift = interpolated(fraction)
            edges[name] = RoadEdge(name, left, right, nodes[left].region_id, "BOUNDARY", 0.,
                                   float(speed), (position, position), pipe_id, ramp_id,
                                   lane_count, shift, shift)
            ramp_metadata(name, fraction, fraction)

        current = source
        for index, (low, high) in enumerate(zip(cuts, cuts[1:])):
            rid = owner(_lerp(start, end, (low + high) / 2.))
            if nodes[current].region_id != rid:
                next_node = anchor(index, rid, low)
                boundary(index, current, next_node, low)
                current = next_node
            final = index == len(cuts) - 2
            next_node = target if final and nodes[target].region_id == rid else anchor(index + 1, rid, high)
            name = "{}:F{}".format(edge_id, index)
            edges[name] = RoadEdge(name, current, next_node, rid, kind, length * (high - low),
                                   float(speed), (_lerp(start, end, low), _lerp(start, end, high)),
                                   pipe_id, ramp_id, lane_count, interpolated(low), interpolated(high))
            ramp_metadata(name, low, high)
            current = next_node
        if current != target:
            boundary(len(cuts), current, target, 1.)

    for pid, pipe in pipes.items():
        ordered = sorted(set(pipe_nodes[pid]))
        for index, ((distance_a, source), (distance_b, target)) in enumerate(zip(ordered, ordered[1:])):
            start_offsets, end_offsets = offsets(pipe, distance_a), offsets(pipe, distance_b)
            split("P{}:E{}".format(pid, index), source, target, "FLY", pipe.max_speed, pid,
                  offsets_start=start_offsets, offsets_end=end_offsets)
            if distance_a == distance_b:
                split("P{}:E{}:ALIAS".format(pid, index), target, source, "FLY", pipe.max_speed, pid,
                      offsets_start=end_offsets, offsets_end=start_offsets)
    for ramp in env.ramps:
        source = "P{}_R{}".format(ramp.pipe_a_id, ramp.ramp_id)
        target = "P{}_R{}".format(ramp.pipe_b_id, ramp.ramp_id)
        start_offsets = offsets(pipes[ramp.pipe_a_id], nodes[source].distance_m)
        end_offsets = offsets(pipes[ramp.pipe_b_id], nodes[target].distance_m)
        speed = min(float(pipes[ramp.pipe_a_id].max_speed), float(pipes[ramp.pipe_b_id].max_speed))
        split("R{}".format(ramp.ramp_id), source, target, "RAMP", speed, ramp_id=int(ramp.ramp_id),
              offsets_start=start_offsets, offsets_end=end_offsets)
        if ramp.is_bidirectional:
            split("R{}:REVERSE".format(ramp.ramp_id), target, source, "RAMP", speed,
                  ramp_id=int(ramp.ramp_id), offsets_start=end_offsets, offsets_end=start_offsets,
                  ramp_direction="reverse")
    for node_id, spec in goal_specs.items():
        pipe = pipes[spec["pipe_id"]]
        distance = spec["distance_m"]
        first_distance, through_node = goal_anchors[node_id]
        terminal_offset = offsets(pipe, distance)[spec["lane_id"]]
        terminal_offsets = (terminal_offset,) * int(pipe.num_lanes)
        # Include ramp entries within the approach window so that joining a
        # pipe shortly before its goal never forces an unnecessary full loop.
        sources = sorted(set((value, name) for value, name in pipe_nodes[pipe.pipe_id]
                             if first_distance <= value <= distance))
        for index, (value, source) in enumerate(sources):
            split("GOAL:{}:E{}".format(node_id, index), source, node_id,
                  "GOAL_APPROACH", pipe.max_speed, pipe.pipe_id,
                  offsets_start=offsets(pipe, value), offsets_end=terminal_offsets)
        # Pickups can continue towards their final goal after reaching this
        # terminal. This one-way zero-distance re-entry cannot be used to
        # bypass the lane approach when navigating *to* the terminal.
        normal_offsets = offsets(pipe, distance)
        split("GOAL:{}:DEPART".format(node_id), node_id, through_node,
              "FLY", pipe.max_speed, pipe.pipe_id,
              offsets_start=normal_offsets, offsets_end=normal_offsets)
    for index, (source_pipe, target_pipe, _external, kind) in enumerate(getattr(env, "continuation_edges", ())):
        if kind == "CONTINUE":
            split("CONTINUE{}".format(index), "P{}_END".format(source_pipe), "P{}_START".format(target_pipe),
                  "CONTINUE", min(pipes[source_pipe].max_speed, pipes[target_pipe].max_speed))
    metadata = dict(original_pipe_count=len(pipes),
                    pipe_endpoints={pid: {"start": "P{}_START".format(pid), "end": "P{}_END".format(pid)} for pid in pipes},
                    pipes={pid: dict(height=float(p.height), length_m=float(p.length), lane_count=int(p.num_lanes),
                                     speed_limit_mps=float(p.max_speed)) for pid, p in pipes.items()},
                    goal_specs=goal_specs, ramp_fragments=ramp_fragments,
                    goal_approach_max_length_m=200.,
                    region_rows=rows, region_cols=cols, region_side_length=side,
                    bounds=(xmin, xmax, ymin, ymax))
    return PartitionGraph(nodes, edges, regions, metadata, int(getattr(env, "topology_version", 0)),
                          {int(r.ramp_id): bool(r.is_active) for r in env.ramps})


class PartitionMaps:
    """Authoritative map service. Routers receive its values, never this object."""

    def __init__(self, graph):
        self.graph = graph
        self.cost_revision = 0
        self.edge_penalties = FrozenDict()
        self._cache_version = graph.version
        self._region_cache = {}
        self._reverse_region_cache = {}
        self._summary = None
        self._destination_cache = {}
        self._edges_by_region = defaultdict(list)
        self._nodes_by_region = defaultdict(dict)
        self._portals_by_region = defaultdict(set)
        self._boundary_edges = []
        for node in graph.nodes.values():
            self._nodes_by_region[node.region_id][node.node_id] = node
        for edge in graph.edges.values():
            self._edges_by_region[edge.region_id].append(edge)
            if edge.kind == "BOUNDARY":
                self._boundary_edges.append(edge)
                for name in (edge.source, edge.target):
                    self._portals_by_region[graph.nodes[name].region_id].add(name)

    def _active(self, edge):
        return edge.ramp_id is None or self.graph.ramp_states.get(edge.ramp_id, True)

    def with_edge_penalties(self, penalties, revision):
        """A routing snapshot shares road geometry and owns its derived costs."""
        if not isinstance(revision, int) or revision <= 0:
            raise ValueError("cost revision must be a positive integer")
        for edge_id, penalty in penalties.items():
            if edge_id not in self.graph.edges or not math.isfinite(penalty) or penalty < 0.:
                raise ValueError("unknown edge or invalid nonnegative penalty: {}".format(edge_id))
        snapshot = copy(self)
        snapshot.cost_revision = revision
        snapshot.edge_penalties = FrozenDict(penalties)
        snapshot._region_cache = {}
        snapshot._reverse_region_cache = {}
        snapshot._destination_cache = {}
        snapshot._summary = None
        return snapshot

    def _routing_cost(self, edge):
        return edge_cost(edge) + self.edge_penalties.get(edge.edge_id, 0.)

    def _check_version(self):
        if self._cache_version != self.graph.version:
            self._region_cache.clear()
            self._reverse_region_cache.clear()
            self._destination_cache.clear()
            self._summary = None
            self._cache_version = self.graph.version

    def region(self, region_id):
        self._check_version()
        region_id = int(region_id)
        if region_id not in self.graph.regions:
            raise KeyError(region_id)
        if region_id in self._region_cache:
            return self._region_cache[region_id]
        nodes = dict(self._nodes_by_region[region_id])
        edges, adjacency = {}, defaultdict(list)
        for edge in self._edges_by_region[region_id]:
            if not self._active(edge):
                continue
            edges[edge.edge_id] = edge
            nodes[edge.target] = self.graph.nodes[edge.target]
            nodes[edge.source] = self.graph.nodes[edge.source]
            adjacency[edge.source].append((edge.target, self._routing_cost(edge), edge.edge_id))
        view = RegionMap(region_id, FrozenDict(nodes), FrozenDict(edges),
                         FrozenDict({n: tuple(a) for n, a in adjacency.items()}),
                         self.graph.version, self.cost_revision)
        self._region_cache[region_id] = view
        return view

    @staticmethod
    def _local_adjacency(region):
        return {source: tuple((target, cost, eid) for target, cost, eid in arcs
                              if region.edges[eid].kind != "BOUNDARY")
                for source, arcs in region.adjacency.items()}

    def _reverse_adjacency(self, region_id):
        region_id = int(region_id)
        if region_id in self._reverse_region_cache:
            return self._reverse_region_cache[region_id]
        region = self.region(region_id)
        reverse = defaultdict(list)
        for source, arcs in self._local_adjacency(region).items():
            for target, cost, eid in arcs:
                reverse[target].append((source, cost, eid))
        frozen = {k: tuple(v) for k, v in reverse.items()}
        self._reverse_region_cache[region_id] = frozen
        return frozen

    def summary(self):
        self._check_version()
        if self._summary is not None:
            return self._summary
        adjacency, node_regions = defaultdict(list), {}
        for rid in self.graph.regions:
            portals = self._portals_by_region[rid]
            local = self._local_adjacency(self.region(rid))
            for source in sorted(portals):
                node_regions[source] = rid
                distances, _ = shortest_paths(local, source)
                adjacency[source].extend(PortalArc(target, distances[target]) for target in sorted(portals)
                                         if target != source and target in distances)
        for edge in self._boundary_edges:
            if self._active(edge):
                adjacency[edge.source].append(PortalArc(edge.target, self._routing_cost(edge), "BOUNDARY", edge.edge_id))
        self._summary = PortalSummary(FrozenDict(node_regions),
                                      FrozenDict({n: tuple(a) for n, a in adjacency.items()}),
                                      self.graph.version, self.cost_revision)
        return self._summary

    def destination(self, node_id):
        self._check_version()
        node_id = str(node_id)
        if node_id in self._destination_cache:
            return self._destination_cache[node_id]
        node = self.graph.nodes[node_id]
        region = self.region(node.region_id)
        reverse = self._reverse_adjacency(node.region_id)
        distances, _ = shortest_paths(reverse, node_id)
        ingress = {p: distances[p] for p in self._portals_by_region[node.region_id] if p in distances}
        spec = self.graph.metadata.get("goal_specs", {}).get(node_id, {})
        lane_id = int(spec.get("lane_id", 0))
        position = node.position
        if node.pipe_id is not None:
            for edge in region.edges.values():
                offsets = edge.offsets_start if edge.source == node_id else edge.offsets_end if edge.target == node_id else ()
                if len(offsets) > lane_id:
                    position = tuple(p + o for p, o in zip(position, offsets[lane_id]))
                    break
        result = Destination(node_id, node.region_id, FrozenDict(ingress), self.graph.version,
                             node.pipe_id, lane_id, float(node.distance_m or 0.), position, self.cost_revision)
        self._destination_cache[node_id] = result
        return result

    def set_ramp_state(self, ramp_id, enabled):
        ramp_id, enabled = int(ramp_id), bool(enabled)
        if ramp_id not in self.graph.ramp_states:
            raise KeyError(ramp_id)
        if self.graph.ramp_states[ramp_id] != enabled:
            self.graph.ramp_states[ramp_id] = enabled
            self.graph.version += 1
            self._check_version()
        return self.graph.version
