"""Exact regional Dijkstra with one detailed region per aircraft.

Reverse portal potentials are shared per immutable summary/version/goal.
They contain costs and portal successors, never remote road geometry.
"""
import heapq
import weakref
from collections import OrderedDict, defaultdict
from sys import getsizeof
from threading import RLock

from uav_sim.airspace.partition_contracts import RegionPlan
from uav_sim.airspace.partition_graph import FrozenDict, shortest_paths


_POTENTIAL_CACHE = OrderedDict()
_REVERSE_CACHE = OrderedDict()
_LOCAL_ADJACENCY_CACHE = {}
_CACHE_LOCK = RLock()
_MAX_GOALS = 16384
_MAX_POTENTIAL_BYTES = 32 * 1024 * 1024
_MAX_SUMMARIES = 64


def _local_adjacency(region_map):
    """Share a local index without keeping old detailed maps alive."""
    key = id(region_map)
    with _CACHE_LOCK:
        cached = _LOCAL_ADJACENCY_CACHE.get(key)
        if cached is not None and cached[0]() is region_map:
            return cached[1]
        adjacency = FrozenDict({source: tuple((target, cost, eid) for target, cost, eid in arcs
                                              if region_map.edges[eid].kind != "BOUNDARY")
                                for source, arcs in region_map.adjacency.items()})
        def discard(reference):
            with _CACHE_LOCK:
                entry = _LOCAL_ADJACENCY_CACHE.get(key)
                if entry is not None and entry[0] is reference:
                    _LOCAL_ADJACENCY_CACHE.pop(key, None)
        _LOCAL_ADJACENCY_CACHE[key] = (weakref.ref(region_map, discard), adjacency)
        return adjacency


def _summary_reference(summary, cache, key):
    """Remove the cached value when its owning summary is released."""
    def discard(reference):
        with _CACHE_LOCK:
            entry = cache.get(key)
            if entry is not None and entry[0] is reference:
                cache.pop(key, None)

    return weakref.ref(summary, discard)

def _potential_size(entry):
    """Estimate retained table bytes; node IDs and arcs are shared with the summary."""
    _, distances, successors = entry
    return getsizeof(distances) + getsizeof(successors) + len(distances) * getsizeof(0.0)

def _goal_potential(summary, destination):
    """A bounded process-local cache shared by all current-region planners."""
    key = (id(summary), summary.version, destination.node_id, destination.region_id)
    with _CACHE_LOCK:
        if key in _POTENTIAL_CACHE:
            _POTENTIAL_CACHE.move_to_end(key)
            return _POTENTIAL_CACHE[key][1:]
        summary_key = (id(summary), summary.version)
        if summary_key not in _REVERSE_CACHE:
            reverse = defaultdict(list)
            for source, arcs in summary.adjacency.items():
                for arc in arcs:
                    reverse[arc.target].append((source, arc))
            _REVERSE_CACHE[summary_key] = (
                _summary_reference(summary, _REVERSE_CACHE, summary_key),
                FrozenDict({n: tuple(a) for n, a in reverse.items()}))
            if len(_REVERSE_CACHE) > _MAX_SUMMARIES:
                _REVERSE_CACHE.popitem(last=False)
        else:
            _REVERSE_CACHE.move_to_end(summary_key)
        reverse = _REVERSE_CACHE[summary_key][1]
        distances, successors = dict(destination.ingress_costs), {}
        queue = [(cost, node) for node, cost in distances.items()]
        heapq.heapify(queue)
        while queue:
            cost, target = heapq.heappop(queue)
            if cost != distances[target]:
                continue
            for source, arc in reverse.get(target, ()):
                candidate = cost + arc.cost
                if candidate < distances.get(source, float("inf")):
                    distances[source] = candidate
                    successors[source] = arc
                    heapq.heappush(queue, (candidate, source))
        value = (_summary_reference(summary, _POTENTIAL_CACHE, key),
                 FrozenDict(distances), FrozenDict(successors))
        _POTENTIAL_CACHE[key] = value
        retained_bytes = sum(_potential_size(entry) for entry in _POTENTIAL_CACHE.values())
        while (len(_POTENTIAL_CACHE) > _MAX_GOALS
               or retained_bytes > _MAX_POTENTIAL_BYTES):
            _, removed = _POTENTIAL_CACHE.popitem(last=False)
            retained_bytes -= _potential_size(removed)
        return value[1:]


class RegionalDijkstra:
    def __init__(self, summary, region_map):
        self.summary = summary
        self.replace_region(region_map)

    def update_summary(self, summary):
        self.summary = summary

    def replace_region(self, region_map):
        self.region_map = region_map
        self._adjacency = _local_adjacency(region_map)

    def plan(self, start_node_id, destination):
        local, summary = self.region_map, self.summary
        version = summary.version
        if (local.version, local.cost_revision) != (version, summary.cost_revision) or (
            destination.version, destination.cost_revision
        ) != (version, summary.cost_revision):
            return RegionPlan("STALE_MAP", version=version, reason="map and destination versions differ")
        if start_node_id not in local.nodes or local.nodes[start_node_id].region_id != local.region_id:
            return RegionPlan("NO_ROUTE", version=version, reason="start is outside the detailed region")
        distances, previous = shortest_paths(self._adjacency, start_node_id)
        best = distances.get(destination.node_id, float("inf")) if destination.region_id == local.region_id else float("inf")
        potentials, successors = _goal_potential(summary, destination)
        first_portal = None
        for name, local_cost in distances.items():
            candidate = local_cost + potentials.get(name, float("inf"))
            if candidate < best:
                best, first_portal = candidate, name
        if best == float("inf"):
            return RegionPlan("NO_ROUTE", version=version, reason="no directed route to destination")

        region_path, first_boundary = [local.region_id], None
        current = first_portal
        while current in successors:
            arc = successors[current]
            rid = summary.node_regions[arc.target]
            if rid != region_path[-1]:
                region_path.append(rid)
            if first_boundary is None and arc.kind == "BOUNDARY":
                first_boundary = (current, arc.edge_id)
            current = arc.target
        target = first_boundary[0] if first_boundary else destination.node_id
        if target not in distances:
            return RegionPlan("NO_ROUTE", version=version, reason="summary has no compatible local prefix")
        edge_ids, current = [], target
        while current != start_node_id:
            source, edge_id = previous[current]
            edge_ids.append(edge_id)
            current = source
        edge_ids.reverse()
        if first_boundary:
            if first_boundary[1] not in local.edges:
                return RegionPlan("STALE_MAP", version=version, reason="boundary is absent from detailed map")
            edge_ids.append(first_boundary[1])
        return RegionPlan("OK", tuple(edge_ids), tuple(region_path), best, version)
