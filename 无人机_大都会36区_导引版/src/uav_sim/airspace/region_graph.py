"""Region-level graph for coarse routing across a 6x6 airspace matrix."""
from dataclasses import dataclass
import heapq
from collections import defaultdict


@dataclass(frozen=True)
class RegionEdge:
    region_a_id: int
    region_b_id: int
    weight: float


@dataclass(frozen=True)
class RegionPathPlan:
    region_ids: list
    estimated_cost: float


class RegionGraph:
    """Small 36-node graph used before local per-region Dijkstra."""

    def __init__(self, regions, cross_portals):
        self.regions = {int(r["region_id"]): dict(r) for r in regions}
        self.cross_portals = list(cross_portals)
        portal_index = defaultdict(list)
        for portal in self.cross_portals:
            a = int(portal.region_a_id)
            b = int(portal.region_b_id)
            portal_index[(a, b)].append(portal)
            if bool(getattr(portal, "is_bidirectional", False)):
                portal_index[(b, a)].append(portal)
        self.portal_index = dict(portal_index)
        self.adjacency = self._build_adjacency()

    def _build_adjacency(self):
        edge_weights = {}
        for portal in self.cross_portals:
            a = int(portal.region_a_id)
            b = int(portal.region_b_id)
            weight = float(max(1.0, portal.time_penalty))
            directions = [(a, b)]
            if bool(getattr(portal, "is_bidirectional", False)):
                directions.append((b, a))
            for key in directions:
                edge_weights[key] = min(weight, edge_weights.get(key, weight))

        adjacency = {rid: [] for rid in self.regions}
        for (a, b), penalty in edge_weights.items():
            region_a = self.regions[a]
            region_b = self.regions[b]
            dx = float(region_a["center_x"]) - float(region_b["center_x"])
            dy = float(region_a["center_y"]) - float(region_b["center_y"])
            distance = (dx * dx + dy * dy) ** 0.5
            weight = distance / 60.0 + penalty
            adjacency[a].append(RegionEdge(a, b, weight))
        return adjacency

    def shortest_region_path(self, start_region_id, dest_region_id,
                             start_xy=None, dest_xy=None,
                             preferred_layer=None, ramp_states=None):
        start_region_id = int(start_region_id)
        dest_region_id = int(dest_region_id)
        if start_region_id == dest_region_id:
            return RegionPathPlan([start_region_id], 0.0)

        pq = [(0.0, start_region_id, [start_region_id])]
        best = {}
        while pq:
            cost, region_id, path = heapq.heappop(pq)
            if region_id in best and best[region_id] <= cost:
                continue
            best[region_id] = cost
            if region_id == dest_region_id:
                return RegionPathPlan(path, cost)
            for edge in self.adjacency.get(region_id, []):
                edge_weight = self._available_edge_weight(edge, ramp_states)
                if edge_weight is None:
                    continue
                if edge.region_b_id not in best:
                    heapq.heappush(
                        pq,
                        (cost + edge_weight, edge.region_b_id, path + [edge.region_b_id]))
        return RegionPathPlan([], float("inf"))

    def _available_edge_weight(self, edge, ramp_states):
        if ramp_states is None:
            return edge.weight
        key = (int(edge.region_a_id), int(edge.region_b_id))
        active_portals = [
            portal for portal in self.portal_index.get(key, ())
            if ramp_states.get(int(portal.ramp_id), True)
        ]
        if not active_portals:
            return None
        region_a = self.regions[int(edge.region_a_id)]
        region_b = self.regions[int(edge.region_b_id)]
        dx = float(region_a["center_x"]) - float(region_b["center_x"])
        dy = float(region_a["center_y"]) - float(region_b["center_y"])
        distance_cost = (dx * dx + dy * dy) ** 0.5 / 60.0
        penalty = min(float(max(1.0, portal.time_penalty)) for portal in active_portals)
        return distance_cost + penalty
