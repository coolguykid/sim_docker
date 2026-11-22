"""Regional matrix helpers for hierarchical 6x6 airspace routing."""
from dataclasses import dataclass
from collections import defaultdict, OrderedDict
import heapq

from uav_sim.airspace.ramp_cost import speed_bucket, switch_edge_cost


@dataclass(frozen=True)
class CrossRegionPortal:
    ramp_id: int
    region_a_id: int
    region_b_id: int
    pipe_a_id: int
    pipe_b_id: int
    dist_a: float
    dist_b: float
    layer_height: int
    local_layer_index: int
    side_a: str
    side_b: str
    time_penalty: float
    node_a: str
    node_b: str
    is_bidirectional: bool = True


@dataclass(frozen=True)
class LocalPathResult:
    raw_path: list
    cost: float
    node_info_patch: dict


@dataclass(frozen=True)
class LocalPathTarget:
    key: object
    pipe_id: int
    pos: float
    node_name: str


class RegionAirspace:
    """Per-region routing view using global pipe and ramp ids."""

    def __init__(self, region_id, pipelines, ramps, pipe_nodes, node_info,
                 continuation_edges=None):
        self.region_id = int(region_id)
        self.pipelines = pipelines
        self.ramps = ramps
        self.ramp_by_id = {int(ramp.ramp_id): ramp for ramp in ramps}
        self.pipe_ids = [
            int(pipe.pipe_id)
            for pipe in pipelines
            if int(getattr(pipe, "region_id", 0)) == self.region_id
        ]
        self.pipe_id_set = set(self.pipe_ids)
        self.pipe_nodes = {
            pid: list(pipe_nodes.get(pid, []))
            for pid in self.pipe_ids
        }
        self.node_info = {
            node: dict(info)
            for node, info in node_info.items()
            if int(info.get("pipe_id", -1)) in self.pipe_id_set
        }
        self.continuation_edges = list(continuation_edges or ())
        self.graph = self._build_local_graph()
        self._path_cache_capacity = 4096
        self._path_cache = OrderedDict()

    def _build_local_graph(self):
        graph = {}
        for pipe_id in self.pipe_ids:
            pipe = self.pipelines[pipe_id]
            nodes = sorted(self.pipe_nodes.get(pipe_id, []), key=lambda x: x[1])
            self.pipe_nodes[pipe_id] = nodes
            for i in range(len(nodes) - 1):
                curr_name, curr_dist, _ = nodes[i]
                next_name, next_dist, _ = nodes[i + 1]
                graph.setdefault(curr_name, [])
                graph.setdefault(next_name, [])
                graph[curr_name].append((
                    next_name,
                    (float(next_dist) - float(curr_dist)) / float(pipe.max_speed),
                    "FLY",
                    None,
                ))

        for source_pipe_id, target_pipe_id, _external_node_id, edge_type in self.continuation_edges:
            source_pipe_id = int(source_pipe_id)
            target_pipe_id = int(target_pipe_id)
            if edge_type != "CONTINUE":
                continue
            if (
                    source_pipe_id not in self.pipe_id_set
                    or target_pipe_id not in self.pipe_id_set):
                continue
            source_node = f"P{source_pipe_id}_END"
            target_node = f"P{target_pipe_id}_START"
            graph.setdefault(source_node, [])
            graph.setdefault(target_node, [])
            graph[source_node].append((
                target_node, 0.0, "CONTINUE", None
            ))

        for ramp in self.ramps:
            pipe_a_id = int(ramp.pipe_a_id)
            pipe_b_id = int(ramp.pipe_b_id)
            if pipe_a_id not in self.pipe_id_set or pipe_b_id not in self.pipe_id_set:
                continue
            node_a = f"P{pipe_a_id}_R{int(ramp.ramp_id)}"
            node_b = f"P{pipe_b_id}_R{int(ramp.ramp_id)}"
            graph.setdefault(node_a, [])
            graph.setdefault(node_b, [])
            graph[node_a].append(
                (node_b, float(ramp.time_penalty), "SWITCH", int(ramp.ramp_id))
            )
            if bool(getattr(ramp, "is_bidirectional", False)):
                graph[node_b].append((node_a, float(ramp.time_penalty), "SWITCH", int(ramp.ramp_id)))
        return graph

    def find_local_path(self, start_pipe_id, start_pos, dest_pipe_id, dest_pos,
                        ramp_states=None, traffic_multiplier=None,
                        dest_node_name=None, ramp_state_version=None,
                        traffic_cost_version=None, uav_speed=None):
        start_pipe_id = int(start_pipe_id)
        dest_pipe_id = int(dest_pipe_id)
        start_pos = float(start_pos)
        dest_pos = float(dest_pos)
        if start_pipe_id not in self.pipe_id_set or dest_pipe_id not in self.pipe_id_set:
            return LocalPathResult([], float("inf"), {})

        if ramp_state_version is None:
            ramp_state_version = self._ramp_state_cache_token(ramp_states)
        if traffic_cost_version is None:
            traffic_cost_version = self._traffic_cache_token(traffic_multiplier)
        cache_key = (
            start_pipe_id,
            round(start_pos, 1),
            dest_pipe_id,
            round(dest_pos, 1),
            dest_node_name,
            speed_bucket(uav_speed),
            int(ramp_state_version),
            int(traffic_cost_version),
        )
        if cache_key in self._path_cache:
            self._path_cache.move_to_end(cache_key)
            return self._path_cache[cache_key]

        end_node = dest_node_name or "V_END"
        pq = [(0.0, "V_START")]
        distances = {"V_START": 0.0}
        previous = {}
        while pq:
            cost, curr_node = heapq.heappop(pq)
            if cost != distances.get(curr_node):
                continue
            if curr_node == end_node:
                path = self._reconstruct_path(previous, curr_node)
                result = LocalPathResult(path, cost, self.get_node_info_patch(path))
                self._cache_path(cache_key, result)
                return result
            for neighbor, weight in self._edges_for_node(
                    curr_node, start_pipe_id, start_pos, dest_pipe_id, dest_pos,
                    dest_node_name, ramp_states, traffic_multiplier,
                    uav_speed):
                new_cost = cost + weight
                if new_cost < distances.get(neighbor, float("inf")):
                    distances[neighbor] = new_cost
                    previous[neighbor] = curr_node
                    heapq.heappush(pq, (new_cost, neighbor))

        result = LocalPathResult([], float("inf"), {})
        self._cache_path(cache_key, result)
        return result

    def find_paths_to_targets(self, start_pipe_id, start_pos, targets,
                              ramp_states=None, traffic_multiplier=None,
                              ramp_state_version=None, traffic_cost_version=None,
                              uav_speed=None):
        targets = list(targets or [])
        if not targets:
            return {}
        start_pipe_id = int(start_pipe_id)
        start_pos = float(start_pos)
        if start_pipe_id not in self.pipe_id_set:
            return {}
        if ramp_state_version is None:
            ramp_state_version = self._ramp_state_cache_token(ramp_states)
        if traffic_cost_version is None:
            traffic_cost_version = self._traffic_cache_token(traffic_multiplier)

        valid_targets = {
            target.node_name: target
            for target in targets
            if int(target.pipe_id) in self.pipe_id_set
            and target.node_name in self.node_info
        }
        if not valid_targets:
            return {}

        pq = [(0.0, "V_START")]
        distances = {"V_START": 0.0}
        previous = {}
        results = {}
        while pq and len(results) < len(valid_targets):
            cost, curr_node = heapq.heappop(pq)
            if cost != distances.get(curr_node):
                continue
            target = valid_targets.get(curr_node)
            if target is not None and target.key not in results:
                path = self._reconstruct_path(previous, curr_node)
                result = LocalPathResult(path, cost, self.get_node_info_patch(path))
                results[target.key] = result
                cache_key = (
                    start_pipe_id,
                    round(start_pos, 1),
                    int(target.pipe_id),
                    round(float(target.pos), 1),
                    target.node_name,
                    speed_bucket(uav_speed),
                    int(ramp_state_version),
                    int(traffic_cost_version),
                )
                self._cache_path(cache_key, result)
            for neighbor, weight in self._edges_for_node(
                    curr_node, start_pipe_id, start_pos,
                    -1, 0.0, "__MULTI_TARGET__",
                    ramp_states, traffic_multiplier, uav_speed):
                new_cost = cost + weight
                if new_cost < distances.get(neighbor, float("inf")):
                    distances[neighbor] = new_cost
                    previous[neighbor] = curr_node
                    heapq.heappush(pq, (new_cost, neighbor))
        return results

    @staticmethod
    def _ramp_state_cache_token(ramp_states):
        if not ramp_states:
            return 0
        return hash(tuple(sorted((int(k), bool(v)) for k, v in ramp_states.items())))

    @staticmethod
    def _traffic_cache_token(traffic_multiplier):
        if not traffic_multiplier:
            return 0
        return hash(tuple(sorted(
            (int(key), round(float(value), 8))
            for key, value in traffic_multiplier.items()
        )))

    @staticmethod
    def _reconstruct_path(previous, end_node):
        path = [end_node]
        while path[-1] != "V_START":
            parent = previous.get(path[-1])
            if parent is None:
                return []
            path.append(parent)
        path.reverse()
        return path

    def _cache_path(self, cache_key, result):
        self._path_cache[cache_key] = result
        self._path_cache.move_to_end(cache_key)
        if len(self._path_cache) > self._path_cache_capacity:
            self._path_cache.popitem(last=False)

    def _edges_for_node(self, curr_node, start_pipe_id, start_pos,
                        dest_pipe_id, dest_pos, dest_node_name,
                        ramp_states, traffic_multiplier, uav_speed=None):
        if curr_node == "V_START":
            return self._start_edges(start_pipe_id, start_pos, dest_pipe_id,
                                     dest_pos, dest_node_name, traffic_multiplier)

        if curr_node not in self.node_info:
            return []
        curr_info = self.node_info[curr_node]
        curr_pipe_id = int(curr_info["pipe_id"])
        curr_dist = float(curr_info["dist"])
        edges = []

        if dest_node_name is None and curr_pipe_id == dest_pipe_id and curr_dist <= dest_pos:
            pipe = self.pipelines[dest_pipe_id]
            if dest_pos >= curr_dist:
                edges.append((
                    "V_END",
                    self._fly_weight(dest_pipe_id, dest_pos - curr_dist, traffic_multiplier),
                ))

        for neighbor, weight, edge_type, ramp_id in self.graph.get(curr_node, []):
            if edge_type == "SWITCH" and ramp_id is not None:
                if ramp_states is not None and not ramp_states.get(int(ramp_id), True):
                    continue
            if neighbor in self.node_info:
                n_info = self.node_info[neighbor]
                if dest_node_name is None and int(n_info["pipe_id"]) == dest_pipe_id and float(n_info["dist"]) > dest_pos:
                    continue
            actual_weight = float(weight)
            if edge_type == "SWITCH" and ramp_id is not None:
                actual_weight = switch_edge_cost(self, ramp_id, uav_speed)
            elif edge_type == "FLY":
                actual_weight *= float((traffic_multiplier or {}).get(curr_pipe_id, 1.0))
            edges.append((neighbor, actual_weight))
        return edges

    def _start_edges(self, start_pipe_id, start_pos, dest_pipe_id,
                     dest_pos, dest_node_name, traffic_multiplier):
        edges = []
        if dest_node_name is None and start_pipe_id == dest_pipe_id and start_pos <= dest_pos:
            edges.append(("V_END", self._fly_weight(start_pipe_id, dest_pos - start_pos, traffic_multiplier)))
        for node_name, node_dist, _node_type in self.pipe_nodes.get(start_pipe_id, []):
            node_dist = float(node_dist)
            if node_dist < start_pos:
                continue
            if dest_node_name is None and start_pipe_id == dest_pipe_id and node_dist > dest_pos:
                continue
            edges.append((
                node_name,
                self._fly_weight(start_pipe_id, node_dist - start_pos, traffic_multiplier),
            ))
        return edges

    def _fly_weight(self, pipe_id, distance, traffic_multiplier):
        pipe = self.pipelines[int(pipe_id)]
        multiplier = 1.0
        if traffic_multiplier:
            multiplier = float(traffic_multiplier.get(int(pipe_id), 1.0))
        return max(0.0, float(distance)) * multiplier / float(pipe.max_speed)

    def get_node_info_patch(self, raw_path):
        return {
            node: dict(self.node_info[node])
            for node in raw_path
            if node in self.node_info
        }


class RegionalAirspaceMatrix:
    """A 36-region view built from the existing regional Airspace snapshot."""

    def __init__(self, snapshot):
        self.snapshot = snapshot
        self.pipelines = list(snapshot["pipelines"])
        self.ramps = list(snapshot["ramps"])
        self.ramp_by_id = {int(ramp.ramp_id): ramp for ramp in self.ramps}
        self.regions = list(snapshot.get("regions", []))
        self.region_rows = int(snapshot.get("region_rows", 1))
        self.region_cols = int(snapshot.get("region_cols", 1))
        self.pipe_to_region = {
            int(pipe.pipe_id): int(getattr(pipe, "region_id", 0))
            for pipe in self.pipelines
        }
        self.pipe_nodes = {
            int(pid): list(nodes)
            for pid, nodes in snapshot.get("pipe_nodes", {}).items()
        }
        self.node_info = {
            node: dict(info)
            for node, info in snapshot.get("node_info", {}).items()
        }
        self.continuation_edges = list(snapshot.get("continuation_edges", ()))
        self.topology_source = snapshot.get("topology_source", "generated")
        self.topology_version = int(snapshot.get("topology_version", 0))
        self.external_id_maps = snapshot.get("external_id_maps")
        self.region_neighbors = self._build_region_neighbors()
        self.cross_region_portals = self._build_cross_region_portals()
        self.portal_index = self._build_portal_index()
        self.region_airspaces = self._build_region_airspaces()

    @classmethod
    def from_snapshot(cls, snapshot):
        return cls(snapshot)

    def region_for_pipe(self, pipe_id):
        return self.pipe_to_region[int(pipe_id)]

    def neighbor_region_ids(self, region_id):
        return list(self.region_neighbors.get(int(region_id), []))

    def cross_portals_between(self, region_a_id, region_b_id):
        return list(self.portal_index.get((int(region_a_id), int(region_b_id)), []))

    def get_region(self, region_id):
        return self.region_airspaces[int(region_id)]

    def _build_region_neighbors(self):
        neighbors = {int(r["region_id"]): set() for r in self.regions}
        for region in self.regions:
            rid = int(region["region_id"])
            row = int(region["row"])
            col = int(region["col"])
            if col > 0:
                neighbors[rid].add(row * self.region_cols + col - 1)
            if col + 1 < self.region_cols:
                neighbors[rid].add(row * self.region_cols + col + 1)
            if row > 0:
                neighbors[rid].add((row - 1) * self.region_cols + col)
            if row + 1 < self.region_rows:
                neighbors[rid].add((row + 1) * self.region_cols + col)
        return {rid: sorted(values) for rid, values in neighbors.items()}

    def _build_cross_region_portals(self):
        portals = []
        for ramp in self.ramps:
            is_cross = getattr(ramp, "is_cross_region", False) or (
                getattr(ramp, "ramp_type", "") == "cross_region_intra_layer")
            if not is_cross:
                continue
            pipe_a = self.pipelines[int(ramp.pipe_a_id)]
            pipe_b = self.pipelines[int(ramp.pipe_b_id)]
            side_a, side_b = self._parse_boundary_side(getattr(ramp, "boundary_side", None))
            portals.append(CrossRegionPortal(
                ramp_id=int(ramp.ramp_id),
                region_a_id=int(getattr(pipe_a, "region_id", 0)),
                region_b_id=int(getattr(pipe_b, "region_id", 0)),
                pipe_a_id=int(pipe_a.pipe_id),
                pipe_b_id=int(pipe_b.pipe_id),
                dist_a=float(ramp.dist_from_start_a),
                dist_b=float(ramp.dist_from_start_b),
                layer_height=int(pipe_a.height),
                local_layer_index=int(getattr(pipe_a, "local_layer_index", 0)),
                side_a=side_a,
                side_b=side_b,
                time_penalty=float(ramp.time_penalty),
                node_a=f"P{int(pipe_a.pipe_id)}_R{int(ramp.ramp_id)}",
                node_b=f"P{int(pipe_b.pipe_id)}_R{int(ramp.ramp_id)}",
                is_bidirectional=bool(getattr(ramp, "is_bidirectional", False)),
            ))
        return portals

    @staticmethod
    def _parse_boundary_side(boundary_side):
        raw = str(boundary_side or "")
        if "_to_" not in raw:
            return "", ""
        left, right = raw.split("_to_", 1)
        return left, right

    def _build_portal_index(self):
        index = defaultdict(list)
        for portal in self.cross_region_portals:
            index[(portal.region_a_id, portal.region_b_id)].append(portal)
            if portal.is_bidirectional:
                index[(portal.region_b_id, portal.region_a_id)].append(portal)
        for key in list(index):
            index[key].sort(key=lambda p: (p.layer_height, p.local_layer_index, p.ramp_id))
        return dict(index)

    def _build_region_airspaces(self):
        return {
            int(region["region_id"]): RegionAirspace(
                int(region["region_id"]),
                self.pipelines,
                self.ramps,
                self.pipe_nodes,
                self.node_info,
                self.continuation_edges,
            )
            for region in self.regions
        }
