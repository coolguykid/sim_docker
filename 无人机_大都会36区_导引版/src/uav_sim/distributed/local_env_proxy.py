"""
local_env_proxy.py — V6.0 环境代理
在分布式模式下替代 UAV 对 Airspace 的直接引用。
实现与 Airspace 相同的接口，使 UAV 类无需任何修改。
"""
import numpy as np
import heapq
from uav_sim.airspace.ramp_cost import switch_edge_cost


class LocalEnvProxy:
    """UAV 视角的环境代理 — 替代直接引用 Airspace"""

    def __init__(self, env_snapshot, routing_mode="DIJKSTRA"):
        if routing_mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        self.pipelines = env_snapshot["pipelines"]
        self.ramps = env_snapshot["ramps"]
        self.ramp_by_id = {int(ramp.ramp_id): ramp for ramp in self.ramps}
        self.directed_ramps_by_pipe_pair = {}
        for ramp in self.ramps:
            directions = [(ramp.source_pipe_id, ramp.target_pipe_id)]
            if bool(getattr(ramp, "is_bidirectional", False)):
                directions.append((ramp.target_pipe_id, ramp.source_pipe_id))
            for source_pipe_id, target_pipe_id in directions:
                key = (int(source_pipe_id), int(target_pipe_id))
                self.directed_ramps_by_pipe_pair.setdefault(key, []).append(
                    int(ramp.ramp_id)
                )
        self.graph = env_snapshot["graph"]
        self.node_info = dict(env_snapshot["node_info"])
        self.pipe_nodes = dict(env_snapshot.get("pipe_nodes", {}))
        self.layer_configs = env_snapshot["layer_configs"]
        self.x_min = env_snapshot.get("x_min", 0.0)
        self.x_max = env_snapshot.get("x_max", 5000.0)
        self.y_min = env_snapshot.get("y_min", 0.0)
        self.y_max = env_snapshot.get("y_max", 5000.0)
        self.z_min = env_snapshot.get("z_min", 0.0)
        self.z_max = env_snapshot.get("z_max", 600.0)
        self.airspace_mode = env_snapshot.get("airspace_mode", "standard")
        self.xy_span = env_snapshot.get("xy_span", self.x_max - self.x_min)
        self.xy_diagonal = env_snapshot.get(
            "xy_diagonal",
            float(np.hypot(self.x_max - self.x_min, self.y_max - self.y_min)),
        )
        self.z_span = env_snapshot.get("z_span", max(1.0, self.z_max - self.z_min))
        self.base_station_pos = np.array(
            env_snapshot.get(
                "base_station_pos",
                [
                    self.x_min + self.xy_span / 2.0,
                    self.y_min + (self.y_max - self.y_min) / 2.0,
                    0.0,
                ],
            )
        )
        self.regions = list(env_snapshot.get("regions", []))
        self.region_rows = int(env_snapshot.get("region_rows", 1))
        self.region_cols = int(env_snapshot.get("region_cols", 1))
        self.region_side_length = float(
            env_snapshot.get("region_side_length", self.xy_span)
        )
        self.num_lanes_per_pipe = int(env_snapshot.get("num_lanes_per_pipe", 10))
        self.topology_source = env_snapshot.get("topology_source", "generated")
        self.topology_version = int(env_snapshot.get("topology_version", 0))
        self.external_id_maps = env_snapshot.get("external_id_maps")
        self.continuation_edges = list(env_snapshot.get("continuation_edges", ()))
        self.routing_mode = routing_mode
        self._cached_collisions_multiplier = {p.pipe_id: 1.0 for p in self.pipelines}
        self._cached_traffic_load = {p.pipe_id: 0 for p in self.pipelines}
        self.pipe_collisions_window = {p.pipe_id: 0 for p in self.pipelines}
        self.pipe_collisions_multiplier = self._cached_collisions_multiplier
        self.pipe_traffic_load = self._cached_traffic_load

    def refresh_shared_state(self, collisions_multiplier, traffic_load, ramp_states):
        self._cached_collisions_multiplier = collisions_multiplier or {}
        self.pipe_collisions_multiplier = self._cached_collisions_multiplier
        self._cached_traffic_load = traffic_load or {}
        self.pipe_traffic_load = self._cached_traffic_load
        if ramp_states:
            for r in self.ramps:
                if r.ramp_id in ramp_states:
                    r.is_active = ramp_states[r.ramp_id]

    def _get_edges_for_node(
        self,
        curr_node,
        start_pipe,
        start_pos,
        dest_pipe,
        dest_pos,
        dest_3d,
        uav_speed=None,
    ):
        edges = []
        if curr_node == "V_START":
            if start_pipe == dest_pipe and start_pos <= dest_pos:
                fly_time = (
                    (dest_pos - start_pos)
                    * self.pipe_collisions_multiplier.get(start_pipe, 1.0)
                    / self.pipelines[start_pipe].max_speed
                )
                edges.append(("V_END", fly_time, "FLY", start_pipe, dest_3d))
            for n_name, n_dist, _node_type in self.pipe_nodes.get(start_pipe, []):
                if n_dist >= start_pos:
                    if start_pipe == dest_pipe and n_dist > dest_pos:
                        continue
                    fly_time = (
                        (n_dist - start_pos)
                        * self.pipe_collisions_multiplier.get(start_pipe, 1.0)
                        / self.pipelines[start_pipe].max_speed
                    )
                    edges.append(
                        (
                            n_name,
                            fly_time,
                            "FLY",
                            start_pipe,
                            self.pipelines[start_pipe].start_pt
                            + self.pipelines[start_pipe].direction * n_dist,
                        )
                    )
        else:
            curr_info = self.node_info[curr_node]
            curr_pipe = curr_info["pipe_id"]
            if curr_pipe == dest_pipe and curr_info["dist"] <= dest_pos:
                fly_time = (
                    (dest_pos - curr_info["dist"])
                    * self.pipe_collisions_multiplier.get(dest_pipe, 1.0)
                    / self.pipelines[dest_pipe].max_speed
                )
                edges.append(("V_END", fly_time, "FLY", dest_pipe, dest_3d))
            for neighbor, weight, edge_type, ramp_id in self.graph.get(curr_node, []):
                if (
                    edge_type == "SWITCH"
                    and ramp_id is not None
                    and (not self.ramps[ramp_id].is_active)
                ):
                    continue
                if neighbor not in ["V_END", "V_START"]:
                    n_info = self.node_info[neighbor]
                    if n_info["pipe_id"] == dest_pipe and n_info["dist"] > dest_pos:
                        continue
                    target_pipe = n_info["pipe_id"]
                    n_coords = (
                        self.pipelines[target_pipe].start_pt
                        + self.pipelines[target_pipe].direction * n_info["dist"]
                    )
                else:
                    n_coords, target_pipe = (dest_3d, dest_pipe)
                if edge_type == "SWITCH" and ramp_id is not None:
                    actual_weight = switch_edge_cost(self, ramp_id, uav_speed)
                else:
                    actual_weight = weight * self.pipe_collisions_multiplier.get(
                        curr_pipe, 1.0
                    )
                edges.append(
                    (neighbor, actual_weight, edge_type, target_pipe, n_coords)
                )
        return edges

    def find_shortest_path(
        self, start_pipe_id, start_pos, dest_pipe_id, dest_pos, uav_speed=None
    ):
        start_node = "V_START"
        end_node = "V_END"
        dest_pipe_obj = self.pipelines[dest_pipe_id]
        dest_3d = dest_pipe_obj.start_pt + dest_pipe_obj.direction * dest_pos
        pq = [(0.0, start_node, [start_node])]
        visited = set()
        while pq:
            cost, curr_node, path = heapq.heappop(pq)
            if curr_node in visited:
                continue
            visited.add(curr_node)
            if curr_node == end_node:
                return path
            edges = self._get_edges_for_node(
                curr_node,
                start_pipe_id,
                start_pos,
                dest_pipe_id,
                dest_pos,
                dest_3d,
                uav_speed=uav_speed,
            )
            for neighbor, actual_weight, edge_type, target_pipe, n_coords in edges:
                if neighbor not in visited:
                    heapq.heappush(
                        pq, (cost + actual_weight, neighbor, path + [neighbor])
                    )
        return []

    @staticmethod
    def _extract_ramp_id_from_node(node_name):
        if "_R" in node_name:
            try:
                return int(node_name.split("_R")[1])
            except (ValueError, IndexError):
                return None
        return None

    def find_directed_ramp_id(self, source_pipe_id, target_pipe_id, target_node=None):
        source_pipe_id = int(source_pipe_id)
        target_pipe_id = int(target_pipe_id)
        ramp_id = self._extract_ramp_id_from_node(target_node or "")
        if target_node is not None and ramp_id is None:
            return None
        if ramp_id is not None:
            ramp = self.ramp_by_id.get(int(ramp_id))
            if ramp is not None:
                forward = (
                    int(ramp.source_pipe_id) == source_pipe_id
                    and int(ramp.target_pipe_id) == target_pipe_id
                )
                reverse = (
                    bool(getattr(ramp, "is_bidirectional", False))
                    and int(ramp.target_pipe_id) == source_pipe_id
                    and (int(ramp.source_pipe_id) == target_pipe_id)
                )
                if forward or reverse:
                    return int(ramp.ramp_id)
            return None
        candidates = self.directed_ramps_by_pipe_pair.get(
            (source_pipe_id, target_pipe_id), ()
        )
        return int(candidates[0]) if len(candidates) == 1 else None
