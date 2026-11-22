"""Two-level Dijkstra router for regional_6x6 airspace."""
from collections import OrderedDict
from dataclasses import dataclass

from uav_sim.airspace.region_graph import RegionGraph
from uav_sim.airspace.ramp_cost import speed_bucket, switch_edge_cost
from uav_sim.airspace.regional_matrix import LocalPathTarget, RegionalAirspaceMatrix


@dataclass(frozen=True)
class _SegmentEndpoint:
    pipe_id: int
    pos: float
    node_name: str = None


@dataclass
class _CorridorState:
    cost: float
    endpoint: _SegmentEndpoint
    raw_path: list
    node_info_patch: dict
    local_segment_count: int


class HierarchicalRegionalRouter:
    """Route on a 36-node region graph, then inside small regional graphs."""

    def __init__(self, env_snapshot, routing_mode="DIJKSTRA",
                 max_portals_per_edge=3, route_cache_capacity=50000):
        self.env_snapshot = env_snapshot
        self.routing_mode = routing_mode
        self.matrix = RegionalAirspaceMatrix.from_snapshot(env_snapshot)
        self.region_graph = RegionGraph(self.matrix.regions, self.matrix.cross_region_portals)
        self.preserve_full_routing_view = (
            str(env_snapshot.get("topology_profile", ""))
            == "legacy_dense_bidirectional_v1"
        )
        self._routing_view_signature = (
            int(env_snapshot.get("topology_version", 0)),
            None,
        )
        self.max_portals_per_edge = int(max(1, max_portals_per_edge))
        self._route_cache_capacity = max(1, int(route_cache_capacity))
        self._route_cache = OrderedDict()
        self.cache_hits = 0
        self.cache_misses = 0
        self.cache_evictions = 0

    def find_path(self, start_pipe_id, start_pos, dest_pipe_id, dest_pos,
                  ramp_states=None, traffic_multiplier=None,
                  ramp_state_version=None, traffic_cost_version=None,
                  topology_version=None, uav_speed=None):
        if ramp_state_version is None:
            ramp_state_version = self._ramp_state_cache_token(ramp_states)
        if traffic_cost_version is None:
            traffic_cost_version = self._traffic_state_cache_token(traffic_multiplier)
        if topology_version is None:
            topology_version = int(self.env_snapshot.get("topology_version", 0))
        self._ensure_active_routing_view(
            ramp_states,
            ramp_state_version,
            topology_version,
        )
        cache_key = (
            int(start_pipe_id),
            round(float(start_pos), 1),
            int(dest_pipe_id),
            round(float(dest_pos), 1),
            speed_bucket(uav_speed),
            int(topology_version),
            int(ramp_state_version),
            int(traffic_cost_version),
        )
        if cache_key in self._route_cache:
            self._route_cache.move_to_end(cache_key)
            raw_path, node_info_patch, diagnostics = self._route_cache[cache_key]
            diagnostics = dict(diagnostics)
            diagnostics["cache_hit"] = True
            self.cache_hits += 1
            return list(raw_path), dict(node_info_patch), diagnostics

        self.cache_misses += 1
        raw_path, node_info_patch, diagnostics = self._find_uncached(
            start_pipe_id, start_pos, dest_pipe_id, dest_pos,
            ramp_states=ramp_states, traffic_multiplier=traffic_multiplier,
            ramp_state_version=ramp_state_version,
            traffic_cost_version=traffic_cost_version,
            uav_speed=uav_speed)
        diagnostics = dict(diagnostics)
        diagnostics["cache_hit"] = False
        self._route_cache[cache_key] = (list(raw_path), dict(node_info_patch), dict(diagnostics))
        self._route_cache.move_to_end(cache_key)
        if len(self._route_cache) > self._route_cache_capacity:
            self._route_cache.popitem(last=False)
            self.cache_evictions += 1
        return raw_path, node_info_patch, diagnostics

    @staticmethod
    def _ramp_state_cache_token(ramp_states):
        if not ramp_states:
            return 0
        return hash(tuple(sorted((int(k), bool(v)) for k, v in ramp_states.items())))

    @staticmethod
    def _traffic_state_cache_token(traffic_multiplier):
        if not traffic_multiplier:
            return 0
        return hash(tuple(sorted(
            (int(key), round(float(value), 8))
            for key, value in traffic_multiplier.items()
        )))

    @staticmethod
    def _ramp_id_from_node(node_name):
        if "_R" not in node_name:
            return None
        try:
            return int(node_name.rsplit("_R", 1)[1])
        except (TypeError, ValueError):
            return None

    def _ensure_active_routing_view(self, ramp_states, ramp_state_version,
                                    topology_version):
        signature = (
            int(topology_version),
            None if ramp_states is None else int(ramp_state_version),
        )
        if signature == self._routing_view_signature:
            return

        if self.preserve_full_routing_view:
            self._routing_view_signature = signature
            return

        if ramp_states is None:
            snapshot = self.env_snapshot
        else:
            active_ramp_ids = {
                int(ramp.ramp_id)
                for ramp in self.env_snapshot["ramps"]
                if bool(ramp_states.get(int(ramp.ramp_id), True))
            }
            node_info = self.env_snapshot.get("node_info", {})

            def node_is_active(node_name):
                info = node_info.get(node_name, {})
                if info.get("type") != "RAMP":
                    return True
                ramp_id = self._ramp_id_from_node(node_name)
                return ramp_id is not None and ramp_id in active_ramp_ids

            snapshot = dict(self.env_snapshot)
            snapshot["ramps"] = [
                ramp for ramp in self.env_snapshot["ramps"]
                if int(ramp.ramp_id) in active_ramp_ids
            ]
            snapshot["pipe_nodes"] = {
                int(pipe_id): [
                    node for node in nodes if node_is_active(node[0])
                ]
                for pipe_id, nodes in self.env_snapshot.get("pipe_nodes", {}).items()
            }
            snapshot["node_info"] = {
                node_name: dict(info)
                for node_name, info in node_info.items()
                if node_is_active(node_name)
            }
            snapshot["graph"] = {}

        self.matrix = RegionalAirspaceMatrix.from_snapshot(snapshot)
        self.region_graph = RegionGraph(
            self.matrix.regions,
            self.matrix.cross_region_portals,
        )
        self._routing_view_signature = signature

    def _find_uncached(self, start_pipe_id, start_pos, dest_pipe_id, dest_pos,
                       ramp_states=None, traffic_multiplier=None,
                       ramp_state_version=0, traffic_cost_version=0,
                       uav_speed=None):
        start_pipe_id = int(start_pipe_id)
        dest_pipe_id = int(dest_pipe_id)
        start_region = self.matrix.region_for_pipe(start_pipe_id)
        dest_region = self.matrix.region_for_pipe(dest_pipe_id)
        region_plan = self.region_graph.shortest_region_path(
            start_region, dest_region, ramp_states=ramp_states)

        if not region_plan.region_ids:
            return self._fallback(start_pipe_id, start_pos, dest_pipe_id, dest_pos, [])

        if start_region == dest_region:
            region = self.matrix.get_region(start_region)
            local = region.find_local_path(
                start_pipe_id, start_pos, dest_pipe_id, dest_pos,
                ramp_states=ramp_states, traffic_multiplier=traffic_multiplier,
                ramp_state_version=ramp_state_version,
                traffic_cost_version=traffic_cost_version,
                uav_speed=uav_speed)
            if local.raw_path:
                return local.raw_path, local.node_info_patch, {
                    "router": "hierarchical_regional",
                    "region_path": [start_region],
                    "local_segment_count": 1,
                    "fallback_used": False,
                }
            return self._fallback(start_pipe_id, start_pos, dest_pipe_id, dest_pos, [start_region])

        corridor_result = self._find_corridor_path(
            region_plan.region_ids,
            start_pipe_id=start_pipe_id,
            start_pos=start_pos,
            dest_pipe_id=dest_pipe_id,
            dest_pos=dest_pos,
            ramp_states=ramp_states,
            traffic_multiplier=traffic_multiplier,
            ramp_state_version=ramp_state_version,
            traffic_cost_version=traffic_cost_version,
            uav_speed=uav_speed,
        )
        if corridor_result is None:
            return self._fallback(start_pipe_id, start_pos, dest_pipe_id, dest_pos, region_plan.region_ids)
        stitched_path, stitched_patch, local_segment_count, _total_cost = corridor_result

        return stitched_path, stitched_patch, {
            "router": "hierarchical_regional",
            "region_path": list(region_plan.region_ids),
            "local_segment_count": local_segment_count,
            "fallback_used": False,
        }

    def _find_corridor_path(self, region_ids, start_pipe_id, start_pos,
                            dest_pipe_id, dest_pos, ramp_states,
                            traffic_multiplier, ramp_state_version,
                            traffic_cost_version, uav_speed=None):
        states = {
            "start": _CorridorState(
                cost=0.0,
                endpoint=_SegmentEndpoint(int(start_pipe_id), float(start_pos)),
                raw_path=[],
                node_info_patch={},
                local_segment_count=0,
            )
        }

        for region_id, next_region_id in zip(region_ids, region_ids[1:]):
            region = self.matrix.get_region(region_id)
            next_states = {}
            for state in states.values():
                portals = self._candidate_portals(
                    region_id,
                    next_region_id,
                    state.endpoint.pipe_id,
                    dest_pipe_id,
                    ramp_states,
                )
                portal_by_key = {portal.ramp_id: portal for portal in portals}
                targets = []
                for portal in portals:
                    exit_node, _entry_node, exit_pipe, exit_pos, _entry_pipe, _entry_pos = (
                        self._portal_direction(portal, region_id, next_region_id)
                    )
                    targets.append(LocalPathTarget(
                        key=portal.ramp_id,
                        pipe_id=exit_pipe,
                        pos=exit_pos,
                        node_name=exit_node,
                    ))
                local_results = region.find_paths_to_targets(
                    state.endpoint.pipe_id,
                    state.endpoint.pos,
                    targets,
                    ramp_states=ramp_states,
                    traffic_multiplier=traffic_multiplier,
                    ramp_state_version=ramp_state_version,
                    traffic_cost_version=traffic_cost_version,
                    uav_speed=uav_speed,
                )
                for ramp_id, local_result in local_results.items():
                    if not local_result.raw_path:
                        continue
                    portal = portal_by_key[ramp_id]
                    exit_node, entry_node, _exit_pipe, _exit_pos, entry_pipe, entry_pos = (
                        self._portal_direction(portal, region_id, next_region_id)
                    )
                    raw_path = self._append_segment(
                        state.raw_path,
                        local_result.raw_path,
                    )
                    if not raw_path or raw_path[-1] != exit_node:
                        raw_path.append(exit_node)
                    raw_path.append(entry_node)
                    patch = dict(state.node_info_patch)
                    patch.update(local_result.node_info_patch)
                    patch[exit_node] = dict(self.matrix.node_info[exit_node])
                    patch[entry_node] = dict(self.matrix.node_info[entry_node])
                    total_cost = (
                        state.cost
                        + float(local_result.cost)
                        + switch_edge_cost(
                            self.matrix, portal.ramp_id, uav_speed
                        )
                    )
                    candidate = _CorridorState(
                        cost=total_cost,
                        endpoint=_SegmentEndpoint(entry_pipe, entry_pos, entry_node),
                        raw_path=raw_path,
                        node_info_patch=patch,
                        local_segment_count=state.local_segment_count + 1,
                    )
                    previous = next_states.get(ramp_id)
                    if previous is None or candidate.cost < previous.cost:
                        next_states[ramp_id] = candidate
            if not next_states:
                return None
            states = next_states

        final_region = self.matrix.get_region(region_ids[-1])
        best = None
        for state in states.values():
            final_local = final_region.find_local_path(
                state.endpoint.pipe_id,
                state.endpoint.pos,
                dest_pipe_id,
                dest_pos,
                ramp_states=ramp_states,
                traffic_multiplier=traffic_multiplier,
                ramp_state_version=ramp_state_version,
                traffic_cost_version=traffic_cost_version,
                uav_speed=uav_speed,
            )
            if not final_local.raw_path:
                continue
            raw_path = self._append_segment(state.raw_path, final_local.raw_path)
            if not raw_path or raw_path[0] != "V_START" or raw_path[-1] != "V_END":
                continue
            patch = dict(state.node_info_patch)
            patch.update(final_local.node_info_patch)
            total_cost = state.cost + float(final_local.cost)
            candidate = (
                raw_path,
                patch,
                state.local_segment_count + 1,
                total_cost,
            )
            if best is None or total_cost < best[3]:
                best = candidate
        return best

    def _candidate_portals(self, region_id, next_region_id, current_pipe_id,
                           dest_pipe_id, ramp_states=None):
        portals = self.matrix.cross_portals_between(region_id, next_region_id)
        if ramp_states is not None:
            portals = [
                portal for portal in portals
                if ramp_states.get(int(portal.ramp_id), True)
            ]
        current_pipe = self.matrix.pipelines[int(current_pipe_id)]
        dest_pipe = self.matrix.pipelines[int(dest_pipe_id)]

        def portal_sort_key(portal):
            exit_node, _entry_node, exit_pipe_id, _exit_pos, _entry_pipe, _entry_pos = self._portal_direction(
                portal, region_id, next_region_id)
            exit_pipe = self.matrix.pipelines[exit_pipe_id]
            height_gap = min(
                abs(float(exit_pipe.height) - float(current_pipe.height)),
                abs(float(exit_pipe.height) - float(dest_pipe.height)))
            local_gap = abs(
                int(getattr(exit_pipe, "local_layer_index", 0))
                - int(getattr(current_pipe, "local_layer_index", 0)))
            return (height_gap, local_gap, portal.time_penalty, portal.ramp_id)

        return sorted(portals, key=portal_sort_key)[:self.max_portals_per_edge]

    @staticmethod
    def _append_segment(stitched_path, segment):
        if not segment:
            return stitched_path
        segment = list(segment)
        if stitched_path:
            segment = [node for node in segment if node != "V_START"]
            if segment and segment[0] == stitched_path[-1]:
                segment = segment[1:]
        return stitched_path + segment

    @staticmethod
    def _portal_direction(portal, region_id, next_region_id):
        if portal.region_a_id == region_id and portal.region_b_id == next_region_id:
            return (
                portal.node_a, portal.node_b,
                portal.pipe_a_id, portal.dist_a,
                portal.pipe_b_id, portal.dist_b,
            )
        if (
                bool(getattr(portal, "is_bidirectional", False))
                and portal.region_b_id == region_id
                and portal.region_a_id == next_region_id):
            return (
                portal.node_b, portal.node_a,
                portal.pipe_b_id, portal.dist_b,
                portal.pipe_a_id, portal.dist_a,
            )
        raise ValueError(
            "跨区域匝道方向不允许: {} -> {}".format(
                region_id, next_region_id)
        )

    def _fallback(self, start_pipe_id, start_pos, dest_pipe_id, dest_pos, region_path):
        return [], {}, {
            "router": "hierarchical_regional",
            "region_path": list(region_path or []),
            "local_segment_count": 0,
            "fallback_used": False,
            "route_failed": True,
        }
