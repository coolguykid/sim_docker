import time
from collections import OrderedDict
from uav_sim.airspace.hierarchical_router import HierarchicalRegionalRouter
from uav_sim.airspace.ramp_cost import speed_bucket
from uav_sim.distributed.messages import RouteUpdate, as_route_request
from uav_sim.distributed.local_env_proxy import LocalEnvProxy

try:
    import ray
except ModuleNotFoundError:
    ray = None


class RoutePlanningCore:
    """Non-Ray implementation used by the Ray actor and direct tests."""

    def __init__(self, env_snapshot, routing_mode="DIJKSTRA"):
        if routing_mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        self.env = LocalEnvProxy(env_snapshot, routing_mode=routing_mode)
        self.routing_mode = routing_mode
        self._current_ramp_states = None
        self._current_collisions_multiplier = None
        self._current_ramp_state_version = 0
        self._current_traffic_cost_version = 0
        self._current_topology_version = int(env_snapshot.get("topology_version", 0))
        self.use_hierarchical_regional = (
            env_snapshot.get("airspace_mode") == "regional_6x6"
        )
        self.regional_router = (
            HierarchicalRegionalRouter(env_snapshot, routing_mode=routing_mode)
            if self.use_hierarchical_regional
            else None
        )

    def set_routing_mode(self, mode):
        if mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        self.routing_mode = mode
        self.env.routing_mode = mode
        self.use_hierarchical_regional = self.regional_router is not None

    def plan_routes(self, requests, global_context=None):
        self._refresh_context(global_context or {})
        normalized_requests = [as_route_request(item) for item in requests or []]
        return self._plan_coalesced_dijkstra(normalized_requests)

    def _plan_coalesced_dijkstra(self, requests):
        groups = OrderedDict()
        for index, request in enumerate(requests):
            key = (
                int(request.current_pipe_id),
                round(float(request.pos_1d), 3),
                int(request.dest_pipe_id),
                round(float(request.dest_pos), 3),
                str(request.phase),
                speed_bucket(request.speed),
                int(self._current_topology_version),
                int(self._current_ramp_state_version),
                int(self._current_traffic_cost_version),
            )
            groups.setdefault(key, []).append((index, request))
        planned = [None] * len(requests)
        total_unique = len(groups)
        started_at = time.perf_counter()
        last_log_at = started_at
        for unique_index, group in enumerate(groups.values(), start=1):
            template = self._plan_one(group[0][1])
            for group_index, (output_index, request) in enumerate(group):
                diagnostics = dict(template.diagnostics or {})
                if group_index > 0:
                    diagnostics["cache_hit"] = True
                    diagnostics["coalesced"] = True
                planned[output_index] = RouteUpdate(
                    uav_id=request.uav_id,
                    raw_path=template.raw_path,
                    dest_pos=request.dest_pos,
                    phase=request.phase,
                    status=template.status,
                    node_info_patch=template.node_info_patch,
                    diagnostics=diagnostics,
                )
            now = time.perf_counter()
            if (
                self.use_hierarchical_regional
                and total_unique >= 500
                and (
                    unique_index == total_unique
                    or unique_index % 500 == 0
                    or now - last_log_at >= 5.0
                )
            ):
                print(
                    f"[RoutePlanningServer] coalesced hierarchical DIJKSTRA: {unique_index}/{total_unique} unique routes from {len(requests)} requests, elapsed={now - started_at:.1f}s",
                    flush=True,
                )
                last_log_at = now
        return planned

    def _refresh_context(self, context):
        if not context:
            return
        self._current_ramp_states = context.get("ramp_states")
        self._current_collisions_multiplier = context.get("collisions_multiplier")
        if "ramp_state_version" in context:
            self._current_ramp_state_version = int(context["ramp_state_version"])
        elif self._current_ramp_states:
            self._current_ramp_state_version = hash(
                tuple(
                    sorted(
                        (
                            (int(key), bool(value))
                            for key, value in self._current_ramp_states.items()
                        )
                    )
                )
            )
        if "traffic_cost_version" in context:
            self._current_traffic_cost_version = int(context["traffic_cost_version"])
        elif self._current_collisions_multiplier:
            self._current_traffic_cost_version = hash(
                tuple(
                    sorted(
                        (
                            (int(key), round(float(value), 8))
                            for key, value in self._current_collisions_multiplier.items()
                        )
                    )
                )
            )
        self._current_topology_version = int(
            context.get("topology_version", self._current_topology_version)
        )
        self.env.refresh_shared_state(
            context.get("collisions_multiplier", {}),
            context.get("traffic_load", {}),
            context.get("ramp_states", {}),
        )

    def _plan_one(self, request):
        dest_pipe = self.env.pipelines[request.dest_pipe_id]
        dest_3d = dest_pipe.start_pt + dest_pipe.direction * request.dest_pos
        mode = self.routing_mode
        raw_path = []
        diagnostics = {}
        node_info_patch = None
        if self.use_hierarchical_regional and self.regional_router is not None:
            raw_path, node_info_patch, diagnostics = self.regional_router.find_path(
                request.current_pipe_id,
                request.pos_1d,
                request.dest_pipe_id,
                request.dest_pos,
                ramp_states=self._current_ramp_states,
                traffic_multiplier=self._current_collisions_multiplier,
                ramp_state_version=self._current_ramp_state_version,
                traffic_cost_version=self._current_traffic_cost_version,
                topology_version=self._current_topology_version,
                uav_speed=request.speed,
            )
        else:
            raw_path = self.env.find_shortest_path(
                request.current_pipe_id,
                request.pos_1d,
                request.dest_pipe_id,
                request.dest_pos,
                uav_speed=request.speed,
            )
        route_failed = not raw_path or len(raw_path) < 2 or raw_path[-1] != "V_END"
        if route_failed and (not self.use_hierarchical_regional):
            raw_path = self.env.find_shortest_path(
                request.current_pipe_id,
                request.pos_1d,
                request.dest_pipe_id,
                request.dest_pos,
                uav_speed=request.speed,
            )
            node_info_patch = None
        return RouteUpdate(
            uav_id=request.uav_id,
            raw_path=list(raw_path or []),
            dest_pos=request.dest_pos,
            phase=request.phase,
            node_info_patch=node_info_patch
            if node_info_patch is not None
            else self._node_info_patch(raw_path),
            diagnostics=diagnostics,
        )

    def _node_info_patch(self, raw_path):
        patch = {}
        for node in raw_path or []:
            if node in ("V_START", "V_END"):
                continue
            if node in self.env.node_info:
                patch[node] = dict(self.env.node_info[node])
        return patch


class _RoutePlanningServer:
    def __init__(self, env_snapshot, routing_mode="DIJKSTRA"):
        if routing_mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        self.core = RoutePlanningCore(env_snapshot, routing_mode=routing_mode)

    def set_routing_mode(self, mode):
        if mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        return self.core.set_routing_mode(mode)

    def plan_routes(self, requests, global_context=None):
        return self.core.plan_routes(requests, global_context)


if ray is not None:
    RoutePlanningServer = ray.remote(num_cpus=1)(_RoutePlanningServer)
else:
    RoutePlanningServer = _RoutePlanningServer


def validate_partition_context(context, version, previous_dt=None):
    """Validate physical time and topology at the routing boundary."""
    import math
    from numbers import Integral

    for name in ("step_index", "topology_version"):
        value = context.get(name)
        if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
            raise ValueError("{} must be a nonnegative integer".format(name))
    for name in ("sim_time_seconds", "sim_dt_seconds"):
        try:
            value = float(context[name])
        except (KeyError, TypeError, ValueError):
            raise ValueError("{} must be finite".format(name))
        if not math.isfinite(value) or value < 0 or (name == "sim_dt_seconds" and value == 0):
            raise ValueError("{} must be finite and {}".format(
                name, "positive" if name == "sim_dt_seconds" else "nonnegative"))
    if context["topology_version"] != version:
        raise ValueError("topology_version does not match the authoritative maps")
    dt = float(context["sim_dt_seconds"])
    if previous_dt is not None and not math.isclose(dt, previous_dt, rel_tol=1.e-12, abs_tol=1.e-15):
        raise ValueError("sim_dt_seconds must remain constant during a run")
    return dt


class PartitionRoutePlanningCore:
    """Authoritative server routing for the virtual physical partitions.

    The existing RoutePlanningCore API remains unchanged. This separate entry
    point supports either a current-region prefix or a complete dispatch route
    assembled by repeatedly asking the same exact regional Dijkstra solver.
    """

    def __init__(self, maps, full_route=False):
        self.maps = maps
        self.full_route = bool(full_route)
        self._dt = None
        self._version = None
        self._cache = OrderedDict()
        self._states = {}
        self._counts = dict(planning_calls=0, requests=0, map_loads=0, cache_hits=0)

    def plan_many(self, requests, context, maps=None):
        from uav_sim.airspace.partition_contracts import RegionPlan

        maps = self.maps if maps is None else maps
        self._dt = validate_partition_context(context, maps.graph.version, self._dt)
        version = maps.graph.version
        if self._version != version:
            self._cache.clear()
            self._version = version
        results = {}
        for uid, request in requests.items():
            self._counts["requests"] += 1
            rid, start, goal = request["region_id"], request["start_node_id"], request["goal_node_id"]
            key = (version, maps.cost_revision, rid, start, goal)
            if key in self._cache:
                result = self._cache[key]
                self._cache.move_to_end(key)
                self._counts["cache_hits"] += 1
            else:
                try:
                    local, destination = maps.region(rid), maps.destination(goal)
                except KeyError:
                    result = RegionPlan("NO_ROUTE", version=version, reason="unknown region or exact goal node")
                else:
                    result = (self._plan_full(start, local, destination, maps) if self.full_route
                              else self._prefix(start, local, destination, maps))
                self._cache[key] = result
                if len(self._cache) > 4096:
                    self._cache.popitem(last=False)
            self._states[uid] = dict(region_id=rid, detailed_region_ids=(), node_count=0,
                                     edge_count=0, version=version)
            results[uid] = result
        return results

    def _prefix(self, start, local, destination, maps):
        from uav_sim.airspace.partition_router import RegionalDijkstra

        self._counts["planning_calls"] += 1
        self._counts["map_loads"] += 1
        return RegionalDijkstra(maps.summary(), local).plan(start, destination)

    def _plan_full(self, start, local, destination, maps):
        from uav_sim.airspace.partition_contracts import RegionPlan

        edge_ids, regions, seen = [], [local.region_id], set()
        node, first = start, None
        while True:
            state = (local.region_id, node)
            if state in seen:
                return RegionPlan("NO_ROUTE", version=local.version, reason="regional prefix made a cycle")
            seen.add(state)
            prefix = self._prefix(node, local, destination, maps)
            if prefix.status != "OK":
                return RegionPlan(prefix.status, version=prefix.version, reason=prefix.reason)
            if first is None:
                first = prefix
            for edge_id in prefix.edge_ids:
                edge = local.edges.get(edge_id)
                if edge is None or edge.source != node:
                    return RegionPlan("NO_ROUTE", version=local.version, reason="noncontiguous regional prefix")
                edge_ids.append(edge_id)
                node = edge.target
                rid = local.nodes[node].region_id
                if rid != regions[-1]:
                    regions.append(rid)
            if node == destination.node_id:
                return RegionPlan("OK", tuple(edge_ids), tuple(regions), first.cost, first.version)
            if not prefix.edge_ids or local.nodes[node].region_id == local.region_id:
                return RegionPlan("NO_ROUTE", version=local.version, reason="regional prefix did not reach a boundary or exact goal")
            local = maps.region(local.nodes[node].region_id)

    def discard(self, uids):
        for uid in uids:
            self._states.pop(uid, None)

    def snapshots(self):
        return {uid: dict(state) for uid, state in self._states.items()}

    def metrics(self):
        import os
        return dict(self._counts, worker_count=1, worker_pids=(os.getpid(),), private_planners=0)
