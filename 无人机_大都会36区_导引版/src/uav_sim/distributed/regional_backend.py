"""Execution adapter for the three routing ownership modes.

Only the authoritative server keeps PartitionMaps. Onboard Ray actors receive
one deduplicated current-region bundle per requested region and dispatch, goal
descriptors and a summary containing portal IDs/costs, without world geometry.
"""
import math
import os
from collections import defaultdict
from numbers import Integral

from uav_sim.distributed.route_planning_server import PartitionRoutePlanningCore, validate_partition_context
from uav_sim.distributed.uav_actor import RegionalUAVRoutingActor, RegionalUAVRoutingGroup, ray


class RegionalRoutingBackend:
    MODES = ("centralized_full", "centralized_regional", "onboard_regional")

    def __init__(self, maps, mode="centralized_full", group_size=300, use_ray=True, target_uavs=10000):
        if mode not in self.MODES:
            raise ValueError("unknown routing mode: {}".format(mode))
        for name, value in (("group_size", group_size), ("target_uavs", target_uavs)):
            if isinstance(value, bool) or not isinstance(value, Integral) or value <= 0:
                raise ValueError("{} must be a positive integer".format(name))
        self.maps, self.mode = maps, mode
        self.group_size, self.target_uavs = int(group_size), int(target_uavs)
        self.use_ray = bool(use_ray) and mode == "onboard_regional"
        self._owns_ray = False
        self._closed = False
        self._dt = None
        self._calls = 0
        self._workers = []
        self._server = None
        self._summary_version = None
        self._summary_ref = None
        self._last_metrics = None
        if mode != "onboard_regional":
            self._server = PartitionRoutePlanningCore(maps, full_route=mode == "centralized_full")
            self.worker_count = 1
            return
        self.worker_count = int(math.ceil(float(target_uavs) / group_size))
        if self.use_ray:
            self._start_ray_workers()
        else:
            summary = maps.summary()
            self._summary_version = (summary.version, summary.cost_revision)
            self._workers = [RegionalUAVRoutingGroup(index, summary) for index in range(self.worker_count)]

    def _start_ray_workers(self):
        if ray is None or RegionalUAVRoutingActor is None:
            raise RuntimeError("onboard_regional with use_ray=True requires Ray; install the project Ray dependency")
        initialized = ray.is_initialized()
        cpus = float(ray.cluster_resources().get("CPU", 0.)) if initialized else float(os.cpu_count() or 1)
        max_workers = max(4, min(256, int(cpus * 4)))
        if cpus <= 0 or self.worker_count > max_workers:
            raise RuntimeError("{} routing actors exceed the available CPU worker budget ({} CPUs, maximum {} actors); "
                               "increase group_size to at least {} or add Ray CPUs".format(
                                   self.worker_count, cpus, max_workers,
                                   int(math.ceil(float(self.target_uavs) / max_workers))))
        try:
            if not initialized:
                # Budget all requested workers explicitly; never wait for an
                # impossible set of actors each reserving a whole CPU.
                ray.init(num_cpus=min(int(cpus), max(1, self.worker_count)),
                         include_dashboard=False, log_to_driver=False)
                self._owns_ray = True
                cpus = float(ray.cluster_resources().get("CPU", 0.))
            available_cpus = float(ray.available_resources().get("CPU", 0.))
            if available_cpus < .01 * self.worker_count:
                raise RuntimeError("insufficient available Ray CPU resources ({} free for {} actors); "
                                   "release other actors, increase group_size, or add Ray CPUs".format(
                                       available_cpus, self.worker_count))
            num_cpus = min(1., math.floor(available_cpus / self.worker_count * .9 * 10000.) / 10000.)
            summary = self.maps.summary()
            self._summary_ref = ray.put(summary)
            self._summary_version = (summary.version, summary.cost_revision)
            # Source is sent explicitly so copied deliveries do not depend on
            # the baseline project's installed/editable package location.
            source_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            actor = RegionalUAVRoutingActor.options(num_cpus=num_cpus,
                        runtime_env={"env_vars": {"PYTHONPATH": source_path}})
            self._workers = [actor.remote(index, self._summary_ref) for index in range(self.worker_count)]
            ray.get([worker.__ray_ready__.remote() for worker in self._workers], timeout=60.)
        except Exception as error:
            self.close()
            raise RuntimeError("unable to start Ray routing workers; check available Ray CPUs and increase group_size: {}".format(error)) from error

    def _ensure_open(self):
        if self._closed:
            raise RuntimeError("routing backend is closed")

    @staticmethod
    def _normalize_requests(requests):
        normalized = {}
        for uid, request in requests.items():
            if isinstance(uid, bool) or not isinstance(uid, Integral) or uid < 0:
                raise ValueError("UAV IDs must be nonnegative integers")
            rid = request.get("region_id")
            if isinstance(rid, bool) or not isinstance(rid, Integral) or rid < 0:
                raise ValueError("region_id must be a nonnegative integer")
            try:
                start, goal = request["start_node_id"], request["goal_node_id"]
            except KeyError as error:
                raise ValueError("route requests require start_node_id and goal_node_id") from error
            normalized[int(uid)] = dict(start_node_id=str(start), region_id=int(rid), goal_node_id=str(goal))
        return normalized

    def plan_many(self, requests, context, maps=None):
        self._ensure_open()
        self._dt = validate_partition_context(context, self.maps.graph.version, self._dt)
        maps = self.maps if maps is None else maps
        requests = self._normalize_requests(requests)
        self._calls += 1
        if self._server is not None:
            return self._server.plan_many(requests, context, maps=maps)
        self._refresh_summary(maps)
        groups = defaultdict(dict)
        for uid, request in requests.items():
            groups[uid % self.worker_count][uid] = request
        bundles, destinations = {}, {}
        for request in requests.values():
            rid, goal = request["region_id"], request["goal_node_id"]
            if rid not in bundles:
                try:
                    bundles[rid] = maps.region(rid)
                except KeyError:
                    pass
            if goal not in destinations:
                try:
                    destinations[goal] = maps.destination(goal)
                except KeyError:
                    pass
        results, futures = {}, []
        for index, group in groups.items():
            region_ids = {r["region_id"] for r in group.values()}
            goal_ids = {r["goal_node_id"] for r in group.values()}
            current = {rid: bundles[rid] for rid in region_ids if rid in bundles}
            goals = {goal: destinations[goal] for goal in goal_ids if goal in destinations}
            if self.use_ray:
                futures.append(self._workers[index].plan_many.remote(group, current, goals, context))
            else:
                results.update(self._workers[index].plan_many(group, current, goals, context))
        if futures:
            for group_result in ray.get(futures, timeout=120.):
                results.update(group_result)
        return results

    def _refresh_summary(self, maps):
        summary = maps.summary()
        revision = (summary.version, summary.cost_revision)
        if revision == self._summary_version:
            return
        if self.use_ray:
            self._summary_ref = ray.put(summary)
            ray.get([worker.update_summary.remote(self._summary_ref) for worker in self._workers], timeout=60.)
        else:
            for worker in self._workers:
                worker.update_summary(summary)
        self._summary_version = revision

    def discard(self, uids):
        self._ensure_open()
        if self._server is not None:
            return self._server.discard(uids)
        groups = defaultdict(list)
        for uid in uids:
            groups[int(uid) % self.worker_count].append(uid)
        if self.use_ray:
            ray.get([self._workers[index].discard.remote(ids) for index, ids in groups.items()], timeout=60.)
        else:
            for index, ids in groups.items():
                self._workers[index].discard(ids)

    def snapshots(self):
        self._ensure_open()
        if self._server is not None:
            return self._server.snapshots()
        states = ray.get([worker.snapshots.remote() for worker in self._workers], timeout=60.) if self.use_ray else [worker.snapshots() for worker in self._workers]
        return {uid: snapshot for group in states for uid, snapshot in group.items()}

    def metrics(self):
        if self._closed:
            return dict(self._last_metrics or {}, calls=self._calls)
        if self._server is not None:
            self._last_metrics = self._server.metrics()
        else:
            groups = ray.get([worker.metrics.remote() for worker in self._workers], timeout=60.) if self.use_ray else [worker.metrics() for worker in self._workers]
            self._last_metrics = {key: sum(group[key] for group in groups)
                                  for key in ("planning_calls", "requests", "map_loads", "cache_hits", "private_planners")}
            self._last_metrics.update(worker_count=len(groups),
                                      worker_pids=tuple(sorted({pid for group in groups for pid in group["worker_pids"]})))
        return dict(self._last_metrics, calls=self._calls, use_ray=self.use_ray, mode=self.mode)

    def close(self):
        if self._closed:
            return
        if self.use_ray and ray is not None and ray.is_initialized():
            for worker in self._workers:
                try:
                    ray.kill(worker, no_restart=True)
                except Exception:
                    pass
            if self._owns_ray:
                ray.shutdown()
        self._workers.clear()
        self._summary_ref = None
        self._closed = True

    def __enter__(self):
        self._ensure_open()
        return self

    def __exit__(self, *exception):
        self.close()
