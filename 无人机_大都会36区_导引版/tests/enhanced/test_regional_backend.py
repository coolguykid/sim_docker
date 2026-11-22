"""Execution contracts for full, centralized regional and onboard routing."""
import os

import pytest

from uav_sim.airspace.partition_contracts import PartitionGraph, RoadEdge, RoadNode
from uav_sim.airspace.partition_graph import PartitionMaps


def maps_fixture():
    regions = {0: {}, 1: {}, 2: {}, 3: {}}
    owners = {"s": 0, "exit0": 0, "entry1": 1, "exit1": 1,
              "entry2": 2, "g": 2, "isolated": 3}
    nodes = {name: RoadNode(name, rid, (float(rid), 0., 100.))
             for name, rid in owners.items()}
    edges = {}
    for name, source, target, length, kind, ramp in (
            ("local0", "s", "exit0", 2., "FLY", None),
            ("cross0", "exit0", "entry1", 0., "BOUNDARY", None),
            ("local1", "entry1", "exit1", 3., "FLY", 7),
            ("cross1", "exit1", "entry2", 0., "BOUNDARY", None),
            ("finish", "entry2", "g", 4., "FLY", None)):
        edges[name] = RoadEdge(name, source, target, owners[source], kind, length, 1.,
                               (nodes[source].position, nodes[target].position), ramp_id=ramp)
    return PartitionMaps(PartitionGraph(nodes, edges, regions, ramp_states={7: True}))


def context(maps, step=0, dt=.05):
    return dict(step_index=step, sim_time_seconds=step * dt,
                sim_dt_seconds=dt, topology_version=maps.graph.version)


def request(start="s", region=0, goal="g"):
    return dict(start_node_id=start, region_id=region, goal_node_id=goal)


def backend_class():
    from uav_sim.distributed.regional_backend import RegionalRoutingBackend
    return RegionalRoutingBackend


def test_full_server_returns_complete_route_while_regional_modes_return_current_prefix():
    cls = backend_class()
    maps = maps_fixture()
    with cls(maps, mode="centralized_full", use_ray=False) as full:
        result = full.plan_many({0: request()}, context(maps))[0]
        assert result.status == "OK"
        assert result.edge_ids == ("local0", "cross0", "local1", "cross1", "finish")
        assert result.region_path == (0, 1, 2) and result.cost == 9.
    for mode in ("centralized_regional", "onboard_regional"):
        with cls(maps, mode=mode, use_ray=False, group_size=1, target_uavs=2) as backend:
            result = backend.plan_many({0: request()}, context(maps))[0]
            assert result.status == "OK" and result.cost == 9.
            assert result.edge_ids == ("local0", "cross0")
            assert result.region_path == (0, 1, 2)


def test_worker_private_maps_replace_on_boundary_and_identical_routes_coalesce():
    cls = backend_class()
    maps = maps_fixture()
    with cls(maps, mode="onboard_regional", group_size=3, target_uavs=3, use_ray=False) as backend:
        results = backend.plan_many({1: request(), 2: request()}, context(maps))
        assert results[1] == results[2]
        assert backend.metrics()["planning_calls"] == 1
        assert backend.metrics()["map_loads"] == 2
        assert set(backend.snapshots()) == {1, 2}
        for snapshot in backend.snapshots().values():
            assert snapshot["detailed_region_ids"] == (0,)
            assert snapshot["node_count"] == 3 and snapshot["edge_count"] == 2
        backend.plan_many({1: request("entry1", 1)}, context(maps, 1))
        states = backend.snapshots()
        assert states[1]["region_id"] == 1
        assert states[1]["detailed_region_ids"] == (1,)
        assert states[1]["node_count"] == 3 and states[1]["edge_count"] == 2
        assert states[2]["detailed_region_ids"] == (0,)
        assert backend.metrics()["map_loads"] == 3
        assert backend.metrics()["worker_count"] == 1


def test_dynamic_large_ids_discard_and_new_topology_refresh_all_private_maps():
    cls = backend_class()
    maps = maps_fixture()
    uid = 10 ** 30 + 19
    with cls(maps, mode="onboard_regional", group_size=2, target_uavs=4, use_ray=False) as backend:
        assert backend.plan_many({uid: request()}, context(maps))[uid].status == "OK"
        backend.discard([uid, 99999])
        assert backend.snapshots() == {}
        assert backend.plan_many({uid + 1: request()}, context(maps, 1))[uid + 1].status == "OK"
        maps.set_ramp_state(7, False)
        result = backend.plan_many({uid + 1: request()}, context(maps, 2))[uid + 1]
        assert result.status == "NO_ROUTE" and not result.edge_ids
        assert backend.snapshots()[uid + 1]["version"] == 1
        assert backend.metrics()["map_loads"] == 3


@pytest.mark.parametrize("mode", ["centralized_full", "centralized_regional", "onboard_regional"])
def test_unreachable_and_unknown_goal_never_substitute_a_different_target(mode):
    cls = backend_class()
    maps = maps_fixture()
    with cls(maps, mode=mode, use_ray=False, target_uavs=2) as backend:
        results = backend.plan_many({7: request(goal="isolated"),
                                     8: request(goal="does-not-exist"),
                                     9: request(start="entry1", region=0)}, context(maps))
        assert all(plan.status == "NO_ROUTE" and not plan.edge_ids for plan in results.values())


@pytest.mark.parametrize("change", [
    {"sim_dt_seconds": 0.}, {"sim_dt_seconds": float("nan")},
    {"sim_time_seconds": -1.}, {"sim_time_seconds": float("inf")},
    {"step_index": -1}, {"step_index": 1.5}, {"topology_version": 5},
])
def test_invalid_context_rejected_before_any_planning(change):
    cls = backend_class()
    maps = maps_fixture()
    with cls(maps, mode="onboard_regional", use_ray=False) as backend:
        ctx = context(maps)
        ctx.update(change)
        with pytest.raises(ValueError):
            backend.plan_many({0: request()}, ctx)
        assert backend.metrics()["planning_calls"] == 0


def test_dt_stays_constant_close_is_idempotent_and_use_after_close_rejected():
    cls = backend_class()
    maps = maps_fixture()
    backend = cls(maps, use_ray=False)
    backend.plan_many({0: request()}, context(maps))
    with pytest.raises(ValueError, match="sim_dt_seconds"):
        backend.plan_many({0: request()}, context(maps, 1, dt=.1))
    backend.close()
    backend.close()
    with pytest.raises(RuntimeError, match="closed"):
        backend.plan_many({0: request()}, context(maps))


def test_worker_api_never_receives_or_retains_world_map_service_or_global_graph():
    from uav_sim.distributed.uav_actor import RegionalUAVRoutingGroup
    maps = maps_fixture()
    worker = RegionalUAVRoutingGroup(0, maps.summary())
    results = worker.plan_many({7: request()}, {0: maps.region(0)},
                               {"g": maps.destination("g")}, context(maps))
    assert results[7].status == "OK"
    assert not any(hasattr(worker, name) for name in ("maps", "graph", "env", "world"))
    planner = worker.planners[7]
    assert not any(hasattr(planner, name) for name in ("maps", "graph", "env", "world"))
    assert planner.region_map.region_id == 0
    assert "local1" not in planner.region_map.edges


def test_worker_does_not_reuse_successful_cached_plan_for_stale_goal_descriptor():
    from dataclasses import replace
    from uav_sim.distributed.uav_actor import RegionalUAVRoutingGroup
    maps = maps_fixture()
    worker = RegionalUAVRoutingGroup(0, maps.summary())
    goal = maps.destination("g")
    assert worker.plan_many({7: request()}, {0: maps.region(0)},
                            {"g": goal}, context(maps))[7].status == "OK"
    stale_goal = replace(goal, version=1)
    result = worker.plan_many({7: request()}, {0: maps.region(0)},
                              {"g": stale_goal}, context(maps))[7]
    assert result.status == "STALE_MAP" and not result.edge_ids


def test_actual_ray_workers_match_central_prefixes_and_preserve_external_runtime():
    ray = pytest.importorskip("ray")
    cls = backend_class()
    was_initialized = ray.is_initialized()
    if not was_initialized:
        ray.init(num_cpus=2, include_dashboard=False, log_to_driver=False)
    maps = maps_fixture()
    backend = None
    try:
        with cls(maps, mode="centralized_regional", use_ray=False) as central:
            expected = central.plan_many({1000001: request(), 1000002: request()}, context(maps))
        backend = cls(maps, mode="onboard_regional", group_size=1, target_uavs=2, use_ray=True)
        assert backend.plan_many({1000001: request(), 1000002: request()}, context(maps)) == expected
        metrics = backend.metrics()
        assert metrics["worker_count"] == 2
        assert len(metrics["worker_pids"]) == 2
        assert all(pid != os.getpid() for pid in metrics["worker_pids"])
        assert set(backend.snapshots()) == {1000001, 1000002}
        backend.plan_many({1000001: request("entry1", 1)}, context(maps, 1))
        assert backend.snapshots()[1000001]["detailed_region_ids"] == (1,)
        backend.discard([1000002])
        assert set(backend.snapshots()) == {1000001}
        maps.set_ramp_state(7, False)
        assert backend.plan_many({1000001: request("entry1", 1)}, context(maps, 2))[1000001].status == "NO_ROUTE"
        assert backend.snapshots()[1000001]["version"] == 1
        backend.close()
        assert ray.is_initialized()
    finally:
        if backend is not None:
            backend.close()
        if not was_initialized:
            ray.shutdown()


def test_actual_ray_fully_reserved_cluster_fails_actionably_without_startup_wait():
    import time
    ray = pytest.importorskip("ray")
    cls = backend_class()
    was_initialized = ray.is_initialized()
    if was_initialized:
        pytest.skip("test requires a dedicated two CPU runtime")
    ray.init(num_cpus=2, include_dashboard=False, log_to_driver=False)
    @ray.remote(num_cpus=2)
    class Busy:
        def ready(self):
            return True
    busy = Busy.remote()
    try:
        assert ray.get(busy.ready.remote(), timeout=20.)
        # GCS resource accounting is eventually consistent with actor startup.
        deadline = time.perf_counter() + 5.
        while ray.available_resources().get("CPU", 0.) and time.perf_counter() < deadline:
            time.sleep(.05)
        assert ray.available_resources().get("CPU", 0.) == 0.
        started = time.perf_counter()
        with pytest.raises(RuntimeError, match="available.*CPU"):
            cls(maps_fixture(), mode="onboard_regional", group_size=1, target_uavs=2, use_ray=True)
        assert time.perf_counter() - started < 5.
        assert ray.is_initialized()
    finally:
        ray.kill(busy)
        ray.shutdown()
