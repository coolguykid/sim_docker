import heapq
import pickle

import pytest

from uav_sim.airspace.partition_contracts import PartitionGraph, RoadNode, RoadEdge


def graph_fixture():
    # Four exits: only the fourth reaches the goal. Returning to region zero
    # is cheaper than the available direct local path.
    regions = {i: {} for i in range(6)}
    node_regions = {"s": 0, "g": 0, "a": 0, "b": 0, "c": 0, "d": 0,
                    "a1": 1, "b2": 2, "c3": 3, "d4": 4, "e4": 4, "back": 0,
                    "almost": 0, "isolated": 5}
    nodes = {n: RoadNode(n, r, (float(r), 0., 100.)) for n, r in node_regions.items()}
    edges = {}
    def add(name, source, target, cost, kind="FLY", ramp=None):
        edges[name] = RoadEdge(name, source, target, node_regions[source], kind, cost, 1.,
                               (nodes[source].position, nodes[target].position), ramp_id=ramp)
    add("direct", "s", "g", 100.)
    for idx, exit_node in enumerate("abcd", 1):
        add("to_" + exit_node, "s", exit_node, float(idx))
        add("cross_" + exit_node, exit_node, exit_node + str(idx), 0., "BOUNDARY")
    add("remote", "d4", "e4", 2., ramp=7)
    add("reenter", "e4", "back", 0., "BOUNDARY")
    add("finish", "back", "g", 3.)
    add("close_but_different", "almost", "g", 7.)
    return PartitionGraph(nodes, edges, regions, ramp_states={7: True})


def oracle(graph, start, target):
    adjacency = {}
    for edge in graph.edges.values():
        if edge.ramp_id is not None and not graph.ramp_states.get(edge.ramp_id, True):
            continue
        adjacency.setdefault(edge.source, []).append((edge.target, edge.length_m / edge.speed_limit_mps))
    queue, distances = [(0., start)], {start: 0.}
    while queue:
        distance, node = heapq.heappop(queue)
        if distance != distances[node]:
            continue
        if node == target:
            return distance
        for neighbor, cost in adjacency.get(node, ()):
            candidate = distance + cost
            if candidate < distances.get(neighbor, float("inf")):
                distances[neighbor] = candidate
                heapq.heappush(queue, (candidate, neighbor))
    return float("inf")


def test_local_router_matches_oracle_for_fourth_exit_and_same_region_reentry():
    from uav_sim.airspace.partition_graph import PartitionMaps
    from uav_sim.airspace.partition_router import RegionalDijkstra
    graph = graph_fixture()
    maps = PartitionMaps(graph)
    local = maps.region(0)
    router = RegionalDijkstra(maps.summary(), local)
    plan = router.plan("s", maps.destination("g"))
    assert plan.status == "OK"
    assert plan.cost == oracle(graph, "s", "g") == 9.
    assert plan.edge_ids == ("to_d", "cross_d")
    assert plan.region_path == (0, 4, 0)
    assert "remote" not in local.edges
    assert "d4" in local.nodes  # an outgoing boundary anchor is allowed
    assert "e4" not in local.nodes
    assert not hasattr(router, "env") and not hasattr(router, "maps") and not hasattr(router, "graph")
    assert all(not hasattr(arc, "points") for arcs in maps.summary().adjacency.values() for arc in arcs)


def test_exact_identity_no_route_versions_and_shared_serializable_views():
    from uav_sim.airspace.partition_graph import PartitionMaps
    from uav_sim.airspace.partition_router import RegionalDijkstra
    graph = graph_fixture()
    maps = PartitionMaps(graph)
    summary = maps.summary()
    router = RegionalDijkstra(summary, maps.region(0))
    assert router.summary is summary
    assert maps.summary() is summary and maps.region(0) is maps.region(0)
    assert router.plan("almost", maps.destination("g")).cost == 7.
    assert router.plan("s", maps.destination("isolated")).status == "NO_ROUTE"
    with pytest.raises(TypeError):
        summary.adjacency["invented"] = ()
    assert pickle.loads(pickle.dumps(summary)).adjacency == summary.adjacency
    previous_destination = maps.destination("g")
    assert maps.set_ramp_state(7, False) == 1
    assert maps.set_ramp_state(7, False) == 1
    updated = maps.summary()
    assert updated is not summary and summary.version == 0
    router.update_summary(updated)
    assert router.plan("s", previous_destination).status == "STALE_MAP"
    router.replace_region(maps.region(0))
    new_plan = router.plan("s", maps.destination("g"))
    assert new_plan.cost == oracle(graph, "s", "g") == 100.
    assert new_plan.region_path == (0,)
    assert new_plan.edge_ids == ("direct",)


def test_compiled_metropolis_routes_match_independent_full_graph_oracle():
    from uav_sim.airspace.environment import Airspace
    from uav_sim.airspace.partition_graph import compile_partition, PartitionMaps
    from uav_sim.airspace.partition_router import RegionalDijkstra
    env = Airspace(mode="metropolis", num_pipes_per_layer=6, include_dynamic=True)
    specs = [dict(node_id="goal{}".format(i), pipe_id=30 + i,
                  distance_m=env.pipelines[30 + i].length * (.2 + i * .1), lane_id=i)
             for i in range(6)]
    graph = compile_partition(env, specs)
    maps = PartitionMaps(graph)
    summary = maps.summary()
    for start in ["P{}_START".format(i) for i in range(6)]:
        region = maps.region(graph.nodes[start].region_id)
        router = RegionalDijkstra(summary, region)
        for spec in specs:
            plan = router.plan(start, maps.destination(spec["node_id"]))
            assert plan.status == "OK"
            assert plan.cost == pytest.approx(oracle(graph, start, spec["node_id"]), rel=1.e-10)
            assert all(eid in region.edges for eid in plan.edge_ids)
            assert sum(region.edges[eid].kind == "BOUNDARY" for eid in plan.edge_ids) <= 1
            if plan.edge_ids:
                assert region.edges[plan.edge_ids[0]].source == start


def test_goal_potentials_are_shared_without_rescanning_summary_per_aircraft():
    from uav_sim.airspace.partition_graph import PartitionMaps
    from uav_sim.airspace.partition_router import RegionalDijkstra
    maps = PartitionMaps(graph_fixture())
    summary = maps.summary()
    class CountedAdjacency(dict):
        reads = 0
        def get(self, *args):
            self.reads += 1
            return super().get(*args)
        def items(self):
            self.reads += len(self)
            return super().items()
    summary.adjacency = CountedAdjacency(summary.adjacency)
    goal = maps.destination("g")
    first_router = RegionalDijkstra(summary, maps.region(0))
    first = first_router.plan("s", goal)
    initial_reads = summary.adjacency.reads
    assert initial_reads > 0
    second_router = RegionalDijkstra(summary, maps.region(0))
    second = second_router.plan("s", goal)
    assert first_router._adjacency is second_router._adjacency
    assert first == second
    assert summary.adjacency.reads == initial_reads
