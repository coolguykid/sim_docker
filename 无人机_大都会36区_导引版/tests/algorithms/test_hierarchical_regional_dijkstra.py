from types import SimpleNamespace

from uav_sim.airspace import Airspace
from uav_sim.airspace.hierarchical_router import HierarchicalRegionalRouter
from uav_sim.airspace.regional_matrix import (
    LocalPathResult,
    LocalPathTarget,
    RegionalAirspaceMatrix,
)


def test_hierarchical_router_finds_cross_region_path_with_diagnostics():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    router = HierarchicalRegionalRouter(env.get_readonly_snapshot())

    start_pipe = next(p for p in env.pipelines if p.region_id == 0 and p.height == 100)
    dest_pipe = next(p for p in env.pipelines if p.region_id == 35 and p.height == 600)

    raw_path, node_info_patch, diagnostics = router.find_path(
        start_pipe.pipe_id,
        0.0,
        dest_pipe.pipe_id,
        dest_pipe.length,
    )

    assert raw_path[0] == "V_START"
    assert raw_path[-1] == "V_END"
    assert diagnostics["router"] == "hierarchical_regional"
    assert diagnostics["fallback_used"] is False
    assert diagnostics["region_path"][0] == 0
    assert diagnostics["region_path"][-1] == 35
    assert len(diagnostics["region_path"]) > 1

    cross_ramp_ids = {
        ramp.ramp_id
        for ramp in env.ramps
        if getattr(ramp, "is_cross_region", False)
    }
    path_ramp_ids = {
        int(node.split("_R")[1])
        for node in raw_path
        if "_R" in node
    }
    assert path_ramp_ids & cross_ramp_ids
    assert all(node in ("V_START", "V_END") or node in node_info_patch for node in raw_path)


def test_hierarchical_router_uses_full_route_cache_on_repeated_request():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    router = HierarchicalRegionalRouter(env.get_readonly_snapshot())

    start_pipe = next(p for p in env.pipelines if p.region_id == 0 and p.height == 100)
    dest_pipe = next(p for p in env.pipelines if p.region_id == 5 and p.height == 100)

    first = router.find_path(start_pipe.pipe_id, 0.0, dest_pipe.pipe_id, dest_pipe.length)
    second = router.find_path(start_pipe.pipe_id, 0.0, dest_pipe.pipe_id, dest_pipe.length)

    assert first[0] == second[0]
    assert first[2]["cache_hit"] is False
    assert second[2]["cache_hit"] is True


def test_hierarchical_router_limits_each_corridor_state_to_three_portals():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    router = HierarchicalRegionalRouter(env.get_readonly_snapshot())
    current_pipe = next(p for p in env.pipelines if p.region_id == 0)
    dest_pipe = next(p for p in env.pipelines if p.region_id == 1)

    portals = router._candidate_portals(
        0,
        1,
        current_pipe.pipe_id,
        dest_pipe.pipe_id,
    )

    assert len(portals) == 3


def test_hierarchical_cache_is_invalidated_by_ramp_state_version():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    router = HierarchicalRegionalRouter(env.get_readonly_snapshot())
    start_pipe = next(p for p in env.pipelines if p.region_id == 0 and p.height == 100)
    dest_pipe = next(p for p in env.pipelines if p.region_id == 5 and p.height == 100)
    states = {ramp.ramp_id: True for ramp in env.ramps}

    first = router.find_path(
        start_pipe.pipe_id, 0.0, dest_pipe.pipe_id, dest_pipe.length,
        ramp_states=states, ramp_state_version=1)
    second = router.find_path(
        start_pipe.pipe_id, 0.0, dest_pipe.pipe_id, dest_pipe.length,
        ramp_states=states, ramp_state_version=2)

    assert first[2]["cache_hit"] is False
    assert second[2]["cache_hit"] is False


def test_region_local_dijkstra_can_resolve_multiple_portal_targets_once():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())
    region = matrix.get_region(0)
    start_pipe = next(p for p in env.pipelines if p.region_id == 0 and p.height == 100)
    portals = matrix.cross_portals_between(0, 1)[:2]
    targets = []
    for portal in portals:
        targets.append(LocalPathTarget(
            key=portal.ramp_id,
            pipe_id=portal.pipe_a_id,
            pos=portal.dist_a,
            node_name=portal.node_a,
        ))

    results = region.find_paths_to_targets(
        start_pipe.pipe_id,
        0.0,
        targets,
        ramp_states={ramp.ramp_id: True for ramp in env.ramps},
        ramp_state_version=1,
    )

    assert set(results) == {portal.ramp_id for portal in portals}
    assert all(result.raw_path[0] == "V_START" for result in results.values())


def test_hierarchical_router_never_uses_closed_cross_region_portal():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())
    router = HierarchicalRegionalRouter(env.get_readonly_snapshot())
    start_pipe = next(p for p in env.pipelines if p.region_id == 0 and p.height == 100)
    dest_pipe = next(p for p in env.pipelines if p.region_id == 1 and p.height == 100)
    states = {ramp.ramp_id: True for ramp in env.ramps}
    closed = {
        portal.ramp_id for portal in matrix.cross_portals_between(0, 1)
    }
    for ramp_id in closed:
        states[ramp_id] = False

    raw_path, patch, diagnostics = router.find_path(
        start_pipe.pipe_id,
        0.0,
        dest_pipe.pipe_id,
        dest_pipe.length,
        ramp_states=states,
        ramp_state_version=1,
    )
    used_ramps = set()
    for left, right in zip(raw_path, raw_path[1:]):
        if "_R" not in left or "_R" not in right:
            continue
        left_ramp = int(left.split("_R", 1)[1])
        right_ramp = int(right.split("_R", 1)[1])
        if left_ramp == right_ramp and patch[left]["pipe_id"] != patch[right]["pipe_id"]:
            used_ramps.add(left_ramp)

    assert raw_path[-1] == "V_END"
    assert diagnostics["region_path"] != [0, 1]
    assert not (used_ramps & closed)


def test_hierarchical_router_does_not_fallback_to_global_dijkstra_when_region_graph_is_closed():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    router = HierarchicalRegionalRouter(env.get_readonly_snapshot())
    start_pipe = next(p for p in env.pipelines if p.region_id == 0 and p.height == 100)
    dest_pipe = next(p for p in env.pipelines if p.region_id == 35 and p.height == 600)
    states = {ramp.ramp_id: True for ramp in env.ramps}
    for ramp in env.ramps:
        if getattr(ramp, "is_cross_region", False):
            states[ramp.ramp_id] = False

    raw_path, patch, diagnostics = router.find_path(
        start_pipe.pipe_id,
        0.0,
        dest_pipe.pipe_id,
        dest_pipe.length,
        ramp_states=states,
        ramp_state_version=1,
    )

    assert raw_path == []
    assert patch == {}
    assert diagnostics["fallback_used"] is False
    assert diagnostics["route_failed"] is True


def test_closed_dynamic_ramps_are_removed_from_active_dijkstra_view_until_opened():
    env = Airspace(
        mode="regional_6x6",
        include_dynamic=True,
        inter_dynamic_level="low",
        intra_dynamic_level="low",
    )
    router = HierarchicalRegionalRouter(env.get_readonly_snapshot())
    start_pipe = next(p for p in env.pipelines if p.region_id == 0 and p.height == 100)
    dest_pipe = next(p for p in env.pipelines if p.region_id == 5 and p.height == 600)
    states = {
        ramp.ramp_id: not getattr(ramp, "is_dynamic", False)
        for ramp in env.ramps
    }

    router.find_path(
        start_pipe.pipe_id,
        0.0,
        dest_pipe.pipe_id,
        dest_pipe.length,
        ramp_states=states,
        ramp_state_version=1,
    )

    assert len(router.matrix.ramps) == 5400
    dynamic_ramp = next(ramp for ramp in env.ramps if ramp.is_dynamic)
    states[dynamic_ramp.ramp_id] = True
    router.find_path(
        start_pipe.pipe_id,
        0.0,
        dest_pipe.pipe_id,
        dest_pipe.length,
        ramp_states=states,
        ramp_state_version=2,
    )
    assert len(router.matrix.ramps) == 5401


def test_legacy_dense_profile_keeps_full_historical_routing_graph_when_dynamic_ramps_are_closed():
    env = Airspace(
        mode="regional_6x6",
        include_dynamic=True,
        topology_profile="legacy_dense_bidirectional_v1",
        inter_dynamic_level="low",
        intra_dynamic_level="low",
    )
    router = HierarchicalRegionalRouter(env.get_readonly_snapshot())
    start_pipe = next(p for p in env.pipelines if p.region_id == 0 and p.height == 100)
    dest_pipe = next(p for p in env.pipelines if p.region_id == 5 and p.height == 600)
    states = {
        ramp.ramp_id: not getattr(ramp, "is_dynamic", False)
        for ramp in env.ramps
    }

    raw_path, _patch, diagnostics = router.find_path(
        start_pipe.pipe_id,
        0.0,
        dest_pipe.pipe_id,
        dest_pipe.length,
        ramp_states=states,
        ramp_state_version=1,
    )

    assert raw_path[-1] == "V_END"
    assert diagnostics["fallback_used"] is False
    assert len(router.matrix.ramps) == 31744
    assert len(router.matrix.node_info) == 67808


def test_hierarchical_route_cache_is_bounded():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    router = HierarchicalRegionalRouter(
        env.get_readonly_snapshot(),
        route_cache_capacity=2,
    )
    start_pipe = next(p for p in env.pipelines if p.region_id == 0 and p.height == 100)
    destinations = [
        next(p for p in env.pipelines if p.region_id == region_id and p.height == 600)
        for region_id in (1, 2, 3)
    ]

    for destination in destinations:
        router.find_path(
            start_pipe.pipe_id,
            0.0,
            destination.pipe_id,
            destination.length,
        )

    assert len(router._route_cache) == 2


def test_region_local_path_cache_is_bounded():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    matrix = RegionalAirspaceMatrix.from_snapshot(env.get_readonly_snapshot())
    region = matrix.get_region(0)
    region._path_cache_capacity = 2
    start_pipe = next(p for p in env.pipelines if p.region_id == 0 and p.height == 100)
    destinations = [
        p for p in env.pipelines
        if p.region_id == 0 and p.height == 600
    ][:3]

    for destination in destinations:
        result = region.find_local_path(
            start_pipe.pipe_id,
            0.0,
            destination.pipe_id,
            destination.length,
        )
        assert result.raw_path[-1] == "V_END"

    assert len(region._path_cache) == 2


def test_corridor_portal_state_search_optimizes_across_region_boundaries():
    portal_a = SimpleNamespace(
        ramp_id=1,
        region_a_id=0,
        region_b_id=1,
        pipe_a_id=10,
        pipe_b_id=110,
        dist_a=10.0,
        dist_b=0.0,
        node_a="A_EXIT",
        node_b="A_ENTRY",
        time_penalty=0.0,
    )
    portal_b = SimpleNamespace(
        ramp_id=2,
        region_a_id=0,
        region_b_id=1,
        pipe_a_id=20,
        pipe_b_id=120,
        dist_a=10.0,
        dist_b=0.0,
        node_a="B_EXIT",
        node_b="B_ENTRY",
        time_penalty=0.0,
    )

    class FirstRegion:
        def find_paths_to_targets(self, _start_pipe, _start_pos, targets, **_kwargs):
            return {
                target.key: LocalPathResult(
                    ["V_START", target.node_name],
                    1.0 if target.key == 1 else 5.0,
                    {target.node_name: {"pipe_id": target.pipe_id, "dist": target.pos}},
                )
                for target in targets
            }

    class FinalRegion:
        def find_local_path(self, start_pipe, _start_pos, *_args, **_kwargs):
            entry = "A_ENTRY" if start_pipe == 110 else "B_ENTRY"
            return LocalPathResult(
                ["V_START", entry, "V_END"],
                100.0 if start_pipe == 110 else 1.0,
                {entry: {"pipe_id": start_pipe, "dist": 0.0}},
            )

    router = HierarchicalRegionalRouter.__new__(HierarchicalRegionalRouter)
    router.max_portals_per_edge = 6
    router.matrix = SimpleNamespace(
        pipelines={
            pipe_id: SimpleNamespace(
                height=100, local_layer_index=0, max_speed=20.0)
            for pipe_id in (0, 10, 20, 110, 120, 999)
        },
        ramp_by_id={
            1: SimpleNamespace(
                source_pipe_id=10,
                target_pipe_id=110,
                length=0.0,
                travel_time_seconds=lambda _speed: 0.0,
            ),
            2: SimpleNamespace(
                source_pipe_id=20,
                target_pipe_id=120,
                length=0.0,
                travel_time_seconds=lambda _speed: 0.0,
            ),
        },
        node_info={
            "A_EXIT": {"pipe_id": 10, "dist": 10.0},
            "A_ENTRY": {"pipe_id": 110, "dist": 0.0},
            "B_EXIT": {"pipe_id": 20, "dist": 10.0},
            "B_ENTRY": {"pipe_id": 120, "dist": 0.0},
        },
        cross_portals_between=lambda _a, _b: [portal_a, portal_b],
        get_region=lambda region_id: FirstRegion() if region_id == 0 else FinalRegion(),
    )

    result = router._find_corridor_path(
        [0, 1],
        start_pipe_id=0,
        start_pos=0.0,
        dest_pipe_id=999,
        dest_pos=100.0,
        ramp_states={1: True, 2: True},
        traffic_multiplier=None,
        ramp_state_version=1,
        traffic_cost_version=0,
    )

    assert result is not None
    raw_path, _patch, _segment_count, total_cost = result
    assert "B_EXIT" in raw_path
    assert "B_ENTRY" in raw_path
    assert "A_EXIT" not in raw_path
    assert total_cost == 6.0
