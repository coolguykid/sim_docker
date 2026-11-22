from uav_sim.airspace import Airspace
from uav_sim.distributed.messages import RouteRequest
from uav_sim.distributed.route_planning_server import RoutePlanningCore


def _request(env, start_region=0, dest_region=35):
    start_pipe = next(p for p in env.pipelines if p.region_id == start_region and p.height == 100)
    dest_pipe = next(p for p in env.pipelines if p.region_id == dest_region and p.height == 600)
    return RouteRequest(
        uav_id=1,
        current_pipe_id=start_pipe.pipe_id,
        pos_1d=0.0,
        dest_pipe_id=dest_pipe.pipe_id,
        dest_pos=dest_pipe.length,
        phase="TO_PICKUP",
        spawn_turn=0,
        current_turn=0,
        speed=start_pipe.max_speed,
    )


def test_route_planning_core_uses_hierarchical_router_for_regional_dijkstra():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    core = RoutePlanningCore(env.get_readonly_snapshot(), routing_mode="DIJKSTRA")

    update = core.plan_routes([_request(env)])[0]

    assert core.use_hierarchical_regional is True
    assert update.raw_path[0] == "V_START"
    assert update.raw_path[-1] == "V_END"
    assert update.diagnostics["router"] == "hierarchical_regional"
    assert update.diagnostics["fallback_used"] is False
    assert update.diagnostics["region_path"][0] == 0
    assert update.diagnostics["region_path"][-1] == 35


def test_route_planning_core_reports_hierarchical_cache_hit_on_repeated_request():
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    core = RoutePlanningCore(env.get_readonly_snapshot(), routing_mode="DIJKSTRA")
    request = _request(env, start_region=0, dest_region=5)

    first, second = core.plan_routes([request, request])

    assert first.diagnostics["cache_hit"] is False
    assert second.diagnostics["cache_hit"] is True
