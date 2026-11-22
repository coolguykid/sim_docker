import pytest
from uav_sim.airspace import Airspace
from uav_sim.distributed.messages import RouteRequest
from uav_sim.distributed.route_planning_server import RoutePlanningCore


def _request(uav_id, start_pipe_id, dest_pipe_id, dest_pos, speed=20.0):
    return RouteRequest(
        uav_id=uav_id,
        current_pipe_id=start_pipe_id,
        pos_1d=0.0,
        dest_pipe_id=dest_pipe_id,
        dest_pos=dest_pos,
        phase="TO_DEST",
        spawn_turn=0,
        current_turn=0,
        speed=speed,
    )


def test_route_batch_plans_identical_dijkstra_query_once(monkeypatch):
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    core = RoutePlanningCore(env.get_readonly_snapshot(), routing_mode="DIJKSTRA")
    start_pipe_id = next(
        (pipe.pipe_id for pipe in env.pipelines if pipe.region_id == 0)
    )
    dest_pipe = next(
        (pipe for pipe in env.pipelines if pipe.region_id == 5 and pipe.height == 600)
    )
    requests = [
        _request(1, start_pipe_id, dest_pipe.pipe_id, dest_pipe.length),
        _request(2, start_pipe_id, dest_pipe.pipe_id, dest_pipe.length),
    ]
    calls = 0
    original = core._plan_one

    def counted(request):
        nonlocal calls
        calls += 1
        return original(request)

    monkeypatch.setattr(core, "_plan_one", counted)
    updates = core.plan_routes(
        requests,
        {
            "ramp_states": {ramp.ramp_id: True for ramp in env.ramps},
            "ramp_state_version": 7,
            "traffic_cost_version": 0,
        },
    )
    assert calls == 1
    assert [update.uav_id for update in updates] == [1, 2]
    assert updates[0].raw_path == updates[1].raw_path
    assert updates[0].raw_path[-1] == "V_END"


def test_route_batch_does_not_coalesce_different_speed_buckets(monkeypatch):
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    core = RoutePlanningCore(env.get_readonly_snapshot(), routing_mode="DIJKSTRA")
    start_pipe_id = next(
        (pipe.pipe_id for pipe in env.pipelines if pipe.region_id == 0)
    )
    dest_pipe = next(
        (pipe for pipe in env.pipelines if pipe.region_id == 5 and pipe.height == 600)
    )
    requests = [
        _request(1, start_pipe_id, dest_pipe.pipe_id, dest_pipe.length, speed=10.0),
        _request(2, start_pipe_id, dest_pipe.pipe_id, dest_pipe.length, speed=20.0),
    ]
    calls = 0
    original = core._plan_one

    def counted(request):
        nonlocal calls
        calls += 1
        return original(request)

    monkeypatch.setattr(core, "_plan_one", counted)
    core.plan_routes(requests, {})
    assert calls == 2


def test_regional_dijkstra_failure_never_falls_back_to_global_graph(monkeypatch):
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    core = RoutePlanningCore(env.get_readonly_snapshot(), routing_mode="DIJKSTRA")
    start_pipe = next((pipe for pipe in env.pipelines if pipe.region_id == 0))
    dest_pipe = next((pipe for pipe in env.pipelines if pipe.region_id == 35))
    request = _request(1, start_pipe.pipe_id, dest_pipe.pipe_id, dest_pipe.length)
    monkeypatch.setattr(
        core.regional_router,
        "find_path",
        lambda *_args, **_kwargs: (
            [],
            {},
            {
                "router": "hierarchical_regional",
                "fallback_used": False,
                "route_failed": True,
            },
        ),
    )
    monkeypatch.setattr(
        core.env,
        "find_shortest_path",
        lambda *_args, **_kwargs: pytest.fail(
            "regional DIJKSTRA must not call the global fallback"
        ),
    )
    update = core.plan_routes(
        [request],
        {
            "ramp_states": {ramp.ramp_id: True for ramp in env.ramps},
            "ramp_state_version": 1,
            "traffic_cost_version": 0,
        },
    )[0]
    assert update.raw_path == []
    assert update.diagnostics["route_failed"] is True
