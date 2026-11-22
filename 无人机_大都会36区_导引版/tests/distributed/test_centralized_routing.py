from uav_sim.airspace import Airspace
from uav_sim.airspace.registry import AirspaceRegistryBase
from uav_sim.distributed.messages import RouteRequest
from uav_sim.distributed.local_env_proxy import LocalEnvProxy
from uav_sim.distributed.route_planning_server import RoutePlanningCore
from uav_sim.distributed.uav_actor import UAVGroupActorBase
from uav_sim.app import _estimate_uav_actor_num_cpus


def test_route_planning_core_matches_local_dijkstra():
    env = Airspace(mode="standard", include_dynamic=False)
    snapshot = env.get_readonly_snapshot()
    local = LocalEnvProxy(snapshot, routing_mode="DIJKSTRA")
    core = RoutePlanningCore(snapshot, routing_mode="DIJKSTRA")
    request = RouteRequest(
        uav_id=7,
        current_pipe_id=0,
        pos_1d=0.0,
        dest_pipe_id=5,
        dest_pos=env.pipelines[5].length,
        phase="TO_DEST",
        spawn_turn=0,
        current_turn=0,
    )
    expected = local.find_shortest_path(
        request.current_pipe_id, request.pos_1d, request.dest_pipe_id, request.dest_pos
    )
    updates = core.plan_routes([request])
    assert len(updates) == 1
    assert updates[0].uav_id == request.uav_id
    assert updates[0].raw_path == expected
    assert updates[0].dest_pos == request.dest_pos


def test_route_planning_core_keeps_avoidance_local_by_default():
    env = Airspace(mode="standard", include_dynamic=False)
    core = RoutePlanningCore(env.get_readonly_snapshot(), routing_mode="DIJKSTRA")
    assert core.routing_mode == "DIJKSTRA"


def test_centralized_actor_does_not_load_router_and_applies_route_update():
    env = Airspace(mode="standard", include_dynamic=False)
    actor = UAVGroupActorBase(
        group_id=0,
        uav_configs=[
            {
                "uav_id": 0,
                "start_pipe_id": 0,
                "lane_id": 0,
                "dest_pipe_id": 5,
                "capacity": 0,
                "spawn_turn": 0,
                "final_dest_pipe_id": 5,
                "pickup_info": None,
            }
        ],
        env_snapshot_ref=env.get_readonly_snapshot(),
        routing_mode="DIJKSTRA",
        control_mode="centralized_route",
    )
    assert actor.local_env.routing_mode == "DIJKSTRA"
    requests = actor.build_route_requests(turn=0, force_all=True)
    updates = RoutePlanningCore(env.get_readonly_snapshot()).plan_routes(requests)
    assert actor.apply_route_updates(updates) == 1
    assert actor.uavs[0].path
    assert actor.uavs[0].path[-1] == "DEST_0_TO_DEST"


def test_registry_geo_fenced_query_returns_only_nearby_pipe_traffic():
    registry = AirspaceRegistryBase(num_pipelines=3)
    registry.batch_register_uavs(
        {
            1: {
                "pipe_id": 0,
                "lane_id": 0,
                "pos_1d": 100.0,
                "speed": 30.0,
                "state": "FLYING",
            },
            2: {
                "pipe_id": 0,
                "lane_id": 0,
                "pos_1d": 5000.0,
                "speed": 30.0,
                "state": "FLYING",
            },
            3: {
                "pipe_id": 1,
                "lane_id": 0,
                "pos_1d": 120.0,
                "speed": 30.0,
                "state": "FLYING",
            },
        }
    )
    result = registry.get_geo_fenced_pipe_uavs_map([(0, 0.0, 2000.0)])
    assert set(result) == {0}
    assert [item["uav_id"] for item in result[0]] == [1]


def test_actor_cpu_reservation_fits_many_groups_on_local_ray():
    actor_cpu = _estimate_uav_actor_num_cpus(
        num_uavs=1000, parallelism_level=10, total_cpus=16, service_actor_cpus=3.0
    )
    assert actor_cpu < 1.0
    assert actor_cpu * 100 + 3.0 + 1.0 <= 16.0
