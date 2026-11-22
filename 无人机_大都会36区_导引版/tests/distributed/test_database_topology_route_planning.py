from tests.fixtures.database_topology_rows import (
    LANE_ROWS,
    LAYER_ROWS,
    PIPE_ROWS,
    RAMP_ROWS,
    REGION_ROWS,
)
from uav_sim.airspace import Airspace, DatabaseTopologyAdapter
from uav_sim.distributed.messages import RouteRequest
from uav_sim.distributed.route_planning_server import RoutePlanningCore


def test_route_planning_core_routes_across_database_subpipes():
    topology = DatabaseTopologyAdapter(strict=True).adapt(
        region_rows=REGION_ROWS,
        layer_rows=LAYER_ROWS,
        pipe_rows=PIPE_ROWS,
        lane_rows=LANE_ROWS,
        ramp_rows=RAMP_ROWS,
        topology_version=73,
    )
    env = Airspace.from_adapted_topology(topology)
    core = RoutePlanningCore(env.get_readonly_snapshot(), routing_mode="DIJKSTRA")
    start_pipe_id = topology.id_maps.pipe_external_to_internal[101]
    dest_pipe_id = topology.id_maps.pipe_external_to_internal[309]

    assert core.use_hierarchical_regional is True

    update = core.plan_routes([RouteRequest(
        uav_id=1,
        current_pipe_id=start_pipe_id,
        pos_1d=0.0,
        dest_pipe_id=dest_pipe_id,
        dest_pos=env.pipelines[dest_pipe_id].length,
        phase="TO_DEST",
        spawn_turn=0,
        current_turn=0,
        speed=20.0,
    )], {})[0]

    assert update.raw_path[0] == "V_START"
    assert update.raw_path[-1] == "V_END"
