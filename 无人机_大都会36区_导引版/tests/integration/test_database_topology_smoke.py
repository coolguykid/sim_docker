from tests.fixtures.database_topology_rows import (
    LANE_ROWS,
    LAYER_ROWS,
    PIPE_ROWS,
    RAMP_ROWS,
    REGION_ROWS,
)
from uav_sim.airspace import Airspace, DatabaseTopologyAdapter


def test_database_topology_routes_and_closes_only_one_ramp_direction():
    topology = DatabaseTopologyAdapter(strict=True).adapt(
        region_rows=REGION_ROWS,
        layer_rows=LAYER_ROWS,
        pipe_rows=PIPE_ROWS,
        lane_rows=LANE_ROWS,
        ramp_rows=RAMP_ROWS,
        topology_version=73,
    )
    env = Airspace.from_adapted_topology(topology)
    start_pipe = topology.id_maps.pipe_external_to_internal[101]
    destination = topology.id_maps.pipe_external_to_internal[450]
    forward = env.ramp_by_id[topology.id_maps.ramp_external_to_internal[9001]]
    reverse = env.ramp_by_id[topology.id_maps.ramp_external_to_internal[9007]]

    open_path = env.find_shortest_path(
        start_pipe,
        0.0,
        destination,
        env.pipelines[destination].length,
        uav_speed=20.0,
    )
    forward.is_active = False
    closed_path = env.find_shortest_path(
        start_pipe,
        0.0,
        destination,
        env.pipelines[destination].length,
        uav_speed=20.0,
    )

    assert open_path[-1] == "V_END"
    assert closed_path == []
    assert reverse.is_active is False
    forward.is_active = True
    assert env.find_shortest_path(
        start_pipe,
        0.0,
        destination,
        env.pipelines[destination].length,
        uav_speed=20.0,
    )[-1] == "V_END"
