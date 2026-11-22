from tests.fixtures.database_topology_rows import (
    LANE_ROWS,
    LAYER_ROWS,
    PIPE_ROWS,
    RAMP_ROWS,
    REGION_ROWS,
)
from uav_sim.airspace import Airspace, DatabaseTopologyAdapter


def _adapted_topology():
    return DatabaseTopologyAdapter(strict=True).adapt(
        region_rows=REGION_ROWS,
        layer_rows=LAYER_ROWS,
        pipe_rows=PIPE_ROWS,
        lane_rows=LANE_ROWS,
        ramp_rows=RAMP_ROWS,
        topology_version=73,
    )


def test_database_subpipes_connect_without_entering_ramp_state():
    adapted = _adapted_topology()
    env = Airspace.from_adapted_topology(adapted)

    path = env.find_shortest_path(
        adapted.id_maps.pipe_external_to_internal[101],
        0.0,
        adapted.id_maps.pipe_external_to_internal[309],
        400.0,
        uav_speed=20.0,
    )

    assert path[0] == "V_START"
    assert path[-1] == "V_END"
    assert env.path_transition_types(path).count("CONTINUE") == 2


def test_database_airspace_keeps_external_maps_and_topology_version_in_snapshot():
    adapted = _adapted_topology()
    env = Airspace.from_adapted_topology(adapted)

    snapshot = env.get_readonly_snapshot()

    assert snapshot["topology_source"] == "postgres"
    assert snapshot["topology_version"] == 73
    assert snapshot["external_id_maps"] is adapted.id_maps
    assert snapshot["continuation_edges"] == adapted.continuation_edges
