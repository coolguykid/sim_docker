import numpy as np
import pytest

from tests.fixtures.database_topology_rows import (
    LANE_ROWS,
    LAYER_ROWS,
    PIPE_ROWS,
    RAMP_ROWS,
    REGION_ROWS,
)
from uav_sim.airspace.database_topology_adapter import DatabaseTopologyAdapter


def _adapt(strict=True):
    return DatabaseTopologyAdapter(strict=strict).adapt(
        region_rows=REGION_ROWS,
        layer_rows=LAYER_ROWS,
        pipe_rows=PIPE_ROWS,
        lane_rows=LANE_ROWS,
        ramp_rows=RAMP_ROWS,
        topology_version=73,
    )


def test_adapter_maps_sparse_external_ids_to_dense_internal_ids():
    topology = _adapt()

    assert [p.pipe_id for p in topology.pipelines] == list(range(len(PIPE_ROWS)))
    assert topology.id_maps.pipe_external_to_internal[101] == 0
    assert topology.id_maps.pipe_external_to_internal[309] == 2
    assert topology.id_maps.ramp_external_to_internal == {9001: 0, 9007: 1}
    assert topology.ramps[0].external_ramp_id == 9001
    assert topology.ramps[0].source_pipe_id == 0
    assert topology.ramps[0].target_pipe_id == 3
    assert topology.ramps[0].is_bidirectional is False
    assert topology.ramps[1].is_active is False
    assert topology.topology_version == 73


def test_adapter_recomputes_and_validates_ramp_length():
    topology = _adapt()
    ramp = topology.ramps[0]

    assert ramp.length == pytest.approx(
        np.linalg.norm(np.array(ramp.pos_b) - np.array(ramp.pos_a))
    )


def test_adapter_preserves_database_pipe_and_lane_metadata():
    topology = _adapt()
    pipe = topology.pipelines[0]

    assert pipe.external_pipe_id == 101
    assert pipe.external_layer_id == 11
    assert pipe.external_start_node_id == 10001
    assert pipe.external_end_node_id == 10002
    assert pipe.d_safe == 8.0
    assert pipe.max_capacity == 40
    assert [lane.external_lane_id for lane in pipe.lanes] == [7001, 7002]


def test_adapter_builds_directed_continue_edges_from_shared_nodes():
    topology = _adapt()

    assert (0, 1, 10002, "CONTINUE") in topology.continuation_edges
    assert (1, 2, 10003, "CONTINUE") in topology.continuation_edges
    assert (1, 0, 10002, "CONTINUE") not in topology.continuation_edges


def test_adapter_rejects_ramp_without_source_or_target_pipe_id():
    bad_ramps = [dict(RAMP_ROWS[0])]
    bad_ramps[0].pop("source_pipe_id")

    with pytest.raises(ValueError, match="入口管道"):
        DatabaseTopologyAdapter(strict=True).adapt(
            region_rows=REGION_ROWS,
            layer_rows=LAYER_ROWS,
            pipe_rows=PIPE_ROWS,
            lane_rows=LANE_ROWS,
            ramp_rows=bad_ramps,
        )
