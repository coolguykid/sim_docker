import numpy as np
import pytest

from uav_sim.airspace.environment import Pipeline, Ramp


def _pipe(pipe_id, start, end, height):
    return Pipeline(
        pipe_id=pipe_id,
        start_pt=start,
        end_pt=end,
        height=height,
        max_speed=50.0,
        num_lanes=1,
    )


def test_ramp_exposes_directed_source_and_target_pipe_ids():
    source = _pipe(7, [0.0, 0.0, 100.0], [100.0, 0.0, 100.0], 100.0)
    target = _pipe(9, [0.0, 100.0, 200.0], [100.0, 100.0, 200.0], 200.0)

    ramp = Ramp(
        3,
        source,
        target,
        pos_a_3d=[20.0, 0.0, 100.0],
        pos_b_3d=[40.0, 100.0, 200.0],
    )

    assert ramp.source_pipe_id == 7
    assert ramp.target_pipe_id == 9
    assert ramp.is_bidirectional is False


def test_ramp_precomputes_length_and_uses_it_for_speed_aware_travel_time():
    source = _pipe(1, [0.0, 0.0, 100.0], [100.0, 0.0, 100.0], 100.0)
    target = _pipe(2, [0.0, 40.0, 130.0], [100.0, 40.0, 130.0], 130.0)

    ramp = Ramp(
        4,
        source,
        target,
        pos_a_3d=[10.0, 0.0, 100.0],
        pos_b_3d=[10.0, 40.0, 130.0],
    )

    assert ramp.length == pytest.approx(50.0)
    assert ramp.travel_time_seconds(25.0) == pytest.approx(2.0)
    assert ramp.travel_time_seconds(0.0) == pytest.approx(50.0)
    assert np.isfinite(ramp.travel_time_seconds(0.0))


def test_ramp_keeps_external_and_pair_identifiers():
    source = _pipe(1, [0.0, 0.0, 100.0], [100.0, 0.0, 100.0], 100.0)
    target = _pipe(2, [0.0, 40.0, 130.0], [100.0, 40.0, 130.0], 130.0)

    ramp = Ramp(
        4,
        source,
        target,
        pos_a_3d=[10.0, 0.0, 100.0],
        pos_b_3d=[10.0, 40.0, 130.0],
        external_ramp_id=9001,
        pair_id="layer-link-12",
    )

    assert ramp.external_ramp_id == 9001
    assert ramp.pair_id == "layer-link-12"


def test_ramp_defaults_external_id_and_nominal_penalty_from_geometry():
    source = _pipe(1, [0.0, 0.0, 100.0], [100.0, 0.0, 100.0], 100.0)
    target = _pipe(2, [0.0, 40.0, 130.0], [100.0, 40.0, 130.0], 130.0)

    ramp = Ramp(
        17,
        source,
        target,
        pos_a_3d=[10.0, 0.0, 100.0],
        pos_b_3d=[10.0, 40.0, 130.0],
        time_penalty=None,
    )

    assert ramp.external_ramp_id == 17
    assert ramp.time_penalty == pytest.approx(ramp.length / 50.0)
