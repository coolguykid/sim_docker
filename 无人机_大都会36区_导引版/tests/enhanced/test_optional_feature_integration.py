from uav_sim.airspace.environment import Airspace
from uav_sim.simulation.metropolis_runtime import MetropolisSession


def test_original_optional_pickup_trace_metrics_run_through_actual_session(tmp_path):
    env = Airspace(mode="metropolis", num_pipes_per_layer=2, include_dynamic=False)
    pipe = env.pipelines[4]
    cfg = dict(
        num_uavs=1,
        max_batch_size=1,
        mission_limit=1,
        max_sim_time_seconds=1,
        drain_timeout_seconds=10000,
        sim_dt_seconds=1.0,
        routing_mode="centralized_full",
        num_pickup_points=1,
        pickup_locations=[dict(pipe_id=4, pos_1d=pipe.length * 0.5)],
        capacity_weights={2: 100},
        trace_uav_ids=[0],
        trace_log_path=str(tmp_path / "trace.txt"),
        enable_kl_divergence=True,
        enable_channel_gain=True,
    )
    with MetropolisSession(env, cfg) as session:
        result = session.run()
        assert result["success"] and result["arrived"] == 1
        assert session.features.pickup_points[0].remaining == 0
        assert len(session.features.kl_history) > 0
        assert len(session.features.gain_history) == len(session.features.kl_history)
    lines = (tmp_path / "trace.txt").read_text().splitlines()
    assert any("TO_PICKUP" in line for line in lines)
    assert any("TO_DEST" in line for line in lines)
