import pytest
from tests.enhanced.test_uav_partition_motion import fixture
from uav_sim.simulation.metropolis_runtime import MetropolisSession


@pytest.mark.parametrize("dt", [1.0, 0.1])
@pytest.mark.parametrize("mode", ["centralized_full", "centralized_regional"])
def test_original_runtime_flow_reaches_exact_goal_and_conserves_population(dt, mode):
    env, graph, _ = fixture()
    from uav_sim.airspace.environment import Pipeline

    env.pipelines[1] = Pipeline(1, (10.5, 0, 100), (40, 0, 100), 100, 20, num_lanes=1)
    graph.metadata.update(
        pipe_endpoints={0: {"start": "a", "end": "b"}, 1: {"start": "e", "end": "f"}},
        pipes={
            0: dict(height=100, length_m=10, lane_count=1, speed_limit_mps=20),
            1: dict(height=100, length_m=19.5, lane_count=1, speed_limit_mps=20),
        },
        goal_specs={"f": dict(pipe_id=1, lane_id=0, distance_m=19.5)},
    )
    config = dict(
        sim_dt_seconds=dt,
        routing_mode=mode,
        num_uavs=2,
        max_batch_size=2,
        mission_limit=2,
        replenishment_period_seconds=100,
        max_sim_time_seconds=10,
        drain_timeout_seconds=30,
        destination_count=1,
        destination_mode="manual",
        manual_destinations=[dict(node_id="f", pipe_id=1, lane_id=0, distance_m=19.5)],
    )
    with MetropolisSession(env, config, graph=graph) as session:
        result = session.run()
        assert result["generated"] == result["departed"] == result["arrived"] == 2
        assert result["active"] == result["pending"] == result["no_route"] == 0
        assert (
            result["success"]
            and result["safety_violations"] == result["hard_collisions"] == 0
        )
        assert all(
            record["actual_goal_node_id"] == "f"
            and record["goal_position_error_m"] < 1e-6
            for record in session.completed
        )
