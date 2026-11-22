from dataclasses import replace
from types import SimpleNamespace

from uav_sim.airspace.partition_graph import compile_partition, PartitionMaps
from uav_sim.airspace.environment import Airspace
from uav_sim.entities.uav import UAV


def test_default_retains_capacity_weights_but_has_no_intermediate_stop():
    from uav_sim.simulation.mission_features import MissionFeatures

    env = Airspace(mode="metropolis", num_pipes_per_layer=2, include_dynamic=False)
    features = MissionFeatures(env, dict(num_uavs=10, capacity_weights={0: 50, 2: 50}))
    assert features.additional_specs == ()
    assert sorted(features.capacity(i) for i in range(10)) == [0] * 5 + [2] * 5


def test_optional_pickup_is_real_exact_first_leg_and_then_final_goal():
    from uav_sim.simulation.mission_features import MissionFeatures

    env = Airspace(mode="metropolis", num_pipes_per_layer=2, include_dynamic=False)
    pipe = env.pipelines[2]
    features = MissionFeatures(
        env,
        dict(
            num_uavs=1,
            capacity_weights={2: 100},
            num_pickup_points=1,
            pickup_locations=[dict(pipe_id=2, pos_1d=pipe.length * 0.5)],
        ),
    )
    graph = compile_partition(
        env,
        features.additional_specs
        + (
            dict(
                node_id="FINAL",
                pipe_id=10,
                lane_id=0,
                distance_m=env.pipelines[10].length * 0.7,
            ),
        ),
    )
    maps = PartitionMaps(graph)
    final = maps.destination("FINAL")
    initial = features.initial_goal(0, final, maps)
    assert (
        initial.node_id.startswith("PICKUP_")
        and initial.distance_m == pipe.length * 0.5
    )
    u = UAV(0, 0, 0, final.pipe_id, env, capacity=features.capacity(0))
    u.configure_partition_motion(
        graph, graph.metadata["pipe_endpoints"][0]["start"], initial
    )
    features.configure(u, final)
    assert not features.is_final_leg(u)
    s = replace(
        u._road_state,
        node_id=initial.node_id,
        position=initial.position,
        region_id=initial.region_id,
        speed=0.0,
        status="ARRIVED",
        arrival_time=4.0,
    )
    u._road_state = s
    assert features.on_goal(u, s, 4.0) is False
    assert u._road_state.goal.node_id == "FINAL" and u._road_state.arrival_time is None
    assert u._road_state.status == "FLYING" and features.is_final_leg(u)
    assert features.pickup_points[0].remaining == 0
    assert (
        features.on_goal(
            u, replace(s, goal=final, node_id="FINAL", position=final.position), 10.0
        )
        is True
    )


def test_enabled_trace_and_metric_controls_produce_actual_output(tmp_path):
    from uav_sim.simulation.mission_features import MissionFeatures

    env = Airspace(mode="metropolis", num_pipes_per_layer=2, include_dynamic=False)
    features = MissionFeatures(
        env,
        dict(
            trace_uav_ids=[0],
            trace_log_path=str(tmp_path / "trace.txt"),
            enable_kl_divergence=True,
            enable_channel_gain=True,
        ),
    )
    session = SimpleNamespace(
        active={},
        pending_uavs={},
        world_objects={},
        clock=SimpleNamespace(time_seconds=1.0, step_index=1),
        routing_mode="centralized_full",
    )
    features.record_grid(session)
    features.close()
    assert features.kl_history == [0.0]
    assert len(features.gain_history) == 1 and features.gain_history[0] > 0
    assert (tmp_path / "trace.txt").read_text().startswith("algo_id")
