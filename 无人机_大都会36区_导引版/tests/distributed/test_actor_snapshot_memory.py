import pickle

from uav_sim.airspace import Airspace
import uav_sim.app as app_module
from uav_sim.distributed.local_env_proxy import LocalEnvProxy


def _pickle_size_mb(value):
    return len(pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)) / 1024 / 1024


def test_centralized_route_actor_snapshot_omits_routing_graph():
    env = Airspace(mode="regional_6x6", include_dynamic=True)

    assert hasattr(app_module, "_snapshot_for_uav_group_actors")
    snapshot = app_module._snapshot_for_uav_group_actors(env, control_mode="centralized_route")

    assert snapshot["graph"] == {}
    assert snapshot["node_info"] == {}
    assert snapshot["pipe_nodes"] == {}
    assert len(snapshot["pipelines"]) == len(env.pipelines)
    assert len(snapshot["ramps"]) == len(env.ramps)
    # Directed v2 contains 14,040 physical ramps instead of the former 9,720;
    # the compact slotted snapshot must still remain comfortably below 2.5 MiB.
    assert _pickle_size_mb(snapshot) < 2.5


def test_server_authoritative_snapshot_does_not_rebuild_lane_objects():
    env = Airspace(mode="regional_6x6", include_dynamic=True)
    snapshot = app_module._snapshot_for_uav_group_actors(
        env, control_mode="server_authoritative")
    restored = pickle.loads(pickle.dumps(snapshot, protocol=pickle.HIGHEST_PROTOCOL))
    proxy = LocalEnvProxy(restored)

    assert all(not hasattr(pipe, "lanes") for pipe in proxy.pipelines)
    assert all(not hasattr(pipe, "__dict__") for pipe in proxy.pipelines)
    assert all(not hasattr(ramp, "__dict__") for ramp in proxy.ramps)
    assert all(hasattr(pipe, "num_lanes") for pipe in proxy.pipelines)
    assert all(hasattr(ramp, "is_dynamic") for ramp in proxy.ramps)


def test_actor_snapshot_keeps_directed_ramp_and_database_fields():
    env = Airspace(
        mode="standard",
        include_dynamic=False,
        topology_profile="balanced_sparse_v1",
    )
    snapshot = app_module._snapshot_for_uav_group_actors(
        env, control_mode="server_authoritative")
    ramp = snapshot["ramps"][0]
    pipe = snapshot["pipelines"][0]

    assert ramp.source_pipe_id >= 0
    assert ramp.target_pipe_id >= 0
    assert ramp.length > 0.0
    assert ramp.time_penalty >= 0.0
    assert ramp.is_bidirectional in (True, False)
    assert ramp.external_ramp_id is not None
    assert hasattr(ramp, "pair_id")
    assert hasattr(pipe, "d_safe")
    assert pipe.external_pipe_id is not None
    assert hasattr(pipe, "external_start_node_id")
    assert hasattr(pipe, "external_end_node_id")
