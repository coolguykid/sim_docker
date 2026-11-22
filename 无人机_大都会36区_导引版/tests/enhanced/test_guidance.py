import json
import math

import numpy as np
import pytest

from uav_sim.airspace.partition_contracts import PartitionGraph, RoadEdge, RoadNode
from uav_sim.airspace.partition_graph import PartitionMaps
from uav_sim.distributed.regional_backend import RegionalRoutingBackend
from uav_sim.distributed.route_planning_server import PartitionRoutePlanningCore


def diamond():
    regions = {i: dict(x_min=x, x_max=x+1, y_min=y, y_max=y+1)
               for i, (x, y) in enumerate(((0, 0), (1, -1), (1, 1), (2, 0)))}
    nodes = {str(i): RoadNode(str(i), i, (r['x_min'], r['y_min'], 100.))
             for i, r in regions.items()}
    edges = {}
    for a, b, length in ((0, 1, 1), (0, 2, 2), (1, 3, 1), (2, 3, 2)):
        name = f'{a}-{b}'
        edges[name] = RoadEdge(name, str(a), str(b), a, 'BOUNDARY', length,
                               1., (nodes[str(a)].position, nodes[str(b)].position))
    return PartitionGraph(nodes, edges, regions)


def test_cost_snapshots_change_both_levels_and_isolate_caches():
    maps = PartitionMaps(diamond())
    request = {0: dict(start_node_id='0', region_id=0, goal_node_id='3')}
    context = dict(step_index=0, topology_version=0, sim_time_seconds=0., sim_dt_seconds=1.)
    weighted = maps.with_edge_penalties({'0-1': 5.}, 1)
    assert weighted.graph is maps.graph
    for mode in RegionalRoutingBackend.MODES:
        with RegionalRoutingBackend(maps, mode, use_ray=False, target_uavs=1) as backend:
            base = backend.plan_many(request, context)[0]
            guided = backend.plan_many(request, context, maps=weighted)[0]
            old = backend.plan_many(request, context)[0]
            assert base.edge_ids[0] == '0-1'
            assert guided.edge_ids[0] == '0-2'
            assert (base.cost, guided.cost) == (2., 4.)
            assert old == base


def test_equal_cosine_splits_intentions_and_optimizer_respects_transfer_feedback():
    from uav_sim.guidance import GuidanceConfig, GuidanceModel, GuidanceNetwork

    network = GuidanceNetwork(diamond(), capacity_scale=1.)
    network.capacities[:] = 100.
    config = GuidanceConfig(enabled=True, gamma=.1, density_limit=.9,
                            intention_weight=10., rollout_steps=1)
    model = GuidanceModel(network, config)
    network_index = network.index
    counts, q = network.intentions([((0, 100.), 3)] * 40 + [((1, 100.), 1)] * 70)
    a, b, c = (network_index[(i, 100.)] for i in range(3))
    assert math.isclose(q[a, 1], .5) and math.isclose(q[a, 2], .5)
    policy, stats = model.evaluate(counts, q, transfer_feedback=lambda _: {((0, 100.), (1, 100.)): .1})
    assert stats['triggered'] and stats['status'] == 'published'
    assert policy[a, b] <= config.sample_interval_seconds * .1 / counts[a] + 1.e-6
    assert policy[a, c] > policy[a, b]
    assert np.allclose(policy.sum(axis=1), 1.)
    assert not policy[~network.legal].any()


def test_actual_probability_is_mass_conserving_for_single_and_multiple_choices():
    from uav_sim.guidance import actual_probabilities

    p = np.array([[.1, .3, .6], [0., 1., 0.], [.7, 0., .3]])
    legal = p > 0
    actual = actual_probabilities(p, legal, .3)
    np.testing.assert_allclose(actual[0], [.205, .315, .48])
    np.testing.assert_allclose(actual[1], [0., 1., 0.])
    assert np.allclose(actual.sum(axis=1), 1.)
    assert not actual[~legal].any()


def test_cli_guidance_overrides_json(tmp_path, monkeypatch):
    from uav_sim import cli

    path = tmp_path / 'run.json'
    path.write_text(json.dumps({'guidance': {'enabled': False, 'gamma': .6}}))
    result = []
    monkeypatch.setattr('uav_sim.app.run_config', result.append)
    cli.main(['--config', str(path), '--guidance', '--guidance-deviation-rate', '.2'])
    assert result[0]['guidance'] == {'enabled': True, 'gamma': .6, 'deviation_rate': .2}


def test_guided_local_cost_and_goal_ingress_use_same_snapshot():
    graph = diamond()
    graph.nodes['g'] = RoadNode('g', 3, (2., 1., 200.))
    graph.edges['up'] = RoadEdge('up', '3', 'g', 3, 'RAMP', 100., 10.,
                                 (graph.nodes['3'].position, graph.nodes['g'].position))
    maps = PartitionMaps(graph)
    weighted = maps.with_edge_penalties({'up': 5.}, 1)
    core = PartitionRoutePlanningCore(maps, full_route=True)
    request = {0: dict(start_node_id='0', region_id=0, goal_node_id='g')}
    context = dict(step_index=0, topology_version=0, sim_time_seconds=0., sim_dt_seconds=1.)
    assert core.plan_many(request, context)[0].cost == 12.
    assert core.plan_many(request, context, maps=weighted)[0].cost == 17.


def test_infeasible_update_retains_previous_policy_and_cost_revision():
    from uav_sim.guidance import GuidanceConfig, GuidanceModel, GuidanceNetwork

    network = GuidanceNetwork(diamond(), capacity_scale=1.)
    network.capacities[:] = 100.
    config = GuidanceConfig(enabled=True, gamma=.1, density_limit=.9,
                            intention_weight=10., rollout_steps=1)
    model = GuidanceModel(network, config)
    observations = [((0, 100.), 3)] * 40 + [((1, 100.), 1)] * 70
    first, stats = model.evaluate(*network.intentions(observations))
    assert stats["status"] == "published"
    costs = dict(model.edge_penalties)
    second, stats = model.evaluate(*network.intentions(observations + [((1, 100.), 1)] * 25))
    assert stats["triggered"]
    assert stats["status"] == "infeasible"
    assert model.revision == 1
    assert model.edge_penalties == costs
    np.testing.assert_array_equal(first, second)


@pytest.mark.parametrize("dt", [1., .1])
def test_live_policy_updates_only_change_new_journeys_at_exact_sample_times(dt):
    from uav_sim.airspace.environment import Airspace
    from uav_sim.simulation.metropolis_runtime import MetropolisSession

    env = Airspace(mode="metropolis", num_pipes_per_layer=4, num_lanes_per_pipe=4,
                   include_dynamic=False)
    config = dict(num_uavs=8, mission_limit=8, max_batch_size=2, max_sim_time_seconds=160,
                  drain_timeout_seconds=9000, replenishment_period_seconds=40, destination_count=3,
                  routing_mode="centralized_regional", sim_dt_seconds=dt,
                  guidance=dict(enabled=True, capacity_scale=.015, gamma=.04, rollout_steps=2,
                                sample_interval_seconds=20, edge_penalty_seconds=60.))
    with MetropolisSession(env, config) as session:
        samples = []
        session.guidance.on_sample = lambda sample: samples.append(
            (sample["time_seconds"], sample["revision"]))
        summary = session.run()
        records = {record["uav_id"]: record for record in session.completed}
        assert summary["success"] and summary["arrived"] == 8
        assert summary["safety_violations"] == 0
        assert summary["guidance"]["guided_missions"] > 0
        assert summary["journey_routing"]["verified_regional_entry_plans"] > 0
        revision_at = dict(samples)
        assert list(revision_at) == list(range(0, 20 * len(samples), 20))
        assert records[0]["guidance_cost_revision"] == records[1]["guidance_cost_revision"] == 0
        for uid in range(2, 8):
            assert records[uid]["guidance_cost_revision"] == revision_at[40 * (uid // 2)]
        assert not session._mission_cost_maps
