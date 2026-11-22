"""Exact manual lane goals must be executable without a lane snap."""
from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from uav_sim.airspace.environment import Pipeline
from uav_sim.airspace.partition_graph import compile_partition, PartitionMaps
from uav_sim.distributed.route_planning_server import PartitionRoutePlanningCore
from uav_sim.entities.partition_motion import RoadState, advance, node_position
from uav_sim.entities.trajectory_profile import build_prefix_profile


def make(length=1000., distance=1000., lane=0):
    pipe = Pipeline(0, (100., 100., 100.), (100.+length, 100., 100.), 100, 19, num_lanes=3)
    env = SimpleNamespace(pipelines=[pipe], ramps=[], x_min=0., x_max=30000.,
                          y_min=0., y_max=30000., topology_version=0, continuation_edges=[])
    spec = dict(node_id='goal', pipe_id=0, lane_id=lane, distance_m=distance)
    graph = compile_partition(env, [spec])
    maps = PartitionMaps(graph)
    goal = maps.destination('goal')
    route = PartitionRoutePlanningCore(maps, full_route=True).plan_many(
        {0: dict(start_node_id='P0_START', region_id=0, goal_node_id='goal')},
        dict(step_index=0, sim_time_seconds=0., sim_dt_seconds=.1, topology_version=0))[0]
    assert route.status == 'OK'
    state = RoadState(0, 'P0_START', 0, 2, node_position(graph, 'P0_START', 2), goal,
                      route=route.edge_ids, visited_regions=(0,))
    return env, graph, maps, state


def test_same_pipe_manual_endpoint_reaches_requested_lane_via_continuous_approach():
    env, graph, maps, state = make()
    profile = build_prefix_profile(state, graph, stop_at_boundaries=True)
    assert profile.end_state.status == 'ARRIVED'
    assert profile.end_state.lane_id == 0
    assert profile.end_state.position == pytest.approx(tuple(env.pipelines[0].lanes[0].end_pt))
    approach = [graph.edges[e] for e in state.route if graph.edges[e].kind == 'GOAL_APPROACH']
    assert approach and approach[-1].target == 'goal'
    assert any(abs(p.velocity0[1]) > 0 or abs(p.acceleration[1]) > 0 for p in profile.pieces)
    for a, b in zip(profile.pieces, profile.pieces[1:]):
        assert a.position_at(a.t1) == pytest.approx(b.position0, abs=1e-8)
        assert a.velocity_at(a.t1) == pytest.approx(b.velocity0, abs=1e-8)


def test_per_step_motion_records_goal_lane_only_after_flying_actual_geometry():
    _, graph, _, state = make(length=100., distance=100.)
    proposed = advance(state, graph, 10., 0., 100.)
    assert proposed.state.status == 'ARRIVED' and proposed.state.lane_id == 0
    assert proposed.state.position == pytest.approx(state.goal.position)
    assert proposed.state.arrival_time > math.sqrt(2*100/10)


def test_near_entry_and_entry_plane_goals_do_not_teleport_across_lanes():
    for distance in (0., 1e-5, 1.):
        _, graph, _, state = make(distance=distance)
        profile = build_prefix_profile(state, graph, stop_at_boundaries=True)
        assert profile.duration > 0
        assert profile.pieces[0].position0 == pytest.approx(state.position)
        assert profile.end_state.position == pytest.approx(state.goal.position)
        assert profile.end_state.lane_id == 0


def test_cross_region_approach_retains_offset_until_actual_terminal_lane_update():
    _, graph, _, state = make(length=6000., distance=5000.)
    profile = build_prefix_profile(state, graph, stop_at_boundaries=True)
    assert profile.end_state.position == pytest.approx(state.goal.position)
    assert profile.end_state.lane_id == 0
    assert any(s.region_id == 1 for _, s in profile.region_events)
    for a, b in zip(profile.pieces, profile.pieces[1:]):
        assert a.position_at(a.t1) == pytest.approx(b.position0, abs=1e-8)


def test_goal_terminal_can_depart_again_after_pickup_without_forcing_other_traffic_to_its_lane():
    env, _, _, _ = make()
    specs = [dict(node_id='pickup', pipe_id=0, lane_id=0, distance_m=500.),
             dict(node_id='final', pipe_id=0, lane_id=2, distance_m=900.)]
    graph = compile_partition(env, specs)
    maps = PartitionMaps(graph)
    first = maps.destination('pickup')
    second = maps.destination('final')
    state = RoadState(0, 'pickup', graph.nodes['pickup'].region_id, 0, first.position, second)
    plan = PartitionRoutePlanningCore(maps, full_route=True).plan_many(
        {0: dict(start_node_id='pickup', region_id=state.region_id, goal_node_id='final')},
        dict(step_index=0, sim_time_seconds=0., sim_dt_seconds=1., topology_version=0))[0]
    assert plan.status == 'OK'
    profile = build_prefix_profile(replace(state, route=plan.edge_ids), graph)
    assert profile.end_state.lane_id == 2
    assert profile.end_state.position == pytest.approx(second.position)
    fly = [e for e in graph.edges.values() if e.kind == 'FLY' and e.pipe_id == 0]
    assert sum(e.length_m for e in fly) == pytest.approx(env.pipelines[0].length)
    assert all(e.offsets_start == e.offsets_end for e in fly)


@pytest.mark.parametrize('mode', ['centralized_full', 'centralized_regional'])
@pytest.mark.parametrize('dt', [1., .1])
def test_manual_single_pipe_three_launch_lanes_run_to_one_actual_target(mode, dt):
    from uav_sim.simulation.metropolis_runtime import MetropolisSession
    env, graph, _, _ = make()
    config = dict(num_uavs=3, max_batch_size=3, mission_limit=3,
                  max_sim_time_seconds=1, drain_timeout_seconds=1000,
                  sim_dt_seconds=dt, routing_mode=mode, destination_mode='manual',
                  destination_count=1, manual_destinations=[dict(node_id='goal', pipe_id=0,
                  lane_id=0, distance_m=1000.)])
    with MetropolisSession(env, config, graph=graph) as session:
        result = session.run()
        assert result['success'] and result['arrived'] == 3
        assert result['safety_violations'] == 0 and result['unfinished'] == 0
        expected = tuple(env.pipelines[0].lanes[0].end_pt)
        assert all(r['position_m'] == pytest.approx(expected) for r in session.completed)
