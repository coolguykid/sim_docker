"""Controller certificates use the real piecewise motion and 3D audit."""
from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from uav_sim.airspace.partition_contracts import (
    Destination, PartitionGraph, RoadEdge, RoadNode,
)
from uav_sim.algorithms.continuous_safety import audit_trajectories
from uav_sim.distributed.authoritative_control_server import AuthoritativeControlCore
from uav_sim.entities.partition_motion import RoadState, advance


def world(paths, caps=None, ramp_ids=None):
    nodes, edges = {}, {}
    for uid, points in paths.items():
        for index, point in enumerate(points):
            name = '%s:%s' % (uid, index)
            nodes[name] = RoadNode(name, 0, point, uid, point[0])
            if index:
                key = '%s/%s' % (uid, index)
                source = '%s:%s' % (uid, index-1)
                ramp = (ramp_ids or {}).get(uid)
                edges[key] = RoadEdge(key, source, name, 0,
                    'RAMP' if ramp is not None else 'FLY',
                    math.dist(points[index-1], point), (caps or {}).get((uid,index), 50.),
                    (points[index-1], point), pipe_id=uid, ramp_id=ramp, lane_count=1)
    return PartitionGraph(nodes, edges, {0:{}})


def aircraft(graph, uid, speed=0., status='FLYING', goal_last=True):
    route = tuple(k for k in graph.edges if k.startswith('%s/' % uid))
    start = graph.nodes['%s:0' % uid]
    target = graph.nodes[graph.edges[route[-1]].target]
    goal = Destination(target.node_id if goal_last else 'beyond', 0, {},
                       pipe_id=uid, position=target.position)
    state = RoadState(uid, start.node_id, 0, 0, start.position, goal,
                      speed=speed, status=status, route=route)
    return SimpleNamespace(uav_id=uid, _road_state=state, max_accel=10., max_decel=10.,
                           spawn_turn=0, layer=start.position[2])


def core(graph, **kwargs):
    factory = getattr(AuthoritativeControlCore, 'for_partition', None)
    assert callable(factory), 'Existing AuthoritativeControlCore needs partition factory'
    instance = factory(graph, **kwargs)
    assert isinstance(instance, AuthoritativeControlCore)
    return instance


def assert_current_safe(result, distance=8.):
    pieces = [p for proposed in result['proposals'].values() for p in proposed.pieces]
    assert not audit_trajectories(pieces, distance)


def test_factory_prepares_exact_dt_without_mutating_aircraft():
    graph=world({1:((0,0,100),(100,0,100))});u=aircraft(graph,1)
    before=replace(u._road_state)
    result=core(graph).compute_partition_proposals({1:u},0,.25,None)
    proposed=result['proposals'][1]
    assert u._road_state==before
    assert proposed.state.speed==pytest.approx(2.5)
    assert proposed.state.position[0]==pytest.approx(.3125)
    assert proposed.pieces[-1].t1==pytest.approx(.25)
    assert result['deferred_ids']==set()
    assert result['metrics']['certified'] is True


def test_same_lane_follower_yields_with_bounded_physical_braking():
    graph=world({1:((0,0,100),(300,0,100)),2:((35,0,100),(335,0,100))})
    rear,front=aircraft(graph,1,20),aircraft(graph,2,5)
    # These routes have the same physical pipe and lane, regardless of UID order.
    graph.nodes['2:0']=replace(graph.nodes['2:0'],pipe_id=1,distance_m=35)
    result=core(graph).compute_partition_proposals({1:rear,2:front},0,1,None)
    assert_current_safe(result)
    assert 10 <= result['proposals'][1].state.speed < 30
    assert result['metrics']['safety_modified_count']>=1


def test_safe_platoon_can_accelerate_together_without_artificial_reaction_hold():
    graph=world({uid:((uid*12,0,100),(300+uid*12,0,100)) for uid in range(8)})
    for uid in range(8):
        graph.nodes['%s:0'%uid]=replace(graph.nodes['%s:0'%uid],pipe_id=0,distance_m=uid*12)
    fleet={uid:aircraft(graph,uid,20) for uid in range(8)}
    result=core(graph).compute_partition_proposals(fleet,0,1,None)
    assert all(p.state.speed==pytest.approx(30) for p in result['proposals'].values())
    assert_current_safe(result)


def test_geometric_crossing_adjusts_acceleration_then_both_make_progress():
    graph=world({1:((-20,0,100),(80,0,100)),2:((0,-20,100),(0,80,100))})
    fleet={uid:aircraft(graph,uid,10) for uid in (1,2)};controller=core(graph)
    for tick in range(40):
        result=controller.compute_partition_proposals(fleet,tick*.25,.25,None)
        assert_current_safe(result)
        for uid,proposed in result['proposals'].items():fleet[uid]._road_state=proposed.state
        fleet={uid:u for uid,u in fleet.items() if u._road_state.status!='ARRIVED'}
        if not fleet:break
    assert not fleet


def test_different_lanes_overtaking_respects_3d_separation():
    graph=world({1:((0,0,100),(200,0,100)),2:((35,6,100),(235,6,100))})
    result=core(graph).compute_partition_proposals(
        {1:aircraft(graph,1,20),2:aircraft(graph,2,5)},0,1,None)
    assert_current_safe(result)
    assert result['metrics']['safety_modified_count']>=1


def test_independent_lanes_advance_concurrently_without_region_reservation():
    graph=world({1:((0,0,100),(200,0,100)),2:((0,20,100),(200,20,100))})
    result=core(graph).compute_partition_proposals(
        {1:aircraft(graph,1,10),2:aircraft(graph,2,10)},0,1,None)
    assert [p.state.speed for p in result['proposals'].values()]==pytest.approx([20,20])
    assert_current_safe(result)


def test_ramp_merge_is_a_geometric_conflict_without_locking_the_entire_ramp():
    graph=world({1:((-25,0,100),(100,0,100)),2:((-25,-20,110),(0,0,100),(100,0,100))},ramp_ids={2:7})
    result=core(graph).compute_partition_proposals(
        {1:aircraft(graph,1,12),2:aircraft(graph,2,12)},0,1,None)
    assert_current_safe(result)
    assert len(result['proposals'])==2


def test_unsafe_cold_spawn_is_deferred_but_stationary_airborne_occupant_remains():
    graph=world({1:((0,0,100),(100,0,100)),2:((2,0,100),(102,0,100))})
    fleet={1:aircraft(graph,1,0,'NO_ROUTE'),2:aircraft(graph,2,0,'WAITING')}
    result=core(graph).compute_partition_proposals(fleet,0,1,None,new_ids=(2,))
    assert result['deferred_ids']=={2}
    assert set(result['proposals'])=={1}
    assert result['proposals'][1].state.position==(0,0,100)
    assert fleet[2]._road_state.status=='WAITING'


def test_local_prefix_guard_preserves_a_full_braking_continuation():
    graph=world({1:((0,0,100),(40,0,100))});u=aircraft(graph,1,15,goal_last=False)
    result=core(graph).compute_partition_proposals({1:u},0,1,None)
    proposed=result['proposals'][1]
    distance_left=40-proposed.state.position[0]
    assert distance_left+1e-7>=proposed.state.speed**2/20
    assert proposed.state.speed>=5


def test_lower_speed_entry_is_anticipated_without_clipping():
    graph=world({1:((0,0,100),(25,0,100),(100,0,100))},caps={(1,1):30,(1,2):5})
    result=core(graph).compute_partition_proposals({1:aircraft(graph,1,15)},0,1,None)
    candidate=result['proposals'][1]
    assert candidate.state.speed>=5
    continuation=advance(candidate.state,graph,-10,1,candidate.state.speed/10+1e-7)
    assert continuation.state.speed==pytest.approx(0)


def test_height_threshold_uses_larger_member_of_pair():
    graph=world({1:((0,0,200),(100,0,200)),2:((0,9,200),(100,9,200))})
    result=core(graph).compute_partition_proposals(
        {1:aircraft(graph,1,0,'NO_ROUTE'),2:aircraft(graph,2,0,'WAITING')},0,.1,None,new_ids=(2,))
    assert result['deferred_ids']=={2}


def test_far_high_destination_does_not_reduce_bottom_layer_lane_capacity():
    paths={uid:((0,uid*10,100),(2000,uid*10,100),(3000,uid*10,600)) for uid in range(20)}
    graph=world(paths)
    fleet={uid:aircraft(graph,uid,0,'WAITING') for uid in paths}
    result=core(graph).compute_partition_proposals(fleet,0,1,None,new_ids=tuple(paths))
    assert not result['deferred_ids']
    assert len(result['proposals'])==20
    assert all(value.state.speed==pytest.approx(10) for value in result['proposals'].values())


def test_infeasible_airborne_initial_state_fails_explicitly_instead_of_teleporting():
    graph=world({1:((0,0,100),(100,0,100)),2:((3,0,100),(103,0,100))})
    with pytest.raises(RuntimeError,match='(?i)(infeasible|certificate|separation)'):
        core(graph).compute_partition_proposals(
            {1:aircraft(graph,1,10),2:aircraft(graph,2,10)},0,1,None)


def test_bounded_search_defers_unresolved_admissions_before_fleet_brake_fallback():
    graph=world({1:((0,0,100),(200,0,100)),2:((40,0,100),(240,0,100)),
                 3:((0,40,100),(200,40,100)),4:((40,40,100),(240,40,100))})
    fleet={1:aircraft(graph,1,30),2:aircraft(graph,2,0,'WAITING'),
           3:aircraft(graph,3,30),4:aircraft(graph,4,0,'WAITING')}
    result=core(graph,max_iterations=1).compute_partition_proposals(fleet,0,1,None,new_ids=(2,4))
    assert result['deferred_ids']=={2,4}
    assert set(result['proposals'])=={1,3}
    assert_current_safe(result)


@pytest.mark.parametrize('dt',[.1,.25,1.])
def test_goal_sink_arrival_uses_exact_physical_event_time(dt):
    graph=world({1:((0,0,100),(30,0,100))});u=aircraft(graph,1)
    controller=core(graph)
    for tick in range(100):
        proposed=controller.compute_partition_proposals({1:u},tick*dt,dt,None)['proposals'][1]
        u._road_state=proposed.state
        if proposed.state.status=='ARRIVED':break
    assert proposed.state.status=='ARRIVED'
    assert proposed.state.arrival_time==pytest.approx(math.sqrt(6),abs=1e-8)
