import math
from types import SimpleNamespace
import pytest
from uav_sim.airspace.environment import Pipeline
from uav_sim.airspace.partition_contracts import RoadNode,RoadEdge,PartitionGraph,Destination
from uav_sim.entities.uav import UAV,get_uav_3d_pos


def fixture():
    points={'a':(0,0,100),'b':(10,0,100),'c':(10,0,100),'d':(10.5,0,100),'e':(10.5,0,100),'f':(30,0,100)}
    regions={'a':0,'b':0,'c':1,'d':1,'e':2,'f':2}
    nodes={k:RoadNode(k,regions[k],p,0 if k in ('a','b','c') else 1, p[0]) for k,p in points.items()}
    kinds=['FLY','BOUNDARY','RAMP','BOUNDARY','FLY']
    edges={s+t:RoadEdge(s+t,s,t,regions[s],kind,math.dist(points[s],points[t]),20,(points[s],points[t]),ramp_id=7 if kind=='RAMP' else None,lane_count=1) for (s,t),kind in zip(zip('abcde','bcdef'),kinds)}
    graph=PartitionGraph(nodes,edges,{i:{} for i in range(3)})
    env=SimpleNamespace(pipelines=[Pipeline(0,(0,0,100),(10,0,100),100,20,num_lanes=1),Pipeline(1,(10.5,0,100),(30,0,100),100,20,num_lanes=1)])
    goal=Destination('f',2,{},pipe_id=1,position=(30,0,100),distance_m=19.5)
    return env,graph,goal


@pytest.mark.parametrize('dt',[1.,.1])
def test_existing_uav_prepares_without_mutation_and_commits_exact_motion(dt):
    env,graph,goal=fixture();u=UAV(0,0,0,1,env,capacity=0)
    u.configure_partition_motion(graph,'a',goal)
    u.set_partition_route(('ab','bc','cd','de','ef'))
    candidate=u.prepare_partition_step(10,0,dt)
    assert u.pos_1d==0 and u.speed==0
    u.commit_partition_step(candidate)
    assert u.speed==pytest.approx(10*dt)
    t=dt
    while u.state!='ARRIVED':
        u.commit_partition_step(u.prepare_partition_step(10,t,dt));t+=dt
    assert u._road_state.arrival_time==pytest.approx(2.5,abs=1e-9)
    assert tuple(get_uav_3d_pos(u,env))==pytest.approx((30,0,100))
    assert u._road_state.visited_regions==(0,1,2)


def test_local_prefix_fetches_next_map_at_exact_boundary_event():
    env,graph,goal=fixture();u=UAV(0,0,0,1,env,capacity=0)
    u.configure_partition_motion(graph,'a',goal);u.set_partition_route(('ab','bc'))
    events=[]
    def next_region(state,time):
        events.append((state.region_id,time))
        return ('cd','de') if state.region_id==1 else ('ef',)
    for i in range(3):u.commit_partition_step(u.prepare_partition_step(10,i,1,next_region))
    assert u.state=='ARRIVED'
    assert events[0]==pytest.approx((1,math.sqrt(2)),abs=1e-9)
    assert u._road_state.arrival_time==pytest.approx(2.5,abs=1e-9)
