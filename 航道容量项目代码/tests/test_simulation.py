import math
import random
import unittest
from collections import Counter

from airway import ThroughputModel, Config, Controller, DroneSimulation, Station, build_network
from airway.model import Layer, Network, Pipe, Segment, Ramp, Region
from airway.simulation import FlightGraph


def small_network():
    net=Network(Config(),ThroughputModel())
    net.layers={0:Layer(0,0,135),1:Layer(1,0,265)}
    net.regions={'R1-1':Region('R1-1',0,0)}
    for i,pid in enumerate(('A','B')):
        z=135+130*i
        p=Pipe(pid,i,1,pid,1,(0,0,z),(100,0,z),20,True,[pid+'@R1-1'])
        net.pipes[pid]=p
        sid=p.segment_ids[0]
        net.segments[sid]=Segment(sid,pid,'R1-1',p.start,p.end,100)
        net.regions['R1-1'].segment_ids.append(sid)
        net.incoming[sid]=[]
    for rid,a,b,x,y in [('AB','A','B',20,80),('BA','B','A',100,0)]:
        sa,sb=a+'@R1-1',b+'@R1-1'
        net.ramps[rid]=Ramp(rid,sa,sb,'interlayer',(x,0,net.pipes[a].start[2]),(y,0,net.pipes[b].start[2]))
        net.incoming[sb].append(rid)
    return net


class SimulationTests(unittest.TestCase):

    def test_reachable_exits_match_routes_with_direction_and_blocked_tail(self):
        net=small_network()
        net.segments['A@R1-1'].length_m=50
        net.segments['A@R1-2']=Segment('A@R1-2','A','R1-2',(50,0,135),(100,0,135),50)
        net.pipes['A'].segment_ids.append('A@R1-2')
        for blocked in ((),{'A@R1-2'},{'B@R1-1'},{'A@R1-1','B@R1-1'}):
            graph=FlightGraph(net,[],blocked)
            for source in ('A','B'):
                reachable=graph.reachable_exits(source)
                for target in ('A','B'):
                    goal=Station(0,target,100,'R1-1')
                    route=graph.route(source,0,goal,random.Random(1),allow_origin=False)
                    self.assertEqual(target in reachable,route is not None)
            if blocked == {'A@R1-2'}:
                self.assertNotIn('A',graph.reachable_exits('A'))
                self.assertIn('B',graph.reachable_exits('A'))
        net.pipes['A'].is_open=False
        self.assertFalse(FlightGraph(net,[]).reachable_exits('A'))

    def test_initial_destination_sets_exclude_unreachable_exits_and_sample_uniformly(self):
        net=small_network()
        net.layers[2]=Layer(2,0,395)
        net.pipes['C']=Pipe('C',2,1,'C',1,(0,0,395),(100,0,395),20,True,['C@R1-1'])
        net.segments['C@R1-1']=Segment('C@R1-1','C','R1-1',(0,0,395),(100,0,395),100)
        net.incoming['C@R1-1']=[]
        sim=DroneSimulation(net,adaptive=False,arrival_alpha=0)
        self.assertEqual(sim.destination_options['A'],('A','B'))
        self.assertEqual(sim.destination_options['C'],('C',))
        chosen=Counter(sim._request('A',i).destination.pipe_id for i in range(4000))
        self.assertEqual(set(chosen),{'A','B'})
        self.assertAlmostEqual(chosen['A']/4000,.5,delta=.03)
        self.assertAlmostEqual(chosen['B']/4000,.5,delta=.03)

    def test_temporary_unreachability_does_not_change_initial_destinations(self):
        net=small_network()
        sim=DroneSimulation(net,adaptive=False,arrival_alpha=0)
        original=sim.destination_options.copy()
        sim.destination_options['A']=('B',)
        net.ramps['AB'].enabled=False
        sim.graph=FlightGraph(net,[])
        first=sim._request('A',0)
        sim.pending_entries['A'].append(first)
        second=sim._spawn('A',1)
        self.assertEqual(first.destination.pipe_id,'B')
        self.assertEqual(second.destination.pipe_id,'B')
        self.assertEqual(sim.generated,2)
        self.assertFalse(sim.drones)
        self.assertEqual(sim.metrics()['outside_waiting'],2)
        net.ramps['AB'].enabled=True
        sim.graph=FlightGraph(net,[])
        sim.step(2)
        self.assertEqual(sim.entry_log[0]['id'],first.id)
        self.assertEqual(sim.arrival_log[first.id]['destination_pipe_id'],'B')
        sim.destination_options['A']=original['A']
        self.assertEqual(sim.destination_options,original)

    def test_temporary_blocking_keeps_identical_generated_tasks_and_conservation(self):
        a=DroneSimulation(small_network(),adaptive=False,seed=11,arrival_alpha=9.37)
        b=DroneSimulation(small_network(),adaptive=False,seed=11,arrival_alpha=9.37)
        b.network.ramps['AB'].enabled=False
        b.graph=FlightGraph(b.network,[],{'B@R1-1'})
        b.blocked_segments=frozenset({'B@R1-1'})
        for _ in range(12):
            a.step(1);b.step(1)
        self.assertEqual(a.arrival_log,b.arrival_log)
        self.assertEqual(b.generated,a.generated)
        self.assertGreater(b.metrics()['outside_waiting'],0)
        for sim in (a,b):
            m=sim.metrics()
            self.assertEqual(m['requests'],m['generated'])
            self.assertEqual(m['generated'],m['entered']+m['outside_waiting'])
            self.assertEqual(m['entered'],m['population']+m['completed'])

    def test_blocked_ramp_target_and_origin_escape(self):
        graph=FlightGraph(small_network(),[],{'B@R1-1'})
        target=Station(1,'B',100,'R1-1')
        self.assertIsNone(graph.route('A',10,target,random.Random(1)))
        self.assertIsNone(graph.route('B',0,target,random.Random(1),allow_origin=False))
        self.assertEqual(graph.route('B',90,target,random.Random(1)),[('B',100)])

    def test_blocked_downstream_segment_cannot_use_direct_shortcut(self):
        net=small_network()
        net.segments['A@R1-1'].length_m=50
        net.segments['A@R1-2']=Segment('A@R1-2','A','R1-2',(50,0,135),(100,0,135),50)
        net.pipes['A'].segment_ids.append('A@R1-2')
        graph=FlightGraph(net,[],{'A@R1-2'})
        end=Station(1,'A',100,'R1-2')
        self.assertIsNone(graph.route('A',10,end,random.Random(1)))
        self.assertEqual(graph.route('A',60,end,random.Random(1)),[('A',100)])
        self.assertEqual(graph.route('A',10,Station(1,'A',40,'R1-1'),random.Random(1)),[('A',40)])

    def test_origin_escape_cannot_reenter_blocked_segment_from_behind(self):
        net=small_network()
        net.segments['A@R1-1'].length_m=50
        net.segments['A@R1-2']=Segment('A@R1-2','A','R1-2',(50,0,135),(100,0,135),50)
        net.pipes['A'].segment_ids.append('A@R1-2')
        net.ramps['escape']=Ramp('escape','A@R1-2','B@R1-1','interlayer',(80,0,135),(80,0,265))
        graph=FlightGraph(net,[],{'A@R1-2'})
        self.assertIsNotNone(graph.route('A',60,Station(1,'B',100,'R1-1'),random.Random(1)))
        self.assertIsNone(graph.route('A',60,Station(1,'A',55,'R1-2'),random.Random(1)))

    def test_new_pipe_carries_rerouted_existing_task_without_new_entry(self):
        net=small_network()
        # A的中段超标，C作为新增旁路；出口仍是最初的A出口。
        net.segments['A@R1-1'].length_m=30
        net.pipes['A'].segment_ids.extend(['A@R1-2','A@R1-3'])
        for rid,a,b in [('R1-2',30,80),('R1-3',80,100)]:
            sid='A@'+rid
            net.segments[sid]=Segment(sid,'A',rid,(a,0,135),(b,0,135),b-a)
            net.incoming[sid]=[]
        net.layers[2]=Layer(2,0,395)
        net.pipes['C']=Pipe('C',2,1,'C',1,(0,0,395),(100,0,395),20,False,['C@R1-1'])
        net.segments['C@R1-1']=Segment('C@R1-1','C','R1-1',(0,0,395),(100,0,395),100)
        net.incoming['C@R1-1']=[]
        for rid,source,target,a,b in [('AC','A@R1-1','C@R1-1',(20,0,135),(0,0,395)),
                                      ('CA','C@R1-1','A@R1-3',(100,0,395),(90,0,135))]:
            net.ramps[rid]=Ramp(rid,source,target,'interlayer',a,b)
            net.incoming[target].append(rid)
        sim=DroneSimulation(net,adaptive=False,arrival_alpha=0)
        d=sim._spawn('A',0)
        d.distance_m=10;d.destination=Station(d.id,'A',100,'R1-3')
        net.pipes['C'].is_open=True
        sim.graph=FlightGraph(net,[],{'A@R1-2'})
        sim._sync_entries();sim._route(d)
        self.assertNotIn('C',sim.entries)
        self.assertTrue(any(pid=='C' for pid,_ in d.route))
        sim.step(7)
        self.assertEqual(sim.completed,1)
        self.assertEqual(sim.expansion_users,{d.id})
        self.assertGreater(sim.expansion_distance_m,0)

    def test_target_stopping_matches_complete_shortest_search(self):
        net=small_network();graph=FlightGraph(net,[])
        for target in [Station(1,'A',70,'R1-1'),Station(1,'B',85,'R1-1'),Station(1,'A',5,'R1-1')]:
            full=graph.finish('A',10,target,graph.search('A',10,random.Random(2)))
            self.assertEqual(graph.route('A',10,target,random.Random(2)),full)


    def test_arbitrary_endpoints_between_graph_nodes(self):
        net=small_network()
        graph=FlightGraph(net,[])
        self.assertEqual(graph.route('A',3.5,Station(1,'A',7.25,'R1-1'),random.Random(4)),
                         [('A',7.25)])
        route=graph.route('A',3.5,Station(1,'B',86.75,'R1-1'),random.Random(4))
        self.assertEqual(route,[('A',20),('B',80),('B',86.75)])
        net.ramps['AB'].enabled=False
        self.assertIsNone(FlightGraph(net,[]).route('A',3.5,Station(1,'B',86.75,'R1-1'),random.Random(4)))
        net.pipes['A'].is_open=False
        self.assertIsNone(FlightGraph(net,[]).route('A',3.5,Station(1,'A',7.25,'R1-1'),random.Random(4)))




    def test_shortest_distance_and_one_way_ramps(self):
        net=small_network()
        start,end=Station(0,'A',0,'R1-1'),Station(1,'B',90,'R1-1')
        graph=FlightGraph(net,[start,end])
        self.assertEqual(graph.search('A',0,random.Random(1))[0][graph.index[('B',90)]][0],30)
        self.assertEqual(graph.route('A',10,end,random.Random(1)),[('A',20),('B',80),('B',90)])
        net.ramps['AB'].congestion_blocked=True
        graph=FlightGraph(net,[start,end])
        self.assertIsNone(graph.route('A',10,end,random.Random(1)))
        self.assertIsNotNone(graph.route('B',90,start,random.Random(1)))






if __name__=='__main__':unittest.main()
