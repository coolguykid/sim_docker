import math
import random
import unittest

from airway import Config, ThroughputModel, Controller, DroneSimulation, build_network


class ThroughputTests(unittest.TestCase):
    def net(self, grid=1):
        n = build_network(Config(boundary_m=3000, grid_n=grid, pipes_per_reference_width=24))
        n.initialize()
        return n

    def test_flow_formula_window_and_optimum(self):
        m = ThroughputModel()
        self.assertAlmostEqual(m.safety_gap(20), 114)
        self.assertAlmostEqual(m.safe_spacing(20), 187.4)
        self.assertAlmostEqual(m.headway(20), 187.4/20)
        self.assertAlmostEqual(m.throughput(20, 60), 60*20/187.4)
        self.assertAlmostEqual(m.throughput(20, 120), 2*m.throughput(20, 60))
        best = m.best_speed(10, 50)
        self.assertGreater(m.throughput(best, 60), m.throughput(10, 60))
        self.assertGreater(m.throughput(best, 60), m.throughput(20, 60))
        for speed, window in [(0,60),(20,0),(20,float('nan'))]:
            with self.assertRaises(ValueError): m.throughput(speed,window)

    def test_original_matlab_sweep_and_continuous_optimum(self):
        # 独立逐点复算原脚本的三组参数、21个扫描速度与单位换算。
        for s0 in (30,60,90):
            model=ThroughputModel(base_spacing_m=s0)
            for speed in range(10,51,2):
                gap=s0+1.2*speed+.6*speed**2/(2*2)
                reference= speed/(5+1.6*gap)
                self.assertAlmostEqual(model.throughput(speed,1),reference)
                self.assertAlmostEqual(model.throughput(speed,60),reference*60)
            best=model.best_speed(10,50)
            self.assertAlmostEqual(best,math.sqrt((5+1.6*s0)/.24))
            for speed in (best-.01,best+.01):
                self.assertGreater(model.throughput(best,60),model.throughput(speed,60))

    def test_layer_expands_without_borrowing_other_layers(self):
        n = self.net()
        maximum = n.throughput_model.throughput(n.throughput_model.best_speed(10,50),60)
        demand = {sid:maximum*1.5 for sid,s in n.segments.items()
                  if n.pipes[s.pipe_id].is_open and n.pipes[s.pipe_id].layer_id == 0}
        self.assertLess(sum(demand.values()), n.region_totals('R1-1')[1])
        events = Controller(n).step(0,demand)
        added = [e for e in events if e.action == 'pairs_expanded']
        self.assertTrue(added)
        self.assertTrue(all(e.details['layer_id'] == 0 for e in added))
        self.assertLess(*n.region_totals('R1-1',0))
        self.assertEqual(sum(s.count for s in n.segments.values()),0)

    def test_actual_flow_validation_is_atomic_and_forecast_is_separate(self):
        n=self.net(); c=Controller(n); sid=next(iter(n.segments))
        n.segments[sid].demand=100
        c.step(0,{sid:1.5},counts={sid:2})
        for invalid in ({sid:-1},{sid:float('nan')},{sid:True},{'bad':1}):
            with self.assertRaises(ValueError): c.step(1,invalid)
        self.assertEqual(c.last_time_s,0)
        self.assertEqual(n.segments[sid].count,2)
        self.assertEqual(n.segments[sid].passed,1.5)
        self.assertEqual(n.segments[sid].demand,100)

    def test_control_uses_measured_flow_even_when_prediction_disagrees(self):
        n=self.net(); sim=DroneSimulation(n,arrival_alpha=0)
        sid=next(sid for sid,s in n.segments.items() if n.pipes[s.pipe_id].is_open)
        sim.predictor=lambda *_:{sid:1000}
        sim.refresh_prediction()
        self.assertEqual(n.segments[sid].demand,1000)
        sim._control()
        self.assertFalse(sim.events)
        self.assertEqual(n.pipes[n.segments[sid].pipe_id].speed_mps,20)
        # 实测超标、预测为零仍触发控制；无整数位增益时跳过调速并保留观测。
        sim.predictor=lambda *_:{}
        sim.refresh_prediction()
        sim.passage_events.extend((0,sid) for _ in range(8))
        sim.time_s=1
        sim._control()
        self.assertFalse(any(e.action=='speed_changed' for e in sim.events))
        self.assertTrue(any(e.action=='unresolved' for e in sim.events))
        self.assertEqual(n.pipes[n.segments[sid].pipe_id].speed_mps,20)
        self.assertEqual(n.segments[sid].passed,8)
        self.assertEqual(n.segments[sid].demand,0)
        sim.time_s=62
        sim._control()
        self.assertEqual(n.pipes[n.segments[sid].pipe_id].speed_mps,20)
        self.assertEqual(n.segments[sid].passed,0)

    def test_motion_and_control_do_not_require_a_predictor(self):
        sim=DroneSimulation(self.net(),arrival_alpha=0)
        def unavailable(*_):
            raise RuntimeError('预测器不可用')
        sim.predictor=unavailable
        sim.step(20)
        self.assertEqual(sim.time_s,20)
        self.assertFalse(sim.events)
        with self.assertRaises(RuntimeError): sim.refresh_prediction()

    def test_skipped_speed_adjustment_preserves_observed_flow_and_logs_overload(self):
        n=self.net(); sid=next(sid for sid,s in n.segments.items() if n.pipes[s.pipe_id].is_open)
        events=Controller(n).step(0,{sid:10})
        self.assertFalse(any(e.action=='speed_changed' for e in events))
        unresolved=next(e for e in events if e.action=='unresolved')
        self.assertEqual(unresolved.details['count'],10)
        self.assertEqual(n.pipes[n.segments[sid].pipe_id].speed_mps,20)
        self.assertEqual(n.segments[sid].passed,10)
        self.assertEqual(n.region_totals('R1-1')[0],10)

    def test_boundary_births_exits_and_population_conservation(self):
        n=self.net(); sim=DroneSimulation(n,adaptive=False,arrival_alpha=2)
        self.assertFalse(sim.drones)
        sim.step(200)
        self.assertGreater(sim.completed,0)
        self.assertEqual(sim.entered-sim.completed,len(sim.drones))
        self.assertEqual(sum(sim.counts().values()),len(sim.drones))
        for d in sim.drones.values():
            self.assertEqual(d.origin.distance_m,0)
            p=n.pipes[d.destination.pipe_id]
            self.assertAlmostEqual(d.destination.distance_m,math.dist(p.start,p.end))
            for point in (n.pipes[d.origin.pipe_id].start,p.end):
                self.assertAlmostEqual(max(abs(point[0]),abs(point[1])),1500)

    def test_truncated_exponential_and_actual_entry_headways(self):
        rng=random.Random(7)
        samples=[DroneSimulation.sample_interval(rng,.5,4) for _ in range(20000)]
        self.assertTrue(all(x>4 for x in samples))
        self.assertAlmostEqual(sum(samples)/len(samples),6,delta=.06)
        sim=DroneSimulation(self.net(),adaptive=False,arrival_alpha=2)
        sim.step(60)
        last={}
        for e in sorted(sim.entry_log,key=lambda e:e['time_s']):
            if e['pipe_id'] in last:
                self.assertGreater(e['time_s']-last[e['pipe_id']],e['headway_s']-1e-8)
            last[e['pipe_id']]=e['time_s']

    def test_entry_rates_derived_from_speed_and_alpha_with_closed_entries_excluded(self):
        n=self.net(); sim=DroneSimulation(n,seed=3,arrival_alpha=2)
        sources={p.id for p in n.pipes.values() if p.is_open}
        self.assertEqual(set(sim.entry_lambdas),sources)
        self.assertEqual(set(sim.entries),sources)
        for pid,rate in sim.entry_lambdas.items():
            self.assertAlmostEqual(rate,2*n.pipes[pid].speed_mps/
                                   n.throughput_model.safe_spacing(n.pipes[pid].speed_mps))
        zero=DroneSimulation(n,arrival_alpha=0)
        zero.step(10)
        self.assertEqual(zero.generated,0)
        self.assertEqual(zero.entered,0)

    def test_rolling_actual_flow_expires_and_is_not_occupancy(self):
        sim=DroneSimulation(self.net(),adaptive=False,arrival_alpha=0)
        sid=next(iter(sim.network.segments))
        sim.passage_events.extend([(1,sid),(10,sid)])
        sim.time_s=61
        self.assertEqual(sim.passed(),{sid:1})
        self.assertEqual(sim.counts(),{})
        sim.time_s=70
        self.assertEqual(sim.passed(),{})

    def test_predicted_crossing_and_real_crossing_agree_without_control(self):
        sim=DroneSimulation(self.net(),adaptive=False,arrival_alpha=0)
        pid=next(iter(sim.graph.positions))
        drone=sim._spawn(pid,0)
        forecast=sim.predict_demand()
        sim.step(60)
        self.assertEqual(forecast,sim.passed())

    def test_expansion_keeps_external_sources_and_current_entry_headways(self):
        n=self.net(); sim=DroneSimulation(n,adaptive=False,arrival_alpha=2)
        sources=set(sim.entries)
        sim.step(10)
        layout=n.layouts['R1-1/L1']
        changes=n.apply_layout(layout.id,1)
        from airway.simulation import FlightGraph
        sim.graph=FlightGraph(n,[])
        sim._sync_entries()
        self.assertEqual(set(sim.entries),sources)
        for pair in changes['opened']:
            self.assertFalse(set(n.pairs[pair].pipe_ids)&sim.entries.keys())
        for p in n.pipes.values(): p.speed_mps=50
        sim.step(60)
        last={}
        for e in sorted(sim.entry_log,key=lambda e:e['time_s']):
            self.assertIn(e['pipe_id'],sources)
            if e['pipe_id'] in last:
                self.assertGreater(e['time_s']-last[e['pipe_id']],e['headway_s']-1e-8)
            last[e['pipe_id']]=e['time_s']

    def test_expansion_keeps_requests_and_initial_destinations_at_unchanged_speed(self):
        from airway.simulation import FlightGraph
        a=DroneSimulation(self.net(),adaptive=False,seed=8)
        b=DroneSimulation(self.net(),adaptive=False,seed=8)
        b.network.apply_layout('R1-1/L1',1)
        b.graph=FlightGraph(b.network,[])
        b._sync_entries()
        a.step(80);b.step(80)
        self.assertEqual(a.arrival_log,b.arrival_log)
        self.assertTrue(all(e['destination_pipe_id'] in a.initial_pipe_ids for e in b.arrival_log))
        for sim in (a,b):
            self.assertEqual(sim.generated,sim.entered+sim.metrics()['outside_waiting'])
            self.assertEqual(sim.entered,sim.completed+len(sim.drones))

    def test_speed_change_updates_requests_without_adding_sources_or_destinations(self):
        a=DroneSimulation(self.net(),adaptive=False,seed=8)
        b=DroneSimulation(self.net(),adaptive=False,seed=8)
        sources=set(b.entries)
        destinations=b.destination_options.copy()
        before=b.entry_lambdas
        for p in b.network.pipes.values(): p.speed_mps=50
        after=b.entry_lambdas
        self.assertTrue(all(after[pid] != before[pid] for pid in sources))
        a.step(80);b.step(80)
        self.assertNotEqual(a.arrival_log,b.arrival_log)
        self.assertEqual(set(b.entries),sources)
        self.assertEqual(b.destination_options,destinations)
        self.assertTrue(all(e['destination_pipe_id'] in b.initial_pipe_ids for e in b.arrival_log))
        for e in b.arrival_log:
            self.assertEqual(e['speed_mps'],50)
            self.assertAlmostEqual(e['lambda_per_s'],e['alpha']*e['lambda0_per_s'])

    def test_temporarily_unreachable_entry_queues_original_tasks_and_recovers(self):
        from test_simulation import small_network
        sim=DroneSimulation(small_network(),adaptive=False,arrival_alpha=0,
                            entry_alphas={'B':9.37},control_interval_s=15)
        sid='B@R1-1'
        sim.passage_events.extend((1,sid) for _ in range(10))
        sim.time_s=1;sim._control()
        sim.step(20)
        self.assertGreater(sim.generated,0)
        self.assertEqual(sim.entered,0)
        self.assertEqual(sim.metrics()['outside_waiting'],sim.generated)
        self.assertFalse(sim.drones)
        first=sim.pending_entries['B'][0]
        sim.step(55)
        self.assertGreater(sim.entered,0)
        self.assertEqual(sim.entry_log[0]['id'],first.id)
        self.assertEqual(sim.arrival_log[first.id]['destination_pipe_id'],first.destination.pipe_id)
        self.assertEqual(sim.generated,sim.entered+sim.metrics()['outside_waiting'])
        self.assertEqual(len(sim.arrival_log),sim.generated)

    def test_already_generated_pending_task_keeps_destination_after_blocking(self):
        from test_simulation import small_network
        sim=DroneSimulation(small_network(),adaptive=False,arrival_alpha=0,
                            entry_alphas={'B':9.37},control_interval_s=15)
        first=sim._request('B',0)
        sim.pending_entries['B'].append(first)
        sim.passage_events.extend((1,'B@R1-1') for _ in range(10))
        sim.time_s=1;sim._control()
        sim.step(20)
        self.assertEqual(sim.pending_entries['B'][0],first)
        self.assertEqual(sim.entered,0)
        self.assertGreater(sim.generated,1)
        sim.step(55)
        self.assertEqual(sim.entry_log[0]['id'],first.id)
        self.assertEqual(sim.arrival_log[first.id]['destination_pipe_id'],first.destination.pipe_id)
        self.assertEqual(sim.generated,sim.entered+sim.metrics()['outside_waiting'])
        self.assertEqual(len(sim.arrival_log),sim.generated)

    def test_prediction_has_no_motion_or_statistics_side_effects(self):
        sim=DroneSimulation(self.net(),adaptive=False)
        sim.step(20)
        before=[(d.id,d.pipe_id,d.distance_m,d.route_index) for d in sim.drones.values()]
        metrics=sim.metrics()
        tracks=sum(len(v) for v in sim.tracks.values())
        sim.predict_demand()
        self.assertEqual(before,[(d.id,d.pipe_id,d.distance_m,d.route_index) for d in sim.drones.values()])
        self.assertEqual(metrics,sim.metrics())
        self.assertEqual(tracks,sum(len(v) for v in sim.tracks.values()))

    def test_exact_step_end_arrival_is_removed_at_boundary(self):
        from test_simulation import small_network
        from airway.simulation import Station
        sim=DroneSimulation(small_network(),adaptive=False,arrival_alpha=0)
        d=sim._spawn('A',0)
        d.destination=Station(0,'A',100,'R1-1')
        d.route=[('A',100)]
        sim.step(5)
        self.assertEqual(sim.completed,1)
        self.assertFalse(sim.drones)
        self.assertEqual(sim.exit_log[0]['time_s'],5)
        self.assertEqual(sim.tracks[d.id][-1][2],100)

    def test_large_and_small_steps_have_same_arrivals(self):
        a=DroneSimulation(self.net(),adaptive=False,seed=8)
        b=DroneSimulation(self.net(),adaptive=False,seed=8)
        a.step(30)
        for _ in range(30): b.step(1)
        aa=sorted((e['pipe_id'],round(e['time_s'],7)) for e in a.entry_log)
        bb=sorted((e['pipe_id'],round(e['time_s'],7)) for e in b.entry_log)
        self.assertEqual(aa,bb)
        self.assertEqual(a.entered-a.completed,len(a.drones))

    def test_replanning_keeps_unreachable_destination_and_pending_demand(self):
        from test_simulation import small_network
        from airway.simulation import FlightGraph, Station
        net=small_network()
        sim=DroneSimulation(net,adaptive=False,arrival_alpha=0)
        d=sim._spawn('A',0)
        d.destination=Station(99,'B',100,'R1-1')
        sim._route(d)
        self.assertFalse(d.waiting)
        net.ramps['AB'].congestion_blocked=True
        sim.graph=FlightGraph(net,[])
        sim._route(d)
        self.assertTrue(d.waiting)
        self.assertTrue(sim.predict_demand())
        sim.step(2)
        self.assertEqual(d.distance_m,0)
        net.ramps['AB'].congestion_blocked=False
        sim.graph=FlightGraph(net,[])
        sim._route(d)
        self.assertFalse(d.waiting)

    def test_shrink_protects_same_layer_in_adjacent_region(self):
        net=self.net(grid=2)
        net.apply_layout('R1-1/L1',1)
        sid=next(s for s in net.regions['R1-2'].segment_ids
                 if net.pipes[net.segments[s].pipe_id].is_open
                 and net.pipes[net.segments[s].pipe_id].layer_id==0)
        net.segments[sid].passed=net.region_totals('R1-2',0)[1]*.95
        events=[]
        Controller(net)._shrink_region('R1-1',0,events)
        self.assertFalse(events)
        self.assertEqual(net.layouts['R1-1/L1'].extra_pairs,1)


if __name__ == '__main__': unittest.main()
