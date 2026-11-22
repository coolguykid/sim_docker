import math
import unittest
from airway import Config, build_network


class SpacingTests(unittest.TestCase):
    def test_reverse_stations_continue_across_regions(self):
        net=build_network(Config(boundary_m=3000,grid_n=2,pipes_per_reference_width=24,
                                 reverse_ramp_spacing_m=400))
        for pair in net.pairs.values():
            pipe=net.pipes[pair.pipe_ids[0]]
            theta=math.radians(net.layers[pipe.layer_id].angle_deg)
            ramps=[r for r in net.ramps.values() if r.kind=='reverse'
                   and net.segments[r.source_segment].pipe_id==pipe.id]
            stations=sorted(r.start[0]*math.cos(theta)+r.start[1]*math.sin(theta) for r in ramps)
            for a,b in zip(stations,stations[1:]):self.assertAlmostEqual(b-a,400)
            for r in ramps:
                reverse=f'{r.target_segment}>{r.source_segment}#{r.id.split("#")[1]}'
                self.assertIn(reverse,net.ramps)
                self.assertEqual(net.ramps[reverse].start,r.end)
                self.assertAlmostEqual(math.dist(r.start,r.end),net.config.pitch_m)
        self.assertTrue(any(sum(net.ramps[r].kind=='reverse' for r in ids)>1 for ids in net.incoming.values()))

    def test_uniform_layer_spacing_and_three_pair_preset(self):
        net=build_network(Config(boundary_m=3000,grid_n=2,pipes_per_reference_width=60,
                                 initial_pair_spacing_m=500))
        net.initialize()
        self.assertTrue(all(l.spacing_m==500 for l in net.layouts.values()))
        for region in net.regions.values():
            pipes=[net.pipes[net.segments[s].pipe_id] for s in region.segment_ids
                   if net.pipes[net.segments[s].pipe_id].layer_id==0
                   and net.pipes[net.segments[s].pipe_id].is_open]
            self.assertEqual(len(pipes),6)
            self.assertEqual(len({p.pair_id for p in pipes}),3)

    def test_invalid_spacing(self):
        for field in ('initial_pair_spacing_m','reverse_ramp_spacing_m'):
            for value in (0,-1,float('nan'),float('inf')):
                with self.assertRaises(ValueError):Config(**{field:value})
