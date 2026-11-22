import math
import unittest
from airway import Config, build_network
from airway.initialization import initial_targets, select_initial_pairs


class InitializationTests(unittest.TestCase):
    def test_global_spacing_anchor_and_snapping(self):
        net=build_network(Config(boundary_m=3000,grid_n=2,pipes_per_reference_width=24))
        net.initialize()
        for layer in net.layers.values():
            layout=net.layouts[f"R2-1/L{layer.id+1}"]
            targets=initial_targets(net,layer.id)
            theta=math.radians(layer.angle_deg)
            anchor=layout.center_offset_m-net.config.region_side_m*(abs(math.sin(theta))+abs(math.cos(theta)))/2+net.config.pair_spacing_m/2
            self.assertTrue(any(abs(t-anchor)<1e-6 for t in targets))
            for a,b in zip(targets,targets[1:]):
                self.assertAlmostEqual(b-a,layout.spacing_m)
            ids=select_initial_pairs(net,layer.id)
            for pid,target in zip(ids,targets):
                self.assertLessEqual(abs(net.pair_offset(pid)-target),2*net.config.pitch_m+1e-6)
                self.assertTrue(all(net.pipes[p].is_open for p in net.pairs[pid].pipe_ids))
            opened={p.pair_id for p in net.pipes.values() if p.layer_id==layer.id and p.is_open}
            self.assertEqual(opened,set(ids))
        for layout in net.layouts.values():
            actual={net.pipes[net.segments[s].pipe_id].pair_id for s in net.regions[layout.region_id].segment_ids
                    if net.pipes[net.segments[s].pipe_id].layer_id==layout.layer_id
                    and net.pipes[net.segments[s].pipe_id].is_open}
            self.assertEqual(actual,set(layout.initial_pair_ids))
            self.assertFalse(set(layout.interior_pair_ids)&actual)

    def test_uniform_spacing_and_binary_expansion_targets(self):
        net=build_network(Config(boundary_m=3000,grid_n=1,pipes_per_reference_width=24))
        for layer in net.layers.values():
            layout=net.layouts[f"R1-1/L{layer.id+1}"]
            self.assertEqual(layout.spacing_m,1500)
            chosen=net.layout_pair_ids(layout.id,2)
            actual=[pid for pid in chosen if pid not in layout.initial_pair_ids]
            self.assertEqual(len(actual),2)
            for pid in actual:
                node=net.split_nodes[pid]
                target=(net.pair_offset(node.left_pair_id)+net.pair_offset(node.right_pair_id))/2
                self.assertLessEqual(abs(net.pair_offset(pid)-target),net.config.pitch_m+1e-6)
            self.assertTrue(set(net.layout_pair_ids(layout.id,1)) < set(chosen))

    def test_grid_counts_are_not_forced_to_two(self):
        net=build_network(Config(boundary_m=3000,grid_n=3,pipes_per_reference_width=24))
        self.assertTrue(any(len(l.initial_pair_ids)!=2 for l in net.layouts.values()))
