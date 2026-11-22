import unittest
from airway import Config, build_network, Controller


class ExpansionTests(unittest.TestCase):
    def build(self, grid_n=1):
        net = build_network(Config(boundary_m=3000, grid_n=grid_n, pipes_per_reference_width=24))
        net.initialize()
        return net



    def test_initial_positions_and_nested_midpoints(self):
        net = self.build()
        layout = net.layouts["R1-1/L1"]
        offsets = [net.pair_offset(pid) for pid in layout.initial_pair_ids]
        # 对中心在125+250j的离散格点，等距时选较小编号。
        self.assertEqual(offsets, [-875, 625])
        for k in (0, 1, 2, 3):
            selected = net.layout_pair_ids(layout.id, k)
            self.assertEqual(len(selected), k+2)
            self.assertEqual(len(set(selected)), k+2)
            self.assertTrue(set(layout.initial_pair_ids) <= set(selected))
        self.assertIn(-125, [net.pair_offset(pid) for pid in net.layout_pair_ids(layout.id, 1)])
        self.assertEqual([net.pair_offset(pid) for pid in net.layout_pair_ids(layout.id, 2)], [-875,-625,-125,625])
        self.assertEqual([net.pair_offset(pid) for pid in net.layout_pair_ids(layout.id, 3)], [-875,-625,-125,125,625])
        previous = set(layout.initial_pair_ids)
        depths = []
        for k in range(1, len(layout.interior_pair_ids)+1):
            changes = net.apply_layout(layout.id, k)
            current = set(layout.selected_pair_ids)
            self.assertTrue(previous < current)
            self.assertEqual(len(changes['opened']), 1)
            self.assertFalse(changes['closed'])
            pid = (current-previous).pop()
            node = net.split_nodes[pid]
            self.assertTrue({node.left_pair_id,node.right_pair_id} <= previous)
            depths.append(node.depth)
            previous = current
        self.assertEqual(depths, sorted(depths))
        with self.assertRaises(ValueError):net.layout_pair_ids(layout.id,10000)

    def test_deepest_leaf_shrinks_and_occupied_leaf_is_skipped(self):
        net = self.build()
        lid = 'R1-1/L1'
        net.apply_layout(lid, 4)
        ctrl = Controller(net)
        extras = set(net.layouts[lid].selected_pair_ids)-set(net.layouts[lid].initial_pair_ids)
        deepest = max(extras, key=lambda p: net.split_nodes[p].depth)
        parent = net.split_nodes[deepest].parent_pair_id
        sid = net.pipes[net.pairs[deepest].pipe_ids[0]].segment_ids[0]
        net.segments[sid].count = 1
        events = []
        ctrl._shrink_region('R1-1', 0, events)
        self.assertEqual(len(events), 1)
        self.assertNotIn(events[0].details['pair_id'], (deepest, parent))
        self.assertEqual(len(events[0].details['closed']), 1)
        self.assertFalse(events[0].details['opened'])
        net.segments[sid].count = 0
        events = []
        ctrl._shrink_region('R1-1', 1, events)
        self.assertEqual(events[0].details['pair_id'], deepest)

    def test_shared_child_keeps_parent_until_pruned(self):
        net = self.build(grid_n=2)
        left, right = 'R1-1/L1', 'R1-2/L1'
        net.apply_layout(left, 3)
        net.apply_layout(right, 3)
        net.apply_layout(left, 0)
        for pid in net.layouts[right].selected_pair_ids:
            self.assertTrue(net.pipes[net.pairs[pid].pipe_ids[0]].is_open)
        ctrl = Controller(net)
        events = []
        ctrl._shrink_region('R1-2', 0, events)
        self.assertEqual(events[0].details['depth'], 1)
        self.assertEqual(len(events[0].details['closed']), 1)

    def test_regional_candidates_are_global_leaf_gaps(self):
        net = self.build(grid_n=2)
        for layout in net.layouts.values():
            for pid in net.expansion_candidates(layout.id):
                self.assertIn(pid, layout.candidate_pair_ids)
                node = net.split_nodes[pid]
                self.assertEqual(node.depth, 0)
                for bound in (node.left_pair_id, node.right_pair_id):
                    self.assertTrue(net.pipes[net.pairs[bound].pipe_ids[0]].is_open)


    def test_whole_pipe_pair_and_shared_region_ownership(self):
        net=self.build(grid_n=2)
        # 0°层同一行的两列引用相同的整根管道对。
        left,right="R1-1/L1","R1-2/L1"
        net.apply_layout(left,1)
        new_pair=next(p for p in net.layouts[left].selected_pair_ids if p not in net.layouts[left].initial_pair_ids)
        net.apply_layout(right,1)
        net.apply_layout(left,0)
        self.assertTrue(all(net.pipes[p].is_open for p in net.pairs[new_pair].pipe_ids))
        net.apply_layout(right,0)
        self.assertFalse(any(net.pipes[p].is_open for p in net.pairs[new_pair].pipe_ids))
        for a,b in net.continuations:
            self.assertEqual(net.segments[a].pipe_id,net.segments[b].pipe_id)
        for pair in net.pairs.values():
            self.assertEqual(*(net.pipes[p].is_open for p in pair.pipe_ids))

    def test_retire_waits_for_whole_pipe_count_to_clear(self):
        net=self.build(grid_n=2)
        lid="R1-1/L1"
        net.apply_layout(lid,1)
        extra=next(p for p in net.layouts[lid].selected_pair_ids if p not in net.layouts[lid].initial_pair_ids)
        pipe=net.pipes[net.pairs[extra].pipe_ids[0]]
        far_sid=next(sid for sid in pipe.segment_ids if net.segments[sid].region_id=="R1-2")
        net.segments[far_sid].count=1
        changes=net.apply_layout(lid,0)
        self.assertIn(extra,changes["retained"])
        self.assertTrue(pipe.is_open)
        Controller(net).step(0,{},counts={})
        self.assertFalse(pipe.is_open)

    def test_occupied_descendant_retains_ancestors(self):
        net = self.build()
        lid = 'R1-1/L1'
        net.apply_layout(lid, 4)
        extras = set(net.layouts[lid].selected_pair_ids)-set(net.layouts[lid].initial_pair_ids)
        leaf = max(extras, key=lambda p: net.split_nodes[p].depth)
        sid = net.pipes[net.pairs[leaf].pipe_ids[0]].segment_ids[0]
        net.segments[sid].count = 1
        parent = net.split_nodes[leaf].parent_pair_id
        changes = net.apply_layout(lid, 0)
        self.assertIn(leaf, changes['retained'])
        self.assertIn(parent, changes['retained'])
        ctrl = Controller(net)
        ctrl.step(0, {}, counts={sid: 1})
        self.assertTrue(net.pipes[net.pairs[parent].pipe_ids[0]].is_open)
        ctrl.step(1, {}, counts={})
        self.assertFalse(net.retained_pair_ids)
        self.assertFalse(net.pipes[net.pairs[parent].pipe_ids[0]].is_open)




if __name__ == "__main__":unittest.main()
