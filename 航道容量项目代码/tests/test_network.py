import math
import unittest
from airway import Config, ThroughputModel, build_network, Controller


class NetworkTests(unittest.TestCase):



    def setUp(self):
        self.net = build_network(Config(boundary_m=3000, grid_n=2,
                                        pipes_per_reference_width=24))

    def active_target(self):
        return next(sid for sid, seg in self.net.segments.items()
                    if self.net.pipes[seg.pipe_id].is_open and any(self.net.ramp_is_open(r) for r in self.net.incoming[sid]))

    def test_layers_geometry_and_continuity(self):
        n = self.net
        self.assertEqual([l.angle_deg for l in n.layers.values()], list(range(0, 180, 30)))
        for p in n.pipes.values():
            self.assertEqual(p.direction, 1 if p.number % 2 else -1)
            length = sum(n.segments[s].length_m for s in p.segment_ids)
            self.assertAlmostEqual(length, math.dist(p.start, p.end), places=6)
            for sid in p.segment_ids:
                s = n.segments[sid]
                r = n.regions[s.region_id]
                x, y = [(a+b)/2 for a,b in zip(s.start[:2],s.end[:2])]
                self.assertTrue(-1500+r.col*1500 <= x <= -1500+(r.col+1)*1500)
                self.assertTrue(-1500+r.row*1500 <= y <= -1500+(r.row+1)*1500)
        self.assertGreater(len(n.continuations), 0)
        for a,b in n.continuations:
            self.assertLess(math.dist(n.segments[a].end, n.segments[b].start), 1e-6)
            self.assertEqual(n.segments[a].pipe_id, n.segments[b].pipe_id)
        for ramp in n.ramps.values():
            if ramp.kind == "interlayer":
                self.assertEqual(ramp.start[:2], ramp.end[:2])
                self.assertNotEqual(ramp.start[2], ramp.end[2])
            else:
                self.assertEqual(ramp.start[2], ramp.end[2])
            for endpoint, sid in ((ramp.start, ramp.source_segment), (ramp.end, ramp.target_segment)):
                s = n.segments[sid]
                self.assertAlmostEqual(math.dist(s.start, endpoint)+math.dist(endpoint, s.end), s.length_m, places=5)

    def test_pair_initialization(self):
        n = self.net
        self.assertFalse(any(p.is_open for p in n.pipes.values()))
        first = next(iter(n.pairs))
        n.initialize([first])
        self.assertEqual(sum(p.is_open for p in n.pipes.values()), 2)
        n.set_pair_open(first, False)
        self.assertFalse(any(p.is_open for p in n.pipes.values()))
        n.initialize()
        for pair in n.pairs.values():
            self.assertEqual(*(n.pipes[p].is_open for p in pair.pipe_ids))



    def test_occupied_pipe_cannot_be_closed(self):
        n=self.net;pid=next(iter(n.pairs));n.initialize([pid])
        pipe=n.pipes[n.pairs[pid].pipe_ids[0]]
        n.segments[pipe.segment_ids[0]].count=1
        with self.assertRaises(ValueError):n.set_pair_open(pid,False)
        self.assertTrue(pipe.is_open)






    def test_configuration_rejects_invalid_geometry(self):
        for kwargs in ({"initial_speed_mps":0},{"pipes_per_reference_width":85},
                       {"pipes_per_reference_width":100},{"reopen_ratio":1},
                       {"boundary_m":float("nan")},{"grid_n":2.5}):
            with self.assertRaises(ValueError):Config(**kwargs)


if __name__ == "__main__":
    unittest.main()
