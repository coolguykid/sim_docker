"""反向寻路的行为核对；完整前向搜索作为独立参考。"""
import random
import unittest

from airway import Config, Station, ThroughputModel
from airway.model import Layer, Network, Pipe, Ramp, Region, Segment
from airway.simulation import FlightGraph


def network_with_pipes(names, cuts=(25, 50, 75, 100)):
    net = Network(Config(), ThroughputModel())
    for index, pid in enumerate(names):
        z = 135 + 130 * index
        net.layers[index] = Layer(index, 0, z)
        pipe = Pipe(pid, index, 1, pid, 1, (0, 0, z), (100, 0, z), 20, True, [])
        net.pipes[pid] = pipe
        lower = 0
        for number, upper in enumerate(cuts, 1):
            rid = f'R1-{number}'
            if rid not in net.regions:
                net.regions[rid] = Region(rid, 0, number - 1)
            sid = f'{pid}@{rid}'
            net.segments[sid] = Segment(sid, pid, rid, (lower, 0, z), (upper, 0, z), upper-lower)
            net.regions[rid].segment_ids.append(sid)
            net.incoming[sid] = []
            pipe.segment_ids.append(sid)
            lower = upper
    return net


def add_ramp(net, name, source, target, start, end):
    def segment(pid, position):
        for sid in net.pipes[pid].segment_ids:
            if position < net.segments[sid].end[0]-1e-6:
                return sid
        return net.pipes[pid].segment_ids[-1]
    source_sid, target_sid = segment(source, start), segment(target, end)
    net.ramps[name] = Ramp(name, source_sid, target_sid, 'interlayer',
                           (start, 0, net.pipes[source].start[2]),
                           (end, 0, net.pipes[target].start[2]))
    net.incoming[target_sid].append(name)


def flown_distance(source, start, route):
    distance = 0.0
    for target, end in route:
        if source == target:
            distance += max(0.0, end-start)
        source, start = target, end
    return round(distance, 6)


class ReverseRoutingTests(unittest.TestCase):
    def assert_valid_route(self, graph, source, start, goal, route, allow_origin):
        escape = graph._escape_segment(source, start, allow_origin)
        for target, end in route:
            if source == target:
                self.assertGreaterEqual(end, start-1e-6)
                self.assertTrue(graph._range_allowed(source, start, end, escape))
            else:
                previous = graph.index.get((source, round(start, 6)))
                following = graph.index.get((target, round(end, 6)))
                self.assertIsNotNone(previous)
                self.assertIsNotNone(following)
                self.assertIn((following, 0.0), graph.edges[previous])
                escape = None
            source, start = target, end
        self.assertEqual(source, goal.pipe_id)
        self.assertAlmostEqual(start, goal.distance_m, places=6)
        self.assertEqual(len(route), len(set(route)), '最短路线不应包含零权环或重复节点')

    def test_reverse_routes_match_forward_oracle_for_directed_masks_and_internal_points(self):
        # 不比较并列路线的具体选择，只核对可达性、飞行长度及合法通行。
        for seed in range(12):
            rng = random.Random(seed)
            net = network_with_pipes('ABCDEF')
            for index in range(24):
                source, target = rng.sample('ABCDEF', 2)
                add_ramp(net, f'r{index}', source, target,
                         rng.randrange(8)*12.5, rng.randrange(8)*12.5)
            segment_ids = list(net.segments)
            masks = ((), rng.sample(segment_ids, 5), rng.sample(segment_ids, 12))
            for blocked in masks:
                graph = FlightGraph(net, [], blocked)
                for query in range(72):
                    source, target = rng.choices('ABCDEF', k=2)
                    start = rng.randrange(801)/8
                    goal = Station(query, target, rng.randrange(801)/8, 'unused')
                    allow_origin = bool(query % 2)
                    expected = graph.finish(source, start, goal,
                        graph.search(source, start, random.Random(query), allow_origin=allow_origin),
                        allow_origin=allow_origin)
                    actual = graph.route(source, start, goal, random.Random(query), allow_origin=allow_origin)
                    with self.subTest(seed=seed, blocked=blocked, query=query, source=source,
                                      start=start, goal=goal, allow_origin=allow_origin):
                        self.assertEqual(actual is None, expected is None)
                        if actual is not None:
                            self.assertAlmostEqual(flown_distance(source, start, actual),
                                                   flown_distance(source, start, expected), places=6)
                            self.assert_valid_route(graph, source, start, goal, actual, allow_origin)

    def test_zero_cost_cycles_do_not_prevent_independent_random_tied_routes(self):
        net = network_with_pipes('ABCD', cuts=(100,))
        for name, source, target, start, end in (
                ('AB', 'A', 'B', 20, 20), ('AC', 'A', 'C', 20, 20),
                ('BC', 'B', 'C', 20, 20), ('CB', 'C', 'B', 20, 20),
                ('BD', 'B', 'D', 80, 80), ('CD', 'C', 'D', 80, 80)):
            add_ramp(net, name, source, target, start, end)
        graph = FlightGraph(net, [])
        goal = Station(0, 'D', 90, 'unused')
        routes = []
        for seed in range(40):
            route = graph.route('A', 10, goal, random.Random(seed))
            self.assertEqual(route, graph.route('A', 10, goal, random.Random(seed)))
            self.assertEqual(flown_distance('A', 10, route), 80)
            self.assert_valid_route(graph, 'A', 10, goal, route, True)
            routes.append(tuple(route))
        self.assertGreater(len(set(routes)), 1, '共享距离表不能使所有并列路线固定成一条')

    def test_micro_precision_edges_and_arbitrary_submicron_endpoints(self):
        net = network_with_pipes('ABCD', cuts=(17.123457, 58.234568, 100))
        for name, source, target, start, end in (
                ('AB', 'A', 'B', 17.234569, 2.987653),
                ('BC', 'B', 'C', 44.765431, 26.876543),
                ('CD', 'C', 'D', 66.543219, 8.123456),
                ('DA', 'D', 'A', 88.456781, 4.234567)):
            add_ramp(net, name, source, target, start, end)
        rng = random.Random(31)
        for blocked in ((), {'A@R1-2'}, {'B@R1-2', 'D@R1-1'}):
            graph = FlightGraph(net, [], blocked)
            for query in range(120):
                source, target = rng.choices('ABCD', k=2)
                start = rng.random()*100
                goal = Station(query, target, rng.random()*100, 'unused')
                expected = graph.finish(source, start, goal,
                    graph.search(source, start, random.Random(query)))
                actual = graph.route(source, start, goal, random.Random(query))
                self.assertEqual(actual is None, expected is None)
                if actual is not None:
                    # 图保持原有微米精度；管内当前位置和最终终点保留原始小数。
                    self.assertAlmostEqual(flown_distance(source, start, actual),
                                           flown_distance(source, start, expected), delta=1e-6)
                    self.assert_valid_route(graph, source, start, goal, actual, True)

    def test_origin_escape_does_not_leak_into_other_sources_or_backward_targets(self):
        net = network_with_pipes('AB', cuts=(50, 100))
        add_ramp(net, 'escape', 'A', 'B', 80, 80)
        add_ramp(net, 'loop', 'B', 'A', 100, 0)
        graph = FlightGraph(net, [], {'A@R1-2'})
        end = Station(0, 'A', 100, 'unused')
        self.assertEqual(graph.route('A', 60, end, random.Random(1)), [('A', 100)])
        self.assertIsNone(graph.route('A', 10, end, random.Random(1)))
        self.assertIsNone(graph.route('B', 90, end, random.Random(1)))
        self.assertIsNone(graph.route('A', 60, Station(0, 'A', 55, 'unused'), random.Random(1)))
        self.assertIsNotNone(graph.route('A', 60, Station(0, 'B', 100, 'unused'), random.Random(1)))

    def test_internal_destination_tail_is_checked_per_query(self):
        net = network_with_pipes('AB', cuts=(50, 100))
        add_ramp(net, 'AB', 'A', 'B', 20, 80)
        graph = FlightGraph(net, [])
        for position, expected in ((90, 20), (85, 15), (80, 10)):
            route = graph.route('A', 10, Station(0, 'B', position, 'unused'), random.Random(1))
            self.assertEqual(flown_distance('A', 10, route), expected)
        self.assertIsNone(graph.route('A', 10, Station(0, 'B', 70, 'unused'), random.Random(1)))
        blocked = FlightGraph(net, [], {'B@R1-2'})
        self.assertIsNone(blocked.route('A', 10, Station(0, 'B', 90, 'unused'), random.Random(1)))
        self.assertEqual(blocked.route('B', 82, Station(0, 'B', 87, 'unused'), random.Random(1)), [('B', 87)])

    def test_destination_cache_reused_within_graph_and_rebuilt_after_graph_change(self):
        net = network_with_pipes('AB', cuts=(100,))
        add_ramp(net, 'AB', 'A', 'B', 20, 80)
        goal = Station(0, 'B', 100, 'unused')
        graph = FlightGraph(net, [])
        before_searches = graph.routing_stats['reverse_searches']
        before_hits = graph.routing_stats['cache_hits']
        first = graph.route('A', 10, goal, random.Random(1))
        second = graph.route('A', 11, goal, random.Random(2))
        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        self.assertEqual(graph.routing_stats['reverse_searches']-before_searches, 1)
        self.assertGreaterEqual(graph.routing_stats['cache_hits']-before_hits, 1)
        net.ramps['AB'].enabled = False
        changed = FlightGraph(net, [])
        self.assertEqual(changed.routing_stats['reverse_searches'], 0)
        self.assertIsNone(changed.route('A', 10, goal, random.Random(1)))
        net.ramps['AB'].enabled = True
        reopened = FlightGraph(net, [])
        self.assertIsNotNone(reopened.route('A', 10, goal, random.Random(1)))
        blocked = FlightGraph(net, [], {'B@R1-1'})
        self.assertIsNone(blocked.route('A', 10, goal, random.Random(1)))


if __name__ == '__main__':
    unittest.main()
