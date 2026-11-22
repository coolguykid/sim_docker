"""仅有窗口整数通过量增益时接受超载调速，恢复和连续统计保持原行为。"""
from dataclasses import replace
import math
import unittest

from airway import Config, Controller, ThroughputModel, build_network
from test_simulation import small_network


def network_with_window(seconds=60):
    network = small_network()
    network.config = replace(network.config, throughput_window_s=seconds)
    return network


def specified_throughputs(before, after):
    """合成单调曲线：20m/s时为before，10m/s时达after，仅用于决策边界测试。"""
    return ThroughputModel(spacing_function=lambda speed:
        60*speed/(after+(before-after)*(speed-10)/10))


class IntegerSpeedControlTests(unittest.TestCase):
    def test_default_continuous_gain_does_not_lower_speed_without_integer_gain(self):
        network = network_with_window()
        before = network.segment_throughput('A@R1-1')
        peak = network.throughput_model.throughput(network.throughput_model.best_speed(10,50),60)
        self.assertGreater(peak,before)
        self.assertEqual(math.floor(before),6)
        self.assertEqual(math.floor(peak),6)
        events = Controller(network).step(0,{'A@R1-1':7})
        self.assertFalse(any(event.action=='speed_changed' for event in events))
        self.assertEqual(network.pipes['A'].speed_mps,20)
        self.assertEqual(network.segment_throughput('A@R1-1'),before)
        self.assertEqual(network.segments['A@R1-1'].passed,7)
        self.assertTrue(network.ramps['BA'].congestion_blocked)
        actions = [event.action for event in events]
        self.assertLess(actions.index('ramps_closed'),actions.index('expansion_exhausted'))

    def test_fractional_overload_still_requires_integer_gain_even_if_continuous_gain_would_clear_it(self):
        network = network_with_window()
        events = Controller(network).step(0,{'A@R1-1':6.5})
        self.assertFalse(any(event.action=='speed_changed' for event in events))
        self.assertEqual(network.pipes['A'].speed_mps,20)
        self.assertTrue(any(event.action=='unresolved' for event in events))

    def test_120_second_window_accepts_twelve_to_thirteen_and_keeps_continuous_stats(self):
        network = network_with_window(120)
        network.segments['A@R1-1'].demand = 1000
        events = Controller(network).step(0,{'A@R1-1':13})
        changed = next(event for event in events if event.action=='speed_changed')
        self.assertEqual(changed.details['old_integer_throughput'],12)
        self.assertEqual(changed.details['new_integer_throughput'],13)
        self.assertEqual(changed.details['observed_throughput'],13)
        self.assertEqual(changed.details['window_s'],120)
        self.assertLess(changed.details['new_speed'],20)
        self.assertGreater(changed.details['new_throughput'],13)
        self.assertEqual(network.segment_throughput('A@R1-1'),changed.details['new_throughput'])
        count, capacity = network.region_totals('R1-1',0)
        self.assertEqual(count,13)
        self.assertEqual(capacity,changed.details['new_throughput'])
        self.assertNotEqual(capacity,math.floor(capacity))
        self.assertEqual(network.segments['A@R1-1'].demand,1000)
        self.assertFalse(network.ramps['BA'].congestion_blocked)

    def test_custom_curve_accepts_six_to_seven(self):
        network = network_with_window()
        network.throughput_model = specified_throughputs(6.4,7.2)
        events = Controller(network).step(0,{'A@R1-1':7})
        changed = next(event for event in events if event.action=='speed_changed')
        self.assertEqual(changed.details['old_integer_throughput'],6)
        self.assertEqual(changed.details['new_integer_throughput'],7)
        self.assertGreater(changed.details['new_throughput'],7)
        self.assertEqual(network.segments['A@R1-1'].passed,7)

    def test_integer_gain_is_accepted_even_when_overload_cannot_be_fully_resolved(self):
        network = network_with_window(120)
        events = Controller(network).step(0,{'A@R1-1':14})
        changed = next(event for event in events if event.action=='speed_changed')
        self.assertEqual(changed.details['old_integer_throughput'],12)
        self.assertEqual(changed.details['new_integer_throughput'],13)
        self.assertLess(changed.details['new_throughput'],14)
        self.assertTrue(network.ramps['BA'].congestion_blocked)
        actions = [event.action for event in events]
        self.assertLess(actions.index('speed_changed'),actions.index('ramps_closed'))
        self.assertLess(actions.index('ramps_closed'),actions.index('expansion_exhausted'))
        self.assertEqual(network.segments['A@R1-1'].passed,14)

    def test_skipped_speed_change_still_allows_real_layer_expansion(self):
        network = build_network(Config(boundary_m=3000,grid_n=1,pipes_per_reference_width=24))
        network.initialize()
        pipes = {pid for pid,pipe in network.pipes.items() if pipe.is_open and pipe.layer_id==0}
        passed = {sid:7 for pid in pipes for sid in network.pipes[pid].segment_ids}
        before_count = sum(pipe.is_open for pipe in network.pipes.values())
        events = Controller(network).step(0,passed)
        actions = [event.action for event in events]
        self.assertNotIn('speed_changed',actions)
        self.assertIn('ramps_closed',actions)
        self.assertIn('pairs_expanded',actions)
        self.assertLess(actions.index('ramps_closed'),actions.index('pairs_expanded'))
        self.assertGreater(sum(pipe.is_open for pipe in network.pipes.values()),before_count)
        self.assertTrue(all(network.pipes[pid].speed_mps==20 for pid in pipes))
        self.assertEqual(network.region_totals('R1-1',0)[0],sum(passed.values()))

    def test_idle_speed_recovery_does_not_require_integer_throughput_gain(self):
        network = network_with_window()
        network.pipes['A'].speed_mps = network.throughput_model.best_speed(10,50)
        controller = Controller(network)
        capacities, speeds = [network.segment_throughput('A@R1-1')], [network.pipes['A'].speed_mps]
        for time_s in (0,15,30):
            events = controller.step(time_s,{})
            self.assertTrue(any(event.action=='speed_recovered' and event.details['pipe_id']=='A'
                                for event in events))
            capacities.append(network.segment_throughput('A@R1-1'))
            speeds.append(network.pipes['A'].speed_mps)
        self.assertEqual(speeds[-1],20)
        self.assertTrue(all(new>old for old,new in zip(speeds,speeds[1:])))
        self.assertTrue(all(new<old for old,new in zip(capacities,capacities[1:])))
        self.assertEqual({math.floor(value+1e-9) for value in capacities},{6})

    def test_integer_tolerance_and_original_continuous_gain_tolerance(self):
        cases = (
            ('no_crossing',6.4,6.9,False),
            ('exact_integer',6.4,7.0,True),
            ('new_within_roundoff',6.4,7-0.5e-9,True),
            ('new_below_roundoff',6.4,7-2e-9,False),
            ('old_already_within_roundoff',7-0.5e-9,7.4,False),
            ('old_below_roundoff',7-2e-9,7.4,True),
            ('continuous_gain_too_small',7-1.2e-9,7-0.4e-9,False),
        )
        for name,before,after,accepted in cases:
            with self.subTest(name=name):
                network = network_with_window()
                network.throughput_model = specified_throughputs(before,after)
                events = Controller(network).step(0,{'A@R1-1':10})
                changed = [event for event in events if event.action=='speed_changed']
                self.assertEqual(bool(changed),accepted)
                if accepted:
                    self.assertEqual(changed[0].details['old_integer_throughput'],6)
                    self.assertEqual(changed[0].details['new_integer_throughput'],7)
                else:
                    self.assertEqual(network.pipes['A'].speed_mps,20)


if __name__ == '__main__':
    unittest.main()
