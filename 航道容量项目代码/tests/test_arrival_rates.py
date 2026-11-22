"""入口频率随当前速度/安全距离更新，指数风险量跨变速时刻连续。"""
import math
import unittest

from airway import DroneSimulation, ThroughputModel
from airway.simulation import ArrivalClock
from test_simulation import small_network


def single_entry_simulation(alpha=2, spacing=80, control_interval=1000):
    network = small_network()
    network.throughput_model = ThroughputModel(spacing_function=lambda _speed:spacing)
    simulation = DroneSimulation(network, adaptive=False, arrival_alpha=0,
                                 entry_alphas={'A':alpha}, control_interval_s=control_interval)
    simulation.destination_options['A'] = ('A',)
    return simulation


class ArrivalRateTests(unittest.TestCase):
    def test_default_rates_match_current_speed_spacing_and_truncation_mean(self):
        sim = DroneSimulation(small_network(), adaptive=False)
        self.assertEqual(sim.arrival_alpha, .5)
        parameters = sim.entry_parameters('A')
        self.assertEqual(parameters['speed_mps'], 20)
        self.assertAlmostEqual(parameters['safety_distance_m'], 187.4)
        self.assertAlmostEqual(parameters['headway_s'], 9.37)
        self.assertAlmostEqual(parameters['lambda0_per_s'], 20/187.4)
        self.assertAlmostEqual(parameters['lambda_per_s'], .5*20/187.4)
        self.assertAlmostEqual(parameters['mean_rate_per_s'], (1/3)*20/187.4)
        self.assertLess(parameters['mean_rate_per_s'], parameters['lambda0_per_s'])
        self.assertEqual(sim.entry_lambdas, {'A':parameters['lambda_per_s'], 'B':parameters['lambda_per_s']})

    def test_zero_alpha_disables_requests_and_entry_clocks(self):
        sim = DroneSimulation(small_network(), adaptive=False, arrival_alpha=0)
        self.assertFalse(sim.entries)
        self.assertEqual(sim.entry_lambdas, {'A':0, 'B':0})
        sim.step(200)
        self.assertFalse(sim.arrival_log)
        self.assertEqual(sim.generated, 0)
        self.assertEqual(sim.entered, 0)

    def test_per_entry_alpha_overrides_only_that_entry(self):
        sim = DroneSimulation(small_network(), adaptive=False, arrival_alpha=2,
                              entry_alphas={'A':0, 'B':4})
        self.assertEqual(set(sim.entries), {'B'})
        self.assertEqual(sim.entry_parameters('A')['alpha'], 0)
        self.assertEqual(sim.entry_parameters('B')['alpha'], 4)
        self.assertAlmostEqual(sim.entry_lambdas['B'], 4/9.37)
        sim.step(40)
        self.assertTrue(sim.arrival_log)
        self.assertEqual({entry['pipe_id'] for entry in sim.arrival_log}, {'B'})

    def test_invalid_alpha_and_unknown_entry_are_rejected(self):
        for value in (-1, math.inf, math.nan):
            with self.assertRaises(ValueError):
                DroneSimulation(small_network(), arrival_alpha=value)
            with self.assertRaises(ValueError):
                DroneSimulation(small_network(), entry_alphas={'A':value})
        with self.assertRaises(ValueError):
            DroneSimulation(small_network(), entry_alphas={'missing':2})

    def test_custom_spacing_and_speed_change_update_all_derived_parameters(self):
        sim = single_entry_simulation()
        parameters = sim.entry_parameters('A')
        self.assertEqual(parameters['safety_distance_m'], 80)
        self.assertEqual(parameters['headway_s'], 4)
        self.assertEqual(parameters['lambda0_per_s'], .25)
        self.assertEqual(parameters['lambda_per_s'], .5)
        sim.network.pipes['A'].speed_mps = 40
        parameters = sim.entry_parameters('A')
        self.assertEqual(parameters['safety_distance_m'], 80)
        self.assertEqual(parameters['headway_s'], 2)
        self.assertEqual(parameters['lambda0_per_s'], .5)
        self.assertEqual(parameters['lambda_per_s'], 1)
        self.assertEqual(sim.entry_lambdas['A'], 1)
        sim.network.throughput_model = ThroughputModel(spacing_function=lambda _speed:160)
        parameters = sim.entry_parameters('A')
        self.assertEqual(parameters['headway_s'], 4)
        self.assertEqual(parameters['lambda_per_s'], .5)
        self.assertEqual(sim.entry_lambdas['A'], .5)

    def test_constant_speed_clock_preserves_first_request_time_across_steps(self):
        sim = single_entry_simulation()
        sim.entries['A'] = ArrivalClock(0, 0, 4)
        self.assertEqual(sim._next_entry('A'), 12)
        sim.step(2)
        self.assertEqual(sim.entries['A'].remaining_hazard, 4)
        self.assertEqual(sim._next_entry('A'), 12)
        sim.step(4)
        self.assertEqual(sim.entries['A'].remaining_hazard, 3)
        self.assertEqual(sim._next_entry('A'), 12)
        sim.step(6.1)
        self.assertEqual(sim.arrival_log[0]['time_s'], 12)
        self.assertEqual(sim.arrival_log[0]['headway_s'], 4)
        self.assertEqual(sim.arrival_log[0]['lambda_per_s'], .5)

    def test_speed_increase_keeps_consumed_hazard_without_retroactive_requests(self):
        sim = single_entry_simulation()
        sim.entries['A'] = ArrivalClock(0, 0, 4)
        sim.step(6)
        self.assertEqual(sim.entries['A'].remaining_hazard, 3)
        sim.network.pipes['A'].speed_mps = 40
        self.assertEqual(sim._next_entry('A'), 9)
        self.assertGreaterEqual(sim._next_entry('A'), sim.time_s)
        sim.step(2)
        self.assertFalse(sim.arrival_log)
        sim.step(1.1)
        entry = sim.arrival_log[0]
        self.assertEqual(entry['time_s'], 9)
        self.assertEqual(entry['speed_mps'], 40)
        self.assertEqual(entry['headway_s'], 2)
        self.assertEqual(entry['lambda_per_s'], 1)

    def test_speed_decrease_applies_new_minimum_without_restarting_hazard(self):
        sim = single_entry_simulation()
        sim.entries['A'] = ArrivalClock(0, 0, 4)
        sim.step(6)
        sim.network.pipes['A'].speed_mps = 10
        self.assertEqual(sim._next_entry('A'), 20)
        sim.step(7)
        self.assertEqual(sim.entries['A'].remaining_hazard, 1.75)
        self.assertEqual(sim._next_entry('A'), 20)
        self.assertFalse(sim.arrival_log)
        sim.step(7.1)
        entry = sim.arrival_log[0]
        self.assertEqual(entry['time_s'], 20)
        self.assertEqual(entry['speed_mps'], 10)
        self.assertEqual(entry['headway_s'], 8)
        self.assertEqual(entry['lambda_per_s'], .25)

    def test_increased_minimum_wait_does_not_consume_hazard_until_eligible(self):
        sim = single_entry_simulation()
        sim.entries['A'] = ArrivalClock(0, 0, 2)
        sim.step(2)
        sim.network.pipes['A'].speed_mps = 5
        self.assertEqual(sim._next_entry('A'), 32)
        sim.step(8)
        self.assertEqual(sim.entries['A'].remaining_hazard, 2)
        sim.step(10)
        self.assertEqual(sim.entries['A'].remaining_hazard, 1.5)
        self.assertEqual(sim._next_entry('A'), 32)
        sim.step(12.1)
        self.assertEqual(sim.arrival_log[0]['time_s'], 32)

    def test_existing_clock_can_pause_and_resume_without_redrawing_hazard(self):
        sim = single_entry_simulation()
        sim.entries['A'] = ArrivalClock(0, 0, 4)
        sim.step(6)
        sim.entry_alphas['A'] = 0
        self.assertTrue(math.isinf(sim._next_entry('A')))
        sim.step(10)
        self.assertEqual(sim.entries['A'].remaining_hazard, 3)
        self.assertFalse(sim.arrival_log)
        sim.entry_alphas['A'] = 4
        self.assertEqual(sim._next_entry('A'), 19)
        sim.step(3.1)
        self.assertEqual(sim.arrival_log[0]['time_s'], 19)
        self.assertEqual(sim.arrival_log[0]['alpha'], 4)

    def test_controller_speed_change_only_affects_future_hazard(self):
        sim = single_entry_simulation(control_interval=5)
        sim.entries['A'] = ArrivalClock(0, 0, 4)
        class SpeedChange:
            def step(self, time_s, passed, *, counts):
                sim.network.pipes['A'].speed_mps = 40
                return []
        sim.controller = SpeedChange()
        sim.step(10)
        # t=4开始累计，t=5前消耗0.5；剩余3.5以新λ=1累计至t=8.5。
        self.assertEqual(sim.arrival_log[0]['time_s'], 8.5)
        self.assertEqual(sim.arrival_log[0]['speed_mps'], 40)

    def test_current_recorded_headways_bound_all_generated_intervals_after_speed_changes(self):
        sim = single_entry_simulation()
        previous_request = 0.0
        for speed in (20, 40, 10, 30, 5, 20):
            sim.network.pipes['A'].speed_mps = speed
            begin = len(sim.arrival_log)
            sim.step(40)
            for entry in sim.arrival_log[begin:]:
                self.assertEqual(entry['speed_mps'], speed)
                self.assertGreaterEqual(entry['time_s']-previous_request,
                                        entry['headway_s']-1e-8)
                self.assertAlmostEqual(entry['lambda0_per_s'],
                                       entry['speed_mps']/entry['safety_distance_m'])
                self.assertAlmostEqual(entry['lambda_per_s'],
                                       entry['alpha']*entry['lambda0_per_s'])
                previous_request = entry['time_s']
        self.assertGreater(len(sim.arrival_log), 10)

    def test_step_partition_roundoff_does_not_change_same_task_tied_route(self):
        from test_reverse_routing import add_ramp, network_with_pipes
        def simulation():
            network = network_with_pipes('ABCD', cuts=(100,))
            for name, source, target, start, end in (
                    ('AB', 'A', 'B', 20, 20), ('AC', 'A', 'C', 20, 20),
                    ('BD', 'B', 'D', 80, 80), ('CD', 'C', 'D', 80, 80)):
                add_ramp(network, name, source, target, start, end)
            sim = DroneSimulation(network, adaptive=False, arrival_alpha=0,
                                  entry_alphas={'A':2}, control_interval_s=1000)
            sim.destination_options['A'] = ('D',)
            return sim
        large, split = simulation(), simulation()
        large.step(300)
        remaining = 300
        while remaining > 1e-8:
            duration = min(.31, remaining)
            split.step(duration)
            remaining -= duration
        def tasks(sim):
            return [(entry['id'], entry['pipe_id'], entry['destination_pipe_id'])
                    for entry in sim.arrival_log]
        def pipe_visits(sim, identity):
            return tuple(dict.fromkeys(pid for _, pid, _, _ in sim.tracks[identity]))
        self.assertTrue(large.arrival_log)
        self.assertEqual(tasks(large), tasks(split))
        self.assertEqual(set(large.tracks), set(split.tracks))
        for first, second in zip(large.arrival_log, split.arrival_log):
            with self.subTest(identity=first['id']):
                self.assertLess(abs(first['time_s']-second['time_s']), 1e-9)
                self.assertEqual(pipe_visits(large, first['id']),
                                 pipe_visits(split, second['id']))
        self.assertGreater(len({pipe_visits(large, identity) for identity in large.tracks}), 1)


if __name__ == '__main__':
    unittest.main()
