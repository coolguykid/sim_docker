import json
import math
import unittest

from airway import Config, ThroughputModel, build_network


class SpacingInterfaceTests(unittest.TestCase):
    def test_default_interface_preserves_original_formula_and_optimum(self):
        model = ThroughputModel()
        for speed in (10, 14.86, 20, 30, 50):
            expected = 5+1.6*(30+1.2*speed+.6*speed**2/(2*2))
            self.assertAlmostEqual(model.safety_distance(speed), expected)
            self.assertEqual(model.safe_spacing(speed), model.safety_distance(speed))
            self.assertAlmostEqual(model.headway(speed), expected/speed)
            self.assertAlmostEqual(model.throughput(speed, 60), 60*speed/expected)
        self.assertEqual(model.best_speed(10, 50), math.sqrt(53/.24))
        self.assertEqual(model.best_speed(30, 50), 30)
        demand = model.throughput(18, 60)
        self.assertAlmostEqual(model.speed_for_demand(demand, 60, 10, 50), 18)

    def test_custom_distance_replaces_entire_spacing_without_extra_factors(self):
        model = ThroughputModel(spacing_function=lambda v: 100+v**2)
        self.assertEqual(model.safety_distance(20), 500)
        self.assertEqual(model.safe_spacing(20), 500)
        self.assertEqual(model.headway(20), 25)
        self.assertEqual(model.throughput(20, 60), 2.4)
        self.assertAlmostEqual(model.best_speed(5, 50), 10, places=5)
        demand = model.throughput(20, 60)
        self.assertAlmostEqual(model.speed_for_demand(demand, 60, 5, 50), 20, places=6)

    def test_multiple_peaks_and_fastest_feasible_island(self):
        def rate(v):
            return 1+2*math.exp(-((v-16)/1.2)**2)+1.6*math.exp(-((v-39)/1.7)**2)

        model = ThroughputModel(spacing_function=lambda v: v/rate(v))
        self.assertAlmostEqual(model.best_speed(10, 50), 16, places=5)
        expected = 39+1.7*math.sqrt(math.log(1.6))
        self.assertAlmostEqual(model.speed_for_demand(120, 60, 10, 50), expected, places=5)
        self.assertAlmostEqual(model.speed_for_demand(1000, 60, 10, 50), 16, places=5)
        self.assertEqual(model.speed_for_demand(0, 60, 10, 50), 50)

    def test_custom_endpoint_and_single_speed_ranges(self):
        model = ThroughputModel(spacing_function=lambda _: 100)
        self.assertEqual(model.best_speed(10, 50), 50)
        self.assertEqual(model.speed_for_demand(1000, 60, 10, 50), 50)
        self.assertEqual(model.best_speed(20, 20), 20)
        self.assertEqual(model.speed_for_demand(1000, 60, 20, 20), 20)
        self.assertEqual(model.speed_for_demand(1, 60, 20, 20), 20)

    def test_invalid_speed_and_custom_results_are_rejected(self):
        model = ThroughputModel(spacing_function=lambda _: 100)
        for speed in (0, -1, float("inf"), float("nan"), True, None, "20"):
            with self.subTest(speed=speed):
                with self.assertRaises(ValueError):
                    model.safety_distance(speed)
        for value in (0, -1, float("inf"), float("nan"), True, None, "100"):
            invalid = ThroughputModel(spacing_function=lambda _, result=value: result)
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    invalid.headway(20)
        with self.assertRaises(ValueError):
            ThroughputModel(spacing_function=1)
        for lower, upper in ((20, 10), (0, 20), (20, float("inf")), (True, 20)):
            with self.assertRaises(ValueError):
                model.best_speed(lower, upper)
        for spacing, speed in ((1e-300, 1e300), (1e300, 1e-300)):
            extreme = ThroughputModel(spacing_function=lambda _, result=spacing: result)
            with self.assertRaises(ValueError):
                extreme.headway(speed)

    def test_json_metadata_and_network_export_exclude_callable(self):
        class CustomSpacing:
            def __call__(self, speed):
                return 50+speed

            def __deepcopy__(self, memo):
                raise AssertionError("导出时不应复制函数对象")

        model = ThroughputModel(spacing_function=CustomSpacing())
        self.assertEqual(model.metadata()["model_name"], "CustomSpacing")
        self.assertNotIn("spacing_function", model.metadata())
        json.dumps(model.metadata())
        net = build_network(Config(boundary_m=3000, grid_n=1, pipes_per_reference_width=24),
                            throughput_model=model)
        net.initialize()
        data = net.to_dict()
        self.assertEqual(data["throughput_model"], model.metadata())
        json.dumps(data)
        sid = next(s for s in net.segments if net.pipes[net.segments[s].pipe_id].is_open)
        self.assertAlmostEqual(net.segment_throughput(sid), 60*20/70)


if __name__ == "__main__":
    unittest.main()
