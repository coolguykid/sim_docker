import random
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from coordinator_service import app as coordinator

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "region_service"))
from integration_ports import evaluate_statistics


class OnlineCongestionTest(unittest.IsolatedAsyncioTestCase):
    async def test_coordinator_stops_after_all_regions_write_trigger_frame(self):
        trigger = random.Random(42).choice(range(205, 600, 5))
        for batch_mode in ("0", "1"):
            with self.subTest(batch_mode=batch_mode):
                published = {1: [], 2: []}
                record = {
                    "run_id": "test", "attempt_id": 0,
                    "step_seconds": 1, "horizon_seconds": 600,
                    "regions": {1: {"endpoint": "http://1"}, 2: {"endpoint": "http://2"}},
                    "expected_region_ids": [1, 2],
                    "handoff_count": 0, "handoff_pairs": {},
                }

                async def post(url, payload):
                    region = int(url.split("/")[2])
                    if url.endswith("batch/prepare"):
                        return {"events": []}
                    if url.endswith("batch/apply"):
                        return {"matches": True}
                    if url.endswith("events/prepare"):
                        return {"prepare_token": "frame"}
                    if url.endswith("events/commit"):
                        return {}
                    self.assertTrue(url.endswith("events/frame"))
                    time_s = payload["frame_time"]
                    published[region].append(time_s)
                    if region == 2 and time_s == trigger:
                        return {"congestion": {"decision": "adjustment_required"}}
                    return {}

                with patch.dict("os.environ", {"SIM_FRAME_BATCH": batch_mode}), patch.object(
                    coordinator, "_post", side_effect=post
                ):
                    await coordinator.drive_attempt(record)
                self.assertEqual(record["status"], "simulation_completed")
                self.assertEqual(record["completed_steps"], trigger)
                self.assertEqual(record["congestion"]["time_s"], trigger)
                self.assertEqual(set(record["congestion"]["regions"]), {"2"})
                self.assertEqual(published[1], published[2])
                self.assertEqual(published[1][-1], trigger)

    def test_threshold_accumulates_across_statistics_batches(self):
        consecutive = {}
        edge = {
            "edge_type": "pipe", "edge_id": 42, "max_capacity": 10,
            "window_average_uav_count": 9, "speed_limit": 10,
            "window_space_mean_speed": 2,
        }
        for time_s, warmup, expected in (
            (55, True, "acceptable"), (60, False, "acceptable"),
            (65, False, "acceptable"), (70, False, "adjustment_required"),
        ):
            evaluation = evaluate_statistics(
                {"statistics": [{"time": time_s, "warmup": warmup, "edges": [edge]}]},
                {},
                consecutive=consecutive,
            )
            self.assertEqual(evaluation["decision"], expected)
        self.assertEqual(evaluation["severe_congestion"][0]["time_s"], 70)
