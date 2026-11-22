from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "region_service"))
sys.path.insert(0, str(ROOT / "航道容量项目代码"))

from integration_ports import invoke_network_adjustment
from network_adjustment_adapter import (
    DatabaseControlNetworkAdapter,
    NetworkDatabaseMapping,
)
from topology_adjustments import apply_to_snapshot, merge_adjustments


def _pipe(pipe_id, main_id, layer_id, pipe_type, y, opened):
    start_node, end_node = pipe_id * 2 - 1, pipe_id * 2
    return ({
        "region_id": 1, "version_id": 1, "pipe_id": pipe_id,
        "main_pipe_id": main_id, "layer_id": layer_id, "type": pipe_type,
        "angle": 0.0, "length": 1000.0, "lane_number": 5,
        "start_node_id": start_node, "end_node_id": end_node,
        "is_open": opened, "speed_limit": 20.0, "d_safe": 20.0,
        "max_capacity": 50,
    }, [
        {"node_id": start_node, "position": [0.0, y, 100.0]},
        {"node_id": end_node, "position": [1000.0, y, 100.0]},
    ])


class NetworkAdjustmentAdapterTest(unittest.TestCase):
    def test_partial_updates_merge_and_preserve_unmodified_snapshot_fields(self):
        original = {
            "network": {
                "pipes": [{
                    "pipe_id": 101,
                    "is_open": True,
                    "speed_limit": 20.0,
                    "d_safe": 20.0,
                    "max_capacity": 50,
                }],
                "ramps": [],
            },
        }
        merged = merge_adjustments(
            {"pipes": [{"pipe_id": 101, "speed_limit": 18.0}], "ramps": []},
            {"pipes": [{"pipe_id": 101, "is_open": False}], "ramps": []},
        )
        self.assertEqual(merged["pipes"], [{
            "pipe_id": 101,
            "speed_limit": 18.0,
            "is_open": False,
        }])

        effective = apply_to_snapshot(original, merged)
        self.assertEqual(effective["network"]["pipes"], [{
            "pipe_id": 101,
            "is_open": False,
            "speed_limit": 18.0,
            "d_safe": 20.0,
            "max_capacity": 50,
        }])

    def test_diff_emits_only_fields_changed_by_controller(self):
        mapping = NetworkDatabaseMapping(
            segment_to_pipe_row={
                "segment-101": {
                    "pipe_id": 101,
                    "d_safe": 20.0,
                    "max_capacity": 50,
                },
            },
            pipe_to_segment_ids={"main-pipe": ["segment-101"]},
            ramp_to_row={},
        )
        before = SimpleNamespace(
            pipes={"main-pipe": SimpleNamespace(is_open=True, speed_mps=20.0)},
            ramps={},
        )

        speed_changed = SimpleNamespace(
            pipes={"main-pipe": SimpleNamespace(is_open=True, speed_mps=18.0)},
            ramps={},
        )
        self.assertEqual(
            DatabaseControlNetworkAdapter.diff(before, speed_changed, mapping)["pipes"],
            [{"pipe_id": 101, "speed_limit": 18.0}],
        )

        open_changed = SimpleNamespace(
            pipes={"main-pipe": SimpleNamespace(is_open=False, speed_mps=20.0)},
            ramps={},
        )
        self.assertEqual(
            DatabaseControlNetworkAdapter.diff(before, open_changed, mapping)["pipes"],
            [{"pipe_id": 101, "is_open": False}],
        )

    def test_controller_is_called_and_returns_database_adjustments(self):
        pipes, nodes = [], []
        # G1 and G3 are primary/open anchors; G2 is a closed backup candidate.
        for index, (pipe_type, opened) in enumerate(
                ((0, True), (0, True), (1, False), (1, False),
                 (0, True), (0, True)), start=1):
            pipe, pipe_nodes = _pipe(index, index, 1, pipe_type, index * 100.0, opened)
            pipes.append(pipe)
            nodes.extend(pipe_nodes)
        ramps = [{
            "region_id": 1, "version_id": 1, "ramp_id": 7, "type": 1,
            "length": 100.0, "start_node_id": 12, "end_node_id": 1,
            "ramp_path": [[1000.0, 600.0, 100.0], [0.0, 100.0, 100.0]],
            "is_open": True,
        }, {
            "region_id": 1, "version_id": 1, "ramp_id": 8, "type": 2,
            "length": 100.0, "start_node_id": 12, "end_node_id": 1,
            "ramp_path": [[1000.0, 600.0, 100.0], [0.0, 100.0, 100.0]],
            "is_open": True,
        }]
        snapshot = {
            "region_id": 1, "topology_version": 1,
            "network": {"nodes": nodes, "pipes": pipes, "ramps": ramps, "lanes": []},
        }
        edges = []
        for pipe in pipes:
            edges.append({
                "edge_type": "pipe", "edge_id": pipe["pipe_id"],
                "instant_uav_count": 0,
                "window_out_count": 30.0 if pipe["pipe_id"] == 1 else 0.0,
            })
        result = invoke_network_adjustment({
            "effective_snapshot": snapshot,
            "statistics": [{
                "time": 60.0, "window_seconds": 60.0,
                "warmup": False, "edges": edges,
            }],
            "evaluation": {"severe_congestion": [
                {"edge": "pipe:1", "duration_seconds": 15.0, "time_s": 60.0}
            ]},
        })
        changed_pipes = {row["pipe_id"]: row for row in result["pipes"]}
        self.assertTrue(changed_pipes[3]["is_open"])
        self.assertTrue(changed_pipes[4]["is_open"])
        for change in result["pipes"]:
            self.assertNotIn("d_safe", change)
            self.assertNotIn("max_capacity", change)
            self.assertTrue(
                "is_open" in change or "speed_limit" in change,
                f"pipe {change['pipe_id']} contains no actual change",
            )
        self.assertEqual(result["ramps"], [{"ramp_id": 7, "is_open": False}])
        self.assertNotIn(8, {row["ramp_id"] for row in result["ramps"]})
        self.assertTrue(result["control"]["events"])


if __name__ == "__main__":
    unittest.main()
