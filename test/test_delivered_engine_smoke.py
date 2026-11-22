"""Real-data checks for complete delivered centralized_regional integration."""
from __future__ import annotations

import csv
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "无人机_大都会36区_导引版" / "src"))
sys.path.insert(1, str(ROOT))

from simulation_engine import LocalSimulationSession
from simulation_engine.delivered_adapter import (
    build_delivered_topology, build_global_partition_graph)


def _array(value, cast=float):
    text = str(value or "").replace("{", "").replace("}", "")
    return [cast(item) for item in text.split(",") if item]


def _rows(region_id, name):
    path = ROOT / "region_data" / "region_{:03d}".format(region_id) / name
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def _network(region_id):
    nodes = _rows(region_id, "node_info.csv")
    for row in nodes:
        row["node_id"] = int(row["node_id"])
        row["position"] = _array(row["position"])
        row["region_ids"] = _array(row["region_ids"], int)
        row["layer_id"] = int(row["layer_id"]) if row["layer_id"] else None
    pipes = _rows(region_id, "pipe_info.csv")
    for row in pipes:
        for field in ("pipe_id", "start_node_id", "end_node_id", "layer_id", "lane_number"):
            row[field] = int(row[field])
        row["length"] = float(row["length"])
        row.update(is_open=True, speed_limit=15.0, max_capacity=50)
    lanes = _rows(region_id, "lane_info.csv")
    for row in lanes:
        row["lane_id"], row["pipe_id"] = int(row["lane_id"]), int(row["pipe_id"])
        row["start_pt"], row["end_pt"] = _array(row["start_pt"]), _array(row["end_pt"])
    ramps = _rows(region_id, "ramp_info.csv")
    for row in ramps:
        for field in ("ramp_id", "start_node_id", "end_node_id", "lane_number"):
            row[field] = int(row[field])
        row["length"], row["is_open"] = float(row["length"]), True
        flat = _array(row["ramp_path"])
        row["ramp_path"] = [flat[i:i+3] for i in range(0, len(flat), 3)]
    return {"nodes": nodes, "pipes": pipes, "lanes": lanes, "ramps": ramps}


def _scenario():
    networks = {1: _network(1), 2: _network(2)}
    graph = build_global_partition_graph(networks, 1)
    boundary_id, transition = next(
        (edge_id, item) for edge_id, item in graph.metadata["boundary_transitions"].items()
        if item["source_region"] == 1 and item["target_region"] == 2)
    boundary = graph.edges[boundary_id]
    source = build_delivered_topology(1, 1, networks[1])
    target = build_delivered_topology(2, 1, networks[2])
    source_edge = next(edge for edge in source.graph.edges.values()
                       if edge.target == boundary.source and edge.kind != "BOUNDARY")
    target_edge = next(edge for edge in target.graph.edges.values()
                       if edge.source == boundary.target and edge.kind != "BOUNDARY")
    transfer = {
        "source_region_id": 1,
        "source_layer": transition["source_layer"],
        "target_region_id": 2,
        "target_layer": transition["target_layer"],
        "proportion": 1.0,
    }
    mission = {
        "uav_id": 1, "region_id": 1,
        "node_id": source.node_external_ids[source_edge.source],
        "current_edge": {"edge_type": "pipe", "edge_id": source_edge.pipe_id},
        "lane_id": 0, "distance_on_edge": max(0., source_edge.length_m-1.),
        "speed": 1., "acceleration": 0.,
        "destination_region_id": 2,
        "destination_node_id": target.node_external_ids[target_edge.target],
    }
    return networks, graph, transition, mission


def _config(tmp_path, region_id, network, graph, transfers, uavs):
    return {
        "run_id": "complete-centralized-regional", "attempt_id": 0,
        "region_id": region_id, "topology_version": 1,
        "network": network, "uavs": uavs, "step_seconds": 1.,
        "global_routing_graph": graph,
        "global_guidance_transfers": transfers,
        "guidance_revision": 1,
        "output_dir": str(tmp_path),
    }


def test_official_mode_selects_boundary_without_custom_portal_choice(tmp_path):
    networks, graph, transition, mission = _scenario()
    session = LocalSimulationSession(
        _config(tmp_path, 1, networks[1], graph, [{**transition,
            "source_region_id": transition["source_region"],
            "target_region_id": transition["target_region"],
            "proportion": 1.0}], [mission]))
    try:
        summary = session.summary()
        assert summary["routing_mode"] == "centralized_regional"
        assert summary["routing_metrics"]["planning_calls"] == 1
        event = session.step(0, 0.)["outbound_handoffs"][0]
        assert (event["from_region"], event["to_region"]) == (1, 2)
        assert 0. < event["crossing_time"] <= 1.
    finally:
        session.close()


def test_target_region_accepts_handoff_and_replans_official_prefix(tmp_path):
    networks, graph, transition, mission = _scenario()
    transfer = {"source_region_id": 1, "source_layer": transition["source_layer"],
                "target_region_id": 2, "target_layer": transition["target_layer"],
                "proportion": 1.0}
    source = LocalSimulationSession(_config(
        tmp_path / "r1", 1, networks[1], graph, [transfer], [mission]))
    target = LocalSimulationSession(_config(
        tmp_path / "r2", 2, networks[2], graph, [transfer], []))
    try:
        event = source.step(0, 0.)["outbound_handoffs"][0]
        target.step(0, 0.)
        target.queue_handoffs([event])
        result = target.step(1, 1.)
        assert result["uav_count"] == 1
        assert target.summary()["routing_metrics"]["planning_calls"] == 1
    finally:
        source.close()
        target.close()


def test_overlapping_waiting_snapshot_is_deferred_instead_of_aborting(tmp_path):
    networks, graph, transition, mission = _scenario()
    mission = {**mission, "speed": 0.0}
    second = {**mission, "uav_id": 2, "current_edge": dict(mission["current_edge"])}
    transfer = {
        "source_region_id": 1,
        "source_layer": transition["source_layer"],
        "target_region_id": 2,
        "target_layer": transition["target_layer"],
        "proportion": 1.0,
    }
    session = LocalSimulationSession(_config(
        tmp_path, 1, networks[1], graph, [transfer], [mission, second]))
    try:
        session.step(0, 0.)
        assert session.summary()["pending_admissions"] == 1
    finally:
        session.close()


def test_strict_event_barrier_consumes_remaining_time_in_target_region(tmp_path):
    networks, graph, transition, mission = _scenario()
    transfer = {"source_region_id": 1, "source_layer": transition["source_layer"],
                "target_region_id": 2, "target_layer": transition["target_layer"],
                "proportion": 1.0}
    source = LocalSimulationSession(_config(
        tmp_path / "r1-event", 1, networks[1], graph, [transfer], [mission]))
    target = LocalSimulationSession(_config(
        tmp_path / "r2-event", 2, networks[2], graph, [transfer], []))
    try:
        source_prepare = source.prepare_substep(0, 0.0, 1.0)
        target_prepare = target.prepare_substep(0, 0.0, 1.0)
        crossing = source_prepare["next_event_time"]
        assert crossing is not None and 0.0 < crossing < 1.0

        source_commit = source.commit_substep(
            source_prepare["prepare_token"], crossing)
        target.commit_substep(target_prepare["prepare_token"], crossing)
        event = source_commit["outbound_handoffs"][0]
        assert event["crossing_time"] == crossing
        assert abs(event["remaining_step_time"] - (1.0-crossing)) < 1e-8

        target.queue_handoffs([event])
        remaining = 1.0-crossing
        source_prepare = source.prepare_substep(0, crossing, remaining)
        target_prepare = target.prepare_substep(0, crossing, remaining)
        source.commit_substep(source_prepare["prepare_token"], 1.0)
        target.commit_substep(target_prepare["prepare_token"], 1.0)
        source.finish_frame(0, 1.0)
        target.finish_frame(0, 1.0)

        assert 1 in target.uavs
        assert target.uavs[1].state in ("FLYING", "ARRIVED")
        assert target.summary()["sim_time"] == 1.0
        assert target.summary()["event_substeps"] == 2
    finally:
        source.close()
        target.close()
