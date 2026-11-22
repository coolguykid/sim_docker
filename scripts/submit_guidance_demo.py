"""Temporary center-DT client for the complete centralized-regional smoke run.

The production center DT is not available yet. This script temporarily reads
the topology projection from every regional DT container, compiles the global
partition graph, and sends that graph with the guidance request. It starts a
small 1 -> 2 journey, while the routing summary still contains all 35 regions.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
from datetime import datetime, timezone
import json
from math import isfinite
from pathlib import Path
import sys
from time import monotonic, sleep
from urllib.error import HTTPError
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
DELIVERY_SOURCE = next(
    path / "src"
    for path in ROOT.iterdir()
    if (path / "src" / "uav_sim" / "distributed" / "regional_backend.py").is_file()
)
sys.path.insert(0, str(DELIVERY_SOURCE))
sys.path.insert(1, str(ROOT))

from simulation_engine.delivered_adapter import (  # noqa: E402
    build_delivered_topology,
    build_global_partition_graph,
    graph_to_dict,
)


def post(url, payload):
    request = Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with urlopen(request, timeout=120) as response:
        return json.loads(response.read().decode("utf-8"))


def get(url, timeout=120):
    with urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def _array(value, cast=float):
    text = str(value or "").replace("{", "").replace("}", "")
    return [cast(item) for item in text.split(",") if item]


def _csv_rows(root, region_id, name):
    path = root / "region_{:03d}".format(region_id) / name
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def load_network(root, region_id):
    """Offline fallback used by local tests when Docker is unavailable."""
    nodes = _csv_rows(root, region_id, "node_info.csv")
    for row in nodes:
        row["node_id"] = int(row["node_id"])
        row["position"] = _array(row["position"])
        row["region_ids"] = _array(row["region_ids"], int)
        row["layer_id"] = int(row["layer_id"]) if row["layer_id"] else None
    pipes = _csv_rows(root, region_id, "pipe_info.csv")
    for row in pipes:
        for field in ("pipe_id", "start_node_id", "end_node_id", "layer_id", "lane_number"):
            row[field] = int(row[field])
        row["length"] = float(row["length"])
        row.update(is_open=True, speed_limit=15.0, max_capacity=50)
    lanes = _csv_rows(root, region_id, "lane_info.csv")
    for row in lanes:
        row["lane_id"], row["pipe_id"] = int(row["lane_id"]), int(row["pipe_id"])
        row["start_pt"], row["end_pt"] = _array(row["start_pt"]), _array(row["end_pt"])
    ramps = _csv_rows(root, region_id, "ramp_info.csv")
    for row in ramps:
        for field in ("ramp_id", "start_node_id", "end_node_id", "lane_number"):
            row[field] = int(row[field])
        row["length"], row["is_open"] = float(row["length"]), True
        flat = _array(row["ramp_path"])
        row["ramp_path"] = [flat[index:index + 3] for index in range(0, len(flat), 3)]
    return {
        "nodes": nodes,
        "pipes": pipes,
        "lanes": lanes,
        "ramps": ramps,
        "layers": _csv_rows(root, region_id, "layer_info.csv"),
        "_topology_version": 1,
    }


def _network_from_topology(dto):
    """Convert one regional read API DTO to the delivered-adapter input."""
    nodes = [
        {
            "node_id": int(row["nodeId"]),
            "position": list(map(float, row["position"])),
            "region_ids": [int(value) for value in row["regionIds"]],
            "layer_id": None if row["layerId"] is None else int(row["layerId"]),
        }
        for row in dto["nodes"]
    ]
    pipes = [
        {
            "pipe_id": int(row["pipeId"]),
            "start_node_id": int(row["startNodeId"]),
            "end_node_id": int(row["endNodeId"]),
            "layer_id": int(row["layerId"]),
            "lane_number": int(row["laneNumber"]),
            "length": float(row["length"]),
            "is_open": bool(row["state"]["isOpen"]),
            "speed_limit": float(row["state"]["speedLimit"]),
            "max_capacity": int(row["state"]["maxCapacity"]),
        }
        for row in dto["pipes"]
    ]
    ramps = [
        {
            "ramp_id": int(row["rampId"]),
            "start_node_id": int(row["startNodeId"]),
            "end_node_id": int(row["endNodeId"]),
            "lane_number": int(row["laneNumber"]),
            "length": float(row["length"]),
            "is_open": bool(row["state"]["isOpen"]),
            "ramp_path": [list(map(float, point)) for point in row["path"]],
        }
        for row in dto["ramps"]
    ]
    lanes = [
        {
            "lane_id": int(row["laneId"]),
            "pipe_id": int(row["pipeId"]),
            "layer_id": int(row["layerId"]),
            "length": float(row["length"]),
            "start_pt": list(map(float, row["startPoint"])),
            "end_pt": list(map(float, row["endPoint"])),
        }
        for row in dto["lanes"]
    ]
    return {
        "nodes": nodes,
        "pipes": pipes,
        "ramps": ramps,
        "lanes": lanes,
        "layers": [
            {"layer_id": int(row["layerId"]), "height": float(row["height"])}
            for row in dto.get("layers", ())
        ],
        "_topology_version": int(dto["topologyVersion"]),
    }


def region_url(region_id, url_template):
    return url_template.format(
        region_id=region_id, port=18100 + region_id).rstrip("/")


def load_docker_network(region_id, url_template):
    return _network_from_topology(get(
        region_url(region_id, url_template) + "/api/v1/topology"))


def load_all_networks(source, root, url_template, region_ids=range(1, 36)):
    if source == "csv":
        return {region_id: load_network(root, region_id) for region_id in region_ids}
    with ThreadPoolExecutor(max_workers=12) as pool:
        values = pool.map(
            lambda rid: load_docker_network(rid, url_template), region_ids)
        return dict(zip(region_ids, values))


def build_requests(root, run_id, duration, networks=None):
    networks = networks or load_all_networks(
        "csv", root, "http://127.0.0.1:{port}")
    global_version = max(
        int(value.get("_topology_version", 1)) for value in networks.values())
    global_graph = build_global_partition_graph(networks, global_version)
    boundary_id, transition = next(
        (edge_id, item)
        for edge_id, item in global_graph.metadata["boundary_transitions"].items()
        if item["source_region"] == 1 and item["target_region"] == 2
    )
    boundary = global_graph.edges[boundary_id]
    source = build_delivered_topology(
        1, int(networks[1].get("_topology_version", 1)), networks[1])
    target = build_delivered_topology(
        2, int(networks[2].get("_topology_version", 1)), networks[2])
    source_edge = next(
        edge for edge in source.graph.edges.values()
        if edge.target == boundary.source
    )
    entry = next(
        edge for edge in target.graph.edges.values()
        if edge.source == boundary.target
    )
    global_transfer = {
        "source_region_id": 1,
        "source_layer": transition["source_layer"],
        "target_region_id": 2,
        "target_layer": transition["target_layer"],
        "proportion": 1.0,
    }
    mission = {
        "uav_id": 1,
        "region_id": 1,
        "node_id": source.node_external_ids[source_edge.source],
        "current_edge": {"edge_type": "pipe", "edge_id": source_edge.pipe_id},
        "lane_id": 0,
        "distance_on_edge": max(0.0, source_edge.length_m - 20.0),
        "speed": 10.0,
        "acceleration": 0.0,
        "destination_region_id": 2,
        "destination_node_id": target.node_external_ids[entry.target],
    }
    now = datetime.now(timezone.utc).isoformat()
    common = {
        "run_id": run_id,
        "attempt_id": 0,
        "snapshot_time": now,
        "horizon_seconds": int(duration),
        "step_seconds": 1.0,
        "random_seed": 42,
        "max_adjustment_rounds": 0,
        "participating_region_ids": [1, 2],
        "coordinator_url": "http://simulation-coordinator:8080",
        "congestion_policy": {"capacity_ratio": 10.0, "duration_seconds": 9999},
        "global_routing_graph": graph_to_dict(global_graph),
        "global_guidance_transfers": [global_transfer],
        "guidance_revision": 1,
    }
    return {
        1: {
            **common,
            "region_id": 1,
            "topology_version": int(networks[1].get("_topology_version", 1)),
            "simulation_uavs": [mission],
            "transfers": [
                {
                    "source_layer": transition["source_layer"],
                    "target_region_id": 2,
                    "target_layer": transition["target_layer"],
                    "proportion": 1.0,
                }
            ],
        },
        2: {
            **common,
            "region_id": 2,
            "topology_version": int(networks[2].get("_topology_version", 1)),
            "simulation_uavs": [],
            "transfers": [],
        },
    }


def verify_simulation(region_urls, coordinator_url, run_id, duration, timeout):
    """Verify the simulated transfer using coordinator state and CSV-backed APIs."""
    attempt_path = f"{run_id}/attempts/0"
    coordinator_attempt = f"{coordinator_url.rstrip('/')}/api/v1/runs/{attempt_path}"
    deadline = monotonic() + timeout
    terminal = {"completed", "waiting_guidance_dispatch"}

    def poll(url):
        remaining = deadline - monotonic()
        if remaining <= 0:
            raise TimeoutError(f"Verification timed out after {timeout:g}s for {run_id}")
        return get(url, timeout=remaining)

    while True:
        tasks = {
            region_id: poll(f"{url}/api/v1/simulations/{attempt_path}")
            for region_id, url in region_urls.items()
        }
        for region_id, task in tasks.items():
            status = task["status"]
            if status == "failed" or (status.startswith("waiting_") and status not in terminal):
                raise RuntimeError(f"Region {region_id}: {status}: {task['error_message']}")
        try:
            coordinated = poll(coordinator_attempt)
        except HTTPError as exc:
            if exc.code != 404:
                raise
            coordinated = {}
        if coordinated.get("status") == "failed":
            raise RuntimeError(f"Coordinator failed: {coordinated['error']}")
        if (coordinated.get("status") == "decisions_completed"
                and all(task["status"] in terminal for task in tasks.values())):
            break
        if monotonic() >= deadline:
            raise TimeoutError(f"Verification timed out. Regions: {tasks}; coordinator: {coordinated}")
        sleep(0.5)

    if (coordinated["sim_time"] != duration
            or coordinated["completed_steps"] != duration
            or coordinated["handoff_pairs"] != {"1->2": 1}
            or coordinated["collective_decision"] != "acceptable"):
        raise RuntimeError(f"Unexpected coordinator result: {coordinated}")

    snapshots = {
        region_id: get(f"{url}/api/v1/simulation-results/{attempt_path}/snapshot?time={duration}")
        for region_id, url in region_urls.items()
    }
    for region_id, snapshot in snapshots.items():
        if snapshot["frameTime"] != duration or not snapshot["statistics"]:
            raise RuntimeError(f"Region {region_id}: expected final CSV frame and edge statistics")
    source_has_uav = any(row["uavId"] == "1" for row in snapshots[1]["uavs"])
    target_has_uav = any(
        row["uavId"] == "1" and row["regionId"] == 2 for row in snapshots[2]["uavs"])
    if source_has_uav or not target_has_uav:
        raise RuntimeError("UAV 1 must be present in region 2 only at the final frame")

    print(f"Verified {duration} steps and one 1->2 handoff; final CSV frame and statistics available.")
    print("Regional workflow states: " + ", ".join(
        f"{region_id}={task['status']}" for region_id, task in tasks.items()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--region-data", type=Path, default=ROOT / "region_data")
    parser.add_argument(
        "--topology-source", choices=("docker", "csv"), default="docker")
    parser.add_argument(
        "--region-url-template",
        default="http://127.0.0.1:{port}",
        help="Supports {region_id} and {port}; default ports are 18101..18135.",
    )
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--duration", type=int, default=10)
    parser.add_argument("--regions", type=int, nargs="+", default=list(range(1, 36)),
                        help="Topology regions to load. Use 1 2 for a two-container smoke run.")
    parser.add_argument("--verify", action="store_true",
                        help="Wait for and verify the 1 -> 2 simulation and CSV-backed results.")
    parser.add_argument("--coordinator-url", default="http://127.0.0.1:18080",
                        help="Coordinator URL used by this client for verification.")
    parser.add_argument("--wait-timeout", type=float, default=180.0)
    args = parser.parse_args()
    if not {1, 2}.issubset(args.regions) or len(set(args.regions)) != len(args.regions):
        parser.error("--regions must include 1 and 2 and contain no duplicates")
    if any(region_id < 1 or region_id > 35 for region_id in args.regions):
        parser.error("--regions must use available database region IDs from 1 to 35")
    if args.verify and (args.duration < 5 or not isfinite(args.wait_timeout)
                        or args.wait_timeout <= 0):
        parser.error("--verify requires --duration >= 5 and a positive --wait-timeout")
    networks = load_all_networks(
        args.topology_source, args.region_data, args.region_url_template, args.regions)
    requests = build_requests(args.region_data, args.run_id, args.duration, networks)
    print("Compiled temporary global graph from {} regional {} sources.".format(
        len(networks), args.topology_source))
    region_urls = {
        region_id: region_url(region_id, args.region_url_template) for region_id in requests
    }
    for region_id, payload in requests.items():
        result = post(f"{region_urls[region_id]}/api/v1/guidance", payload)
        print("region {}: {}".format(region_id, result["status"]))
    if args.verify:
        verify_simulation(region_urls, args.coordinator_url, args.run_id,
                          args.duration, args.wait_timeout)


if __name__ == "__main__":
    main()
