"""35-region complete-centralized-regional stress test.

Random OD missions are not prevalidated. Every route is planned by the final
delivered centralized_regional backend in the owning region. A failed plan is
retained as BLOCKED at its source edge, matching the tolerant-test policy.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import random
import statistics
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


TEST_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = TEST_ROOT.parent
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.submit_guidance_demo import load_all_networks  # noqa: E402
from simulation_engine.routing_planning_input_adapter import (  # noqa: E402
    build_global_partition_graph,
    graph_to_dict,
    guidance_transitions,
)




REGION_IDS = tuple(range(1, 36))


def call_json(method, url, payload=None, timeout=600):
    body = None if payload is None else json.dumps(
        payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    request = Request(
        url, data=body, method=method,
        headers={"Content-Type": "application/json; charset=utf-8"})
    try:
        with urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(
            "HTTP {} {} {}: {}".format(exc.code, method, url, detail)) from exc


def build_uniform_guidance(global_graph):
    choices = {}
    for source, target in guidance_transitions(global_graph).values():
        choices.setdefault(source, {source}).add(target)
    transfers = []
    for (source_region, source_layer), targets in sorted(choices.items()):
        probability = 1.0 / len(targets)
        for target_region, target_layer in sorted(targets):
            transfers.append({
                "source_region_id": source_region,
                "source_layer": source_layer,
                "target_region_id": target_region,
                "target_layer": target_layer,
                "proportion": probability,
            })
    return transfers


def generate_missions(networks, count, departure_span, seed):
    rng = random.Random(seed)
    selectable = {
        region_id: [row for row in network["pipes"] if row.get("is_open", True)]
        for region_id, network in networks.items()
    }
    positions = {
        region_id: {
            int(row["node_id"]): list(map(float, row["position"]))
            for row in network["nodes"]
        }
        for region_id, network in networks.items()
    }
    if any(not rows for rows in selectable.values()):
        raise RuntimeError("at least one region has no open source/destination pipe")
    # A purely independent random draw can place aircraft on parallel lanes at
    # the same longitudinal point. The delivered controller applies airborne
    # separation across lanes too, so allocate at most one UAV per open pipe.
    source_slots = {}
    for region_id, pipes in selectable.items():
        slots = [
            (pipe, rng.randrange(max(1, int(pipe.get("lane_number", 1)))))
            for pipe in pipes
        ]
        rng.shuffle(slots)
        source_slots[region_id] = slots
    capacity = sum(len(slots) for slots in source_slots.values())
    if count > capacity:
        raise RuntimeError(
            "{} UAVs exceed {} collision-free one-per-pipe source slots".format(
                count, capacity))
    base, remainder = divmod(count, len(REGION_IDS))
    requested_by_region = Counter({region_id: base for region_id in REGION_IDS})
    eligible = [
        region_id for region_id in REGION_IDS
        if len(source_slots[region_id]) > base
    ]
    rng.shuffle(eligible)
    if remainder > len(eligible):
        raise RuntimeError("balanced source allocation exceeds per-region pipe slots")
    for region_id in eligible[:remainder]:
        requested_by_region[region_id] += 1
    source_sequence = [
        region_id
        for region_id, amount in requested_by_region.items()
        for _ in range(amount)
    ]
    rng.shuffle(source_sequence)
    missions = []
    for index, source_region in enumerate(source_sequence):
        destination_region = rng.choice(REGION_IDS)
        while destination_region == source_region:
            destination_region = rng.choice(REGION_IDS)
        source, lane_id = source_slots[source_region].pop()
        destination = rng.choice(selectable[destination_region])
        destination_node = int(destination["end_node_id"])
        missions.append({
            "uav_id": index + 1,
            "region_id": source_region,
            "node_id": int(source["start_node_id"]),
            "current_edge": {
                "edge_type": "pipe",
                "edge_id": int(source["pipe_id"]),
            },
            "lane_id": lane_id,
            "distance_on_edge": 0.0,
            "speed": 0.0,
            "acceleration": 0.0,
            "spawn_turn": rng.randrange(max(1, departure_span)),
            "destination_region_id": destination_region,
            "destination_node_id": destination_node,
            "destination": positions[destination_region][destination_node],
            "destination_edge": {
                "edge_type": "pipe",
                "edge_id": int(destination["pipe_id"]),
            },
            "destination_distance": float(destination["length"]),
        })
    return missions


def write_snapshot(path, missions):
    fields = (
        "uav_id", "region_id", "source_pipe_id", "source_node_id",
        "destination_region_id", "destination_pipe_id", "destination_node_id",
        "lane_id", "spawn_turn",
    )
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for mission in missions:
            writer.writerow({
                "uav_id": mission["uav_id"],
                "region_id": mission["region_id"],
                "source_pipe_id": mission["current_edge"]["edge_id"],
                "source_node_id": mission["node_id"],
                "destination_region_id": mission["destination_region_id"],
                "destination_pipe_id": mission["destination_edge"]["edge_id"],
                "destination_node_id": mission["destination_node_id"],
                "lane_id": mission["lane_id"],
                "spawn_turn": mission["spawn_turn"],
            })


def build_requests(run_id, duration, missions, networks, global_graph, transfers):
    grouped = {region_id: [] for region_id in REGION_IDS}
    for mission in missions:
        grouped[int(mission["region_id"])].append(mission)
    serialized_graph = graph_to_dict(global_graph)
    snapshot_time = datetime.now(timezone.utc).isoformat()
    requests = {}
    for region_id in REGION_IDS:
        local_transfers = [
            {
                "source_layer": row["source_layer"],
                "target_region_id": row["target_region_id"],
                "target_layer": row["target_layer"],
                "proportion": row["proportion"],
            }
            for row in transfers
            if row["source_region_id"] == region_id
        ]
        requests[region_id] = {
            "run_id": run_id,
            "attempt_id": 0,
            "region_id": region_id,
            "participating_region_ids": list(REGION_IDS),
            "coordinator_url": "http://simulation-coordinator:8080",
            "snapshot_time": snapshot_time,
            "topology_version": int(
                networks[region_id].get("_topology_version", 1)),
            "horizon_seconds": duration,
            "step_seconds": 1.0,
            "random_seed": 42,
            "max_adjustment_rounds": 0,
            "transfers": local_transfers,
            "global_routing_graph": serialized_graph,
            "global_guidance_transfers": transfers,
            "guidance_revision": 1,
            "simulation_uavs": grouped[region_id],
            "congestion_policy": {
                "capacity_ratio": 1.0e12,
                "speed_ratio": -1.0,
                "duration_seconds": 1.0e12,
            },
        }
    return requests


def submit_requests(requests, workers):
    responses = {}
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(
                call_json,
                "POST",
                "http://127.0.0.1:{}/api/v1/guidance".format(18100 + region_id),
                payload,
            ): region_id
            for region_id, payload in requests.items()
        }
        for future in as_completed(futures):
            region_id = futures[future]
            responses[region_id] = future.result()
    return responses


def wait_for_run(run_id, timeout_seconds, expected_steps, attempt_id=0):
    url = "http://127.0.0.1:18080/api/v1/runs/{}/attempts/{}".format(run_id, attempt_id)
    deadline = time.monotonic() + timeout_seconds
    next_report = 0.0
    last = None
    while time.monotonic() < deadline:
        try:
            last = call_json("GET", url, timeout=15)
        except (RuntimeError, URLError):
            time.sleep(1)
            continue
        if last.get("status") in ("decisions_completed", "failed"):
            return last
        if time.monotonic() >= next_report:
            timing = last.get("timing") or {}
            print(
                "PROGRESS status={} registered={}/35 steps={}/{} frame_avg={}".format(
                    last.get("status"),
                    len(last.get("registered_region_ids") or ()),
                    last.get("completed_steps", 0),
                    expected_steps,
                    timing.get("frame_wall_seconds_average"),
                ),
                flush=True,
            )
            next_report = time.monotonic() + 20.0
        time.sleep(1)
    raise TimeoutError("run timed out; last={}".format(last))


def percentile(values, q):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * q)]


def inspect_regions(run_id):
    failures, preparations, status_counts = {}, [], Counter()
    for region_id in REGION_IDS:
        base = (
            "http://127.0.0.1:{}/api/v1/simulations/{}/attempts/0".format(
                18100 + region_id, run_id))
        status = call_json("GET", base)
        status_counts[status["status"]] += 1
        events = call_json("GET", base + "/events").get("events", [])
        by_type = {row["event_type"]: row for row in events}
        if "snapshotting" in by_type and "regional_session_ready" in by_type:
            start = datetime.fromisoformat(
                by_type["snapshotting"]["created_at"].replace("Z", "+00:00"))
            end = datetime.fromisoformat(
                by_type["regional_session_ready"]["created_at"].replace("Z", "+00:00"))
            preparations.append((end - start).total_seconds())
        evaluated = [row for row in events if row["event_type"] == "evaluated"]
        rows = (evaluated[-1]["payload"].get("planning_failures", [])
                if evaluated else [])
        failures[str(region_id)] = len(rows)
    return {
        "status_counts": dict(status_counts),
        "planning_failure_count": sum(failures.values()),
        "planning_failures_by_region": failures,
        "regional_preparation_seconds": {
            "average": statistics.fmean(preparations) if preparations else None,
            "p50": percentile(preparations, 0.50),
            "p95": percentile(preparations, 0.95),
            "max": max(preparations) if preparations else None,
        },
    }


def inspect_outputs(run_id, attempt_id=0):
    rows_by_time, departed_by_time = Counter(), Counter()
    total_bytes = 0
    for region_id in REGION_IDS:
        path = (PROJECT_ROOT / "output" / "region_{:02d}".format(region_id) /
                run_id / "attempt_{:03d}".format(attempt_id) / "uav_states.csv")
        total_bytes += path.stat().st_size
        with path.open(encoding="utf-8-sig", newline="") as stream:
            for row in csv.DictReader(stream):
                sim_time = str(float(row["time"]))
                rows_by_time[sim_time] += 1
                departed_by_time[sim_time] += int(row["departed"])
    return {
        "uav_rows_by_time": dict(sorted(
            rows_by_time.items(), key=lambda item: float(item[0]))),
        "departed_rows_by_time": dict(sorted(
            departed_by_time.items(), key=lambda item: float(item[0]))),
        "uav_states_total_bytes": total_bytes,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--uavs", type=int, default=10000)
    parser.add_argument("--duration", type=int, default=10)
    parser.add_argument("--departure-span", type=int, default=1)
    parser.add_argument("--seed", type=int, default=20261118)
    parser.add_argument("--timeout", type=int, default=3600)
    parser.add_argument("--submit-workers", type=int, default=8)
    args = parser.parse_args()
    if min(args.uavs, args.duration, args.departure_span, args.submit_workers) <= 0:
        parser.error("numeric arguments must be positive")

    run_dir = TEST_ROOT / "runs" / args.run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    total_started = time.perf_counter()

    started = time.perf_counter()
    networks = load_all_networks(
        "docker", PROJECT_ROOT / "region_data", "http://127.0.0.1:{port}")
    global_graph = build_global_partition_graph(networks, max(
        int(value.get("_topology_version", 1)) for value in networks.values()))
    transfers = build_uniform_guidance(global_graph)
    topology_seconds = time.perf_counter() - started

    started = time.perf_counter()
    missions = generate_missions(
        networks, args.uavs, args.departure_span, args.seed)
    write_snapshot(run_dir / "initial_snapshot.csv", missions)
    demand_seconds = time.perf_counter() - started

    started = time.perf_counter()
    requests = build_requests(
        args.run_id, args.duration, missions, networks, global_graph, transfers)
    request_seconds = time.perf_counter() - started
    manifest = {
        "run_id": args.run_id,
        "region_count": len(REGION_IDS),
        "uav_count": len(missions),
        "global_nodes": len(global_graph.nodes),
        "global_edges": len(global_graph.edges),
        "boundary_edges": len(global_graph.metadata["boundary_transitions"]),
        "guidance_transfer_count": len(transfers),
    }
    (run_dir / "request_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")

    started = time.perf_counter()
    responses = submit_requests(requests, args.submit_workers)
    submission_seconds = time.perf_counter() - started

    started = time.perf_counter()
    coordinated = wait_for_run(args.run_id, args.timeout, args.duration)
    wait_seconds = time.perf_counter() - started
    if coordinated.get("status") == "failed":
        raise RuntimeError(coordinated.get("error"))

    started = time.perf_counter()
    regional = inspect_regions(args.run_id)
    outputs = inspect_outputs(args.run_id)
    inspection_seconds = time.perf_counter() - started
    timing = coordinated.get("timing") or {}
    average = timing.get("frame_wall_seconds_average")
    report = {
        **manifest,
        "duration_seconds": args.duration,
        "departure_span": args.departure_span,
        "source_region_distribution": dict(sorted(Counter(
            row["region_id"] for row in missions).items())),
        "destination_region_distribution": dict(sorted(Counter(
            row["destination_region_id"] for row in missions).items())),
        "phase_wall_seconds": {
            "docker_topology_load_global_graph_and_guidance": topology_seconds,
            "random_demand_generation_and_snapshot_write": demand_seconds,
            "request_build": request_seconds,
            "parallel_guidance_submission": submission_seconds,
            "wait_for_preparation_simulation_and_evaluation": wait_seconds,
            "result_inspection": inspection_seconds,
            "total": time.perf_counter() - total_started,
        },
        "global_frame_wall_seconds": {
            "average": average,
            "min": timing.get("frame_wall_seconds_min"),
            "p50": timing.get("frame_wall_seconds_p50"),
            "p95": timing.get("frame_wall_seconds_p95"),
            "max": timing.get("frame_wall_seconds_max"),
            "frames_per_wall_second": 1.0 / average if average else None,
        },
        "coordinator": coordinated,
        "regional": regional,
        "outputs": outputs,
        "submission_region_count": len(responses),
    }
    path = run_dir / "timing_report.json"
    path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print("TIMING_REPORT={}".format(path), flush=True)


if __name__ == "__main__":
    main()
