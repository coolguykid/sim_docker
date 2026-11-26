"""Submit the two-round demonstration to the previously deployed demo containers."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "test")]
import run_random_35_complete_mode as workload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default=datetime.now().strftime("round-comparison-%Y%m%d-%H%M%S"))
    parser.add_argument("--seed", type=int, default=20261118)
    args = parser.parse_args()
    trigger = random.Random(args.seed).choice(range(205, 600, 5))
    with ThreadPoolExecutor(max_workers=8) as executor:
        schemas = list(executor.map(
            lambda region: workload.call_json(
                "GET", f"http://127.0.0.1:{18100 + region}/openapi.json", timeout=10),
            workload.REGION_IDS,
        ))
    fields = {"demo_adjustment_time", "demo_stop_at_trigger"}
    if any(not fields <= schema["components"]["schemas"]["GuidanceRequest"]["properties"].keys()
           for schema in schemas):
        raise SystemExit("This replay demonstration requires the previously deployed demo image.")
    networks = workload.load_all_networks("docker", ROOT / "region_data", "http://127.0.0.1:{port}")
    graph = workload.build_global_partition_graph(
        networks, max(int(network.get("_topology_version", 1)) for network in networks.values()))
    missions = workload.generate_missions(networks, 10_000, 1, args.seed)
    requests = workload.build_requests(
        args.run_id, 600, missions, networks, graph, workload.build_uniform_guidance(graph))
    for request in requests.values():
        request.update(max_adjustment_rounds=1, demo_adjustment_time=trigger, demo_stop_at_trigger=True)
    print(f"{args.run_id}: 35 regions, 10000 UAVs, adjustment at {trigger}s, second round 600s", flush=True)
    workload.submit_requests(requests, 8)
    for attempt, end in enumerate((trigger, 600)):
        result = workload.wait_for_run(args.run_id, 7200, end, attempt_id=attempt)
        if result.get("status") != "decisions_completed" or result["completed_steps"] != end:
            raise RuntimeError(result)
        print(f"Attempt {attempt} complete at {end}s", flush=True)


if __name__ == "__main__":
    main()
