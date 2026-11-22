"""
Profile memory for the current process tree during distributed simulations.

For Ray-specific object-store leaks, run `ray memory` in the same environment
while the simulation is active. This script complements that by recording
process RSS/USS over time.
"""
import argparse
import json
import os
import subprocess
import time

import psutil


def collect_process_tree(root_pid):
    root = psutil.Process(root_pid)
    processes = [root] + root.children(recursive=True)
    rows = []
    for proc in processes:
        try:
            mem = proc.memory_full_info()
            rows.append({
                "pid": proc.pid,
                "name": proc.name(),
                "cmdline": " ".join(proc.cmdline()),
                "rss_mb": round(mem.rss / 1024 / 1024, 2),
                "uss_mb": round(getattr(mem, "uss", 0) / 1024 / 1024, 2),
            })
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            continue
    return rows


def collect_ray_memory():
    try:
        completed = subprocess.run(
            ["ray", "memory", "--stats-only"],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    return {
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pid", type=int, default=os.getpid())
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--output", default="memory_profile.json")
    parser.add_argument("--include-ray-memory", action="store_true")
    args = parser.parse_args()

    samples = []
    for idx in range(args.samples):
        sample = {
            "sample": idx,
            "timestamp": time.time(),
            "processes": collect_process_tree(args.pid),
        }
        if args.include_ray_memory:
            sample["ray_memory"] = collect_ray_memory()
        samples.append(sample)
        time.sleep(args.interval)

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(samples, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()

