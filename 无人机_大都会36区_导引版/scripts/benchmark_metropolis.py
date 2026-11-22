"""Reproducible load test through the enhanced original simulation session.

Run with the project's Python 3.9 environment. Each process performs one real
run; the six combinations are separate processes to isolate Ray ownership.
"""
import argparse
import ctypes
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def memory_metrics():
    if os.name != "nt":
        return {}

    class Counters(ctypes.Structure):
        _fields_ = [("cb", ctypes.c_ulong), ("PageFaultCount", ctypes.c_ulong)] + [
            (name, ctypes.c_size_t)
            for name in (
                "PeakWorkingSetSize",
                "WorkingSetSize",
                "QuotaPeakPagedPoolUsage",
                "QuotaPagedPoolUsage",
                "QuotaPeakNonPagedPoolUsage",
                "QuotaNonPagedPoolUsage",
                "PagefileUsage",
                "PeakPagefileUsage",
            )
        ]

    counters = Counters()
    counters.cb = ctypes.sizeof(counters)
    current_process = ctypes.windll.kernel32.GetCurrentProcess
    current_process.restype = ctypes.c_void_p
    query = ctypes.windll.psapi.GetProcessMemoryInfo
    query.argtypes = [ctypes.c_void_p, ctypes.POINTER(Counters), ctypes.c_ulong]
    if not query(current_process(), ctypes.byref(counters), counters.cb):
        raise ctypes.WinError()
    return dict(
        driver_peak_working_set_mib=counters.PeakWorkingSetSize / 1024**2,
        driver_working_set_mib=counters.WorkingSetSize / 1024**2,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=["centralized_full", "centralized_regional", "onboard_regional"],
        required=True,
    )
    parser.add_argument("--dt", type=float, choices=[0.1, 1.0], required=True)
    parser.add_argument("--target", type=int, default=10000)
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--spawn-seconds", type=float, default=4000.0)
    parser.add_argument("--drain-seconds", type=float, default=15000.0)
    parser.add_argument("--mission-limit", type=int)
    parser.add_argument("--stop-after-peak", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-safety", action="store_true")
    parser.add_argument('--continuous-boundaries', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    from uav_sim.app import _build_airspace_from_config
    from uav_sim.simulation.metropolis_runtime import MetropolisSession

    config = dict(
        airspace_mode="metropolis",
        topology_profile="metropolis_partition_v1",
        num_pipes_per_layer=50,
        num_lanes_per_pipe=20,
        include_dynamic=False,
        inter_dynamic_level="none",
        intra_dynamic_level="none",
        routing_mode=args.mode,
        sim_dt_seconds=args.dt,
        num_uavs=args.target,
        max_batch_size=min(1000, args.target),
        replenishment_period_seconds=args.period,
        max_sim_time_seconds=args.spawn_seconds,
        drain_timeout_seconds=args.drain_seconds,
        destination_count=10,
        destination_policy="fixed",
        random_seed=42,
        num_pickup_points=0,
        track_uav_ids=[0, 499, 999],
        run_output_dir=str(args.output),
        use_ray=args.mode == "onboard_regional",
        parallelism_level=1000,
        stop_after_peak_active=args.stop_after_peak,
        report_admission_progress=True,
        admission_progress_interval=100,
        stop_at_region_boundaries=not args.continuous_boundaries,
    )
    if args.mission_limit is not None:
        config["mission_limit"] = args.mission_limit
    (args.output / "config.json").write_text(
        json.dumps(config, indent=2), encoding="utf-8"
    )
    started = time.perf_counter()
    env = _build_airspace_from_config(config)
    progress = []
    next_report = [0.0]
    sampled = dict(
        sample_count=0,
        unsafe_pairs=0,
        body_contacts=0,
        minimum_distance_m=None,
        sampling_period_seconds=10.0,
    )
    next_sample = [0.0]
    with MetropolisSession(env, config) as session:

        def on_step(value):
            now = value["sim_time_seconds"]
            if value.get('stage') == 'admission':
                print(json.dumps(dict(value, **memory_metrics()), ensure_ascii=False), flush=True)
                return
            if args.sample_safety and now >= next_sample[0]:
                # Independent cKDTree checks actual sampled 3D points. Full
                # continuous safety is certified separately by reservations.
                import numpy as np
                from scipy.spatial import cKDTree

                states = [
                    u.sample_partition_state(now) for u in session.active.values()
                ]
                if len(states) > 1:
                    points = np.array([s.position for s in states])
                    tree = cKDTree(points)
                    nearest = float(tree.query(points, k=2)[0][:, 1].min())
                    sampled["minimum_distance_m"] = (
                        nearest
                        if sampled["minimum_distance_m"] is None
                        else min(nearest, sampled["minimum_distance_m"])
                    )
                    for a, b in tree.query_pairs(30.001):
                        distance = float(np.linalg.norm(points[a] - points[b]))
                        required = max(
                            session._threshold(states[a]), session._threshold(states[b])
                        )
                        sampled["unsafe_pairs"] += int(distance < required - 1e-6)
                        sampled["body_contacts"] += int(distance < 2.0)
                sampled["sample_count"] += 1
                next_sample[0] += sampled["sampling_period_seconds"]
            if now >= next_report[0]:
                row = dict(value, **memory_metrics())
                progress.append(row)
                print(json.dumps(row, ensure_ascii=False), flush=True)
                next_report[0] += 100.0

        try:
            summary = session.run(on_step)
        finally:
            (args.output / "progress.json").write_text(
                json.dumps(progress, indent=2), encoding="utf-8"
            )
        summary["independent_sampled_safety"] = sampled if args.sample_safety else None
        summary["memory"] = memory_metrics()
        summary["total_wall_seconds_including_setup"] = time.perf_counter() - started
        summary["source_root"] = str(ROOT)
        summary["config"] = config
        summary["goal_errors_over_1e5_m"] = sum(
            r["goal_position_error_m"] > 1e-5 for r in session.completed
        )
        summary['load_requirement_met'] = summary['peak_active'] >= args.target
        summary['acceptance_passed'] = bool(summary['success'] and
            (not args.stop_after_peak or summary['load_requirement_met']) and
            summary['goal_errors_over_1e5_m'] == 0 and
            (not args.sample_safety or sampled['unsafe_pairs'] == sampled['body_contacts'] == 0))
        (args.output / "benchmark_result.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print("RESULT " + json.dumps(summary, ensure_ascii=False), flush=True)
    if not summary["success"]:
        raise SystemExit(2)
    if args.stop_after_peak and summary["peak_active"] < args.target:
        raise SystemExit(3)
    if not summary['acceptance_passed']:
        raise SystemExit(4)


if __name__ == "__main__":
    main()
