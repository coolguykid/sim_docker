"""Regional-DT business workflow; the coordinator is only a simulation tool."""
from __future__ import annotations

import json
import os
from pathlib import Path

from simulation_engine.csv_output import iter_rolling_statistics

from coordinator_client import (
    register_attempt, report_decision, wait_for_status)
from distributed_runtime import finalize, get_result, initialize_session
from engine_contract import IntegrationPending
from integration_ports import (
    dispatch_guidance, evaluate_statistics, invoke_network_adjustment)
from repository import (
    REGION_ID, append_workflow_event, commit_to_database, load_snapshot, mark_status)
from snapshot_loader import snapshot_summary
from topology_reader import attempt_topology
from topology_adjustments import apply_to_snapshot, merge_adjustments


WAITING_STATUS = {
    "simulation_engine": "waiting_simulation_engine",
    "traffic_evaluator": "waiting_traffic_evaluator",
    "network_adjustment": "waiting_network_adjustment",
    "guidance_dispatch": "waiting_guidance_dispatch",
}


def _uav_missions(snapshot, request):
    """依据请求和快照选取本区域任务。

    输入
        snapshot：区域数据库快照。
        request：区域任务请求及运行配置。
    输出
        无人机任务列表。
    """
    supplied = request.get("simulation_uavs") or ()
    if supplied:
        return [dict(value) for value in supplied]
    missions = []
    for row in snapshot.get("uavs", ()):
        edge_type = int(row["edge_type"])
        mission = {
            "uav_id": int(row["uav_id"]),
            "region_id": int(row["region_id"]),
            "edge_type": edge_type,
            "pipe_id": row.get("pipe_id"),
            "ramp_id": row.get("ramp_id"),
            "node_id": row.get("node_id"),
            "lane_id": row.get("lane_id"),
            "distance_on_edge": row.get("distance_on_edge") or 0.0,
            "speed": row.get("speed") or 0.0,
            "acceleration": row.get("acceleration") or 0.0,
            "destination": row["destination"],
        }
        missions.append(mission)
    return missions


def _attempt_output_dir(run_id, attempt_id):
    """根据运行 ID 和尝试编号构造区域输出目录。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        Path。
    """
    root = Path(os.environ.get("REGION_OUTPUT_DIR", "/app/data"))
    return root / str(run_id) / "attempt_{:03d}".format(int(attempt_id))


def run_workflow(request: dict) -> None:
    """执行快照、区域注册、仿真评估和可能的网络调整或下发。

    输入
        request：区域任务请求及运行配置。
    输出
        None，结果与状态写入存储。

    原有说明
        Execute snapshot -> distributed simulation -> evaluate -> adjust/commit.
    """
    run_id = request["run_id"]
    base_attempt_id = int(request["attempt_id"])
    max_rounds = int(request.get("max_adjustment_rounds", 3))
    coordinator_url = str(request.get(
        "coordinator_url", "http://simulation-coordinator:8080"))
    region_endpoint = os.environ.get(
        "REGION_PUBLIC_URL", "http://region-dt-{:02d}:8080".format(REGION_ID))
    cumulative_adjustments = dict(request.get("topology_adjustments") or {})

    try:
        global_transfers = request.get("global_guidance_transfers") or []
        local_transfers = request.get("transfers", [])
        if global_transfers:
            local_transfers = [
                {
                    "source_layer": row["source_layer"],
                    "target_region_id": row["target_region_id"],
                    "target_layer": row["target_layer"],
                    "proportion": row["proportion"],
                }
                for row in global_transfers if row["source_region_id"] == REGION_ID
            ]

        mark_status(run_id, base_attempt_id, "snapshotting")
        append_workflow_event(run_id, base_attempt_id, "snapshotting", {})
        original_snapshot = load_snapshot(request)
        summary = snapshot_summary(original_snapshot)
        missions = _uav_missions(original_snapshot, request)
        mark_status(
            run_id, base_attempt_id, "snapshot_ready", snapshot_summary=summary)
        append_workflow_event(run_id, base_attempt_id, "snapshot_ready", summary)

        rounds = []
        for round_index in range(max_rounds + 1):
            distributed_attempt_id = base_attempt_id + round_index
            effective_snapshot = apply_to_snapshot(
                original_snapshot, cumulative_adjustments)
            output_dir = _attempt_output_dir(run_id, distributed_attempt_id)
            session_config = {
                "run_id": run_id,
                "attempt_id": distributed_attempt_id,
                "region_id": REGION_ID,
                "topology_version": request["topology_version"],
                "step_seconds": request["step_seconds"],
                "random_seed": request.get("random_seed", 42),
                "network": effective_snapshot["network"],
                "uavs": missions,
                "guidance": {"transfers": local_transfers},
                "global_routing_graph": request.get("global_routing_graph"),
                "global_guidance_transfers": global_transfers,
                "guidance_revision": request.get("guidance_revision", 1),
                "statistics_window_seconds": 60.0,
                "statistics_interval_seconds": 5.0,
                "congestion_policy": request.get("congestion_policy", {}),
                "fixed_statistics_interval_seconds": 60.0,
                "output_dir": str(output_dir),
            }
            output_dir.mkdir(parents=True, exist_ok=True)
            topology_path = output_dir / "topology.json"
            temporary_path = topology_path.with_suffix(".tmp")
            temporary_path.write_text(
                json.dumps(attempt_topology(effective_snapshot), default=str), encoding="utf-8")
            temporary_path.replace(topology_path)
            initialize_session(session_config)
            mark_status(run_id, base_attempt_id, "simulating")
            append_workflow_event(run_id, base_attempt_id, "regional_session_ready", {
                "round_index": round_index,
                "distributed_attempt_id": distributed_attempt_id,
                "output_dir": str(output_dir),
                "topology_adjustments": cumulative_adjustments,
            })
            register_attempt(coordinator_url, {
                "run_id": run_id,
                "attempt_id": distributed_attempt_id,
                "region_id": REGION_ID,
                "endpoint": region_endpoint,
                "expected_region_ids": request["participating_region_ids"],
                "horizon_seconds": request["horizon_seconds"],
                "step_seconds": request["step_seconds"],
            })
            coordinated = wait_for_status(
                coordinator_url, run_id, distributed_attempt_id,
                {"simulation_completed", "failed"})
            if coordinated["status"] == "failed":
                raise RuntimeError("distributed simulation failed: {}".format(
                    coordinated.get("error")))

            local_result = get_result(run_id, distributed_attempt_id)
            mark_status(run_id, base_attempt_id, "evaluating")
            statistics_csv = local_result["statistics_csv"]
            congestion = coordinated.get("congestion")
            evaluation = (congestion or {}).get("regions", {}).get(str(REGION_ID))
            if evaluation is None:
                evaluation = evaluate_statistics({}, request)
            report_decision(coordinator_url, {
                "run_id": run_id, "attempt_id": distributed_attempt_id,
                "region_id": REGION_ID, "decision": evaluation["decision"],
            })
            collective = wait_for_status(
                coordinator_url, run_id, distributed_attempt_id,
                {"decisions_completed", "failed"})
            round_result = {
                "round_index": round_index,
                "distributed_attempt_id": distributed_attempt_id,
                "evaluation": evaluation,
                "collective_decision": collective.get("collective_decision"),
                "end_time_s": coordinated["sim_time"],
                "congestion": congestion,
                "frame_csv": local_result["frame_csv"],
                "statistics_csv": local_result["statistics_csv"],
                "fixed_statistics_csv": local_result["fixed_statistics_csv"],
                "planning_failures": local_result["planning_failures"],
            }
            rounds.append(round_result)
            append_workflow_event(
                run_id, base_attempt_id, "evaluated", round_result)
            finalize(run_id, distributed_attempt_id)
            del local_result

            if collective.get("collective_decision") == "acceptable":
                commit_receipt = commit_to_database(
                    request["topology_version"], effective_snapshot,
                    cumulative_adjustments)
                mark_status(run_id, base_attempt_id, "dispatching_guidance")
                dispatch_receipt = dispatch_guidance({
                    "run_id": run_id,
                    "attempt_id": base_attempt_id,
                    "region_id": REGION_ID,
                    "guidance": {"transfers": local_transfers},
                    "evaluation": evaluation,
                    "topology_changed": commit_receipt["topology_changed"],
                    "topology_change": commit_receipt,
                    "topology_adjustments": cumulative_adjustments,
                })
                result = {
                    "decision": "acceptable",
                    "rounds": rounds,
                    "topology_commit": commit_receipt,
                    "dispatch_receipt": dispatch_receipt,
                }
                append_workflow_event(
                    run_id, base_attempt_id, "guidance_dispatched", result)
                mark_status(
                    run_id, base_attempt_id, "completed", result_summary=result)
                return

            if round_index >= max_rounds:
                result = {
                    "decision": "adjustment_limit_reached", "rounds": rounds}
                mark_status(
                    run_id, base_attempt_id, "completed", result_summary=result)
                return

            if evaluation["decision"] == "adjustment_required":
                mark_status(run_id, base_attempt_id, "adjusting_network")
                new_adjustments = invoke_network_adjustment({
                    "run_id": run_id,
                    "attempt_id": distributed_attempt_id,
                    "region_id": REGION_ID,
                    "original_snapshot_summary": summary,
                    "effective_snapshot": effective_snapshot,
                    "current_adjustments": cumulative_adjustments,
                    "statistics": iter_rolling_statistics(statistics_csv),
                    "evaluation": evaluation,
                })
                append_workflow_event(run_id, base_attempt_id, "network_adjustment_proposed", {
                    "round_index": round_index,
                    "distributed_attempt_id": distributed_attempt_id,
                    "pipe_updates": len(new_adjustments.get("pipes", ())),
                    "ramp_updates": len(new_adjustments.get("ramps", ())),
                    "control": new_adjustments.get("control", {}),
                })
                cumulative_adjustments = merge_adjustments(
                    cumulative_adjustments, new_adjustments)

    except IntegrationPending as exc:
        waiting_status = WAITING_STATUS[exc.component]
        append_workflow_event(run_id, base_attempt_id, waiting_status, {
            "component": exc.component, "message": str(exc)})
        mark_status(
            run_id, base_attempt_id, waiting_status, error_message=str(exc))
    except BaseException as exc:
        append_workflow_event(run_id, base_attempt_id, "failed", {
            "error_type": type(exc).__name__, "message": str(exc)})
        mark_status(
            run_id, base_attempt_id, "failed",
            error_message="{}: {}".format(type(exc).__name__, exc))
