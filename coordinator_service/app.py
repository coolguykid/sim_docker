"""A deliberately small step barrier and handoff relay.

The coordinator never loads road networks/UAVs, interprets guidance, computes
statistics, evaluates congestion, adjusts routes, or dispatches guidance.
"""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from datetime import datetime, timezone
import os
import time
import traceback

from fastapi import FastAPI, HTTPException, status
import httpx


ATTEMPTS = {}
TASKS = {}
HTTP_CLIENT: httpx.AsyncClient | None = None

HTTP_TIMEOUT_SECONDS = 120.0
HTTP_KEEPALIVE_SECONDS = 300.0
HTTP_MAX_CONNECTIONS = 96
HTTP_MAX_KEEPALIVE_CONNECTIONS = 70


def _key(run_id, attempt_id):
    """规范化运行 ID 与尝试编号，供内存索引使用。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        二元组 (run_id, attempt_id)。
    """
    return str(run_id), int(attempt_id)


async def _post(url, payload):
    """通过共享异步连接池向区域服务发送 JSON POST。

    输入
        url：目标 HTTP URL。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        解码后的 JSON 响应，通信失败时抛出异常。

    原有说明
        POST through the process-wide pool so region connections are reused.
    """
    client = HTTP_CLIENT
    if client is None:
        raise RuntimeError("coordinator HTTP client is not initialized")
    try:
        response = await client.post(url, json=payload)
        response.raise_for_status()
        return response.json()
    except httpx.HTTPStatusError as exc:
        raise RuntimeError(
            "HTTP {} {}: {}".format(
                exc.response.status_code, url, exc.response.text)) from exc
    except httpx.HTTPError as exc:
        raise RuntimeError("HTTP request failed {}: {}".format(url, exc)) from exc


def public_attempt(record):
    """投影协调器记录，供状态接口返回。

    输入
        record：协调器维护的尝试记录。
    输出
        公开字段字典。
    """
    return {
        name: record.get(name) for name in (
            "run_id", "attempt_id", "status", "expected_region_ids",
            "registered_region_ids", "completed_steps", "sim_time",
            "handoff_count", "handoff_pairs", "decisions", "collective_decision", "error",
            "timing", "created_at", "started_at", "finished_at",
        )
    }


async def drive_attempt(record):
    """推进所有区域的尝试，处理事件同步、移交与帧结束并写入计时。

    输入
        record：协调器维护的尝试记录。
    输出
        None，原地更新运行记录。
    """
    try:
        drive_started = time.perf_counter()
        record["status"] = "running"
        record["started_at"] = datetime.now(timezone.utc).isoformat()
        dt = float(record["step_seconds"])
        steps = int(round(float(record["horizon_seconds"]) / dt))
        regions = record["regions"]
        step_wall_seconds = []
        handoff_wall_total = 0.0
        event_substep_total = 0
        batch_enabled = os.environ.get('SIM_FRAME_BATCH') == '1'
        fallback_frames = 0
        event_substeps_per_frame = []
        epsilon = 1e-8

        def endpoint(region_id, suffix):
            """依据区域注册的服务地址拼接本次尝试的接口路径。

            输入
                region_id：区域编号。
                suffix：区域内部接口的相对路径。
            输出
                区域接口 URL。
            """
            return (
                regions[region_id]["endpoint"] +
                "/api/v1/distributed/runs/{}/attempts/{}/{}".format(
                    record["run_id"], record["attempt_id"], suffix))

        async def relay_handoffs(results):
            """将提交结果中的出境事件按接收区分组转发，并更新计数。

            输入
                results：各区域返回的提交结果。
            输出
                本轮转发的事件数。
            """
            nonlocal handoff_wall_total
            grouped = {}
            for result in results:
                for event in result.get("outbound_handoffs", ()):
                    grouped.setdefault(int(event["to_region"]), []).append(event)
                    pair = "{}->{}".format(event["from_region"], event["to_region"])
                    record["handoff_pairs"][pair] = (
                        record["handoff_pairs"].get(pair, 0) + 1)
            if not grouped:
                return 0
            handoff_started = time.perf_counter()
            await asyncio.gather(*[
                _post(endpoint(target, "handoffs"), {"events": events})
                for target, events in grouped.items()
            ])
            handoff_wall_total += time.perf_counter() - handoff_started
            count = sum(len(events) for events in grouped.values())
            record["handoff_count"] += count
            return count

        for step_index in range(steps):
            step_started = time.perf_counter()
            frame_start = step_index * dt
            frame_end = frame_start + dt
            cursor = frame_start
            frame_substeps = 0
            if batch_enabled:
                ids = record["expected_region_ids"]
                batches = await asyncio.gather(*[
                    _post(endpoint(region_id, "batch/prepare"), {
                        "frame_index": step_index, "start": frame_start, "end": frame_end,
                    }) for region_id in ids
                ])
                inboxes = {region_id: [] for region_id in ids}
                for batch in batches:
                    for event in batch["events"]:
                        inboxes[int(event["to_region"])].append(event)
                applied = await asyncio.gather(*[
                    _post(endpoint(region_id, "batch/apply"), {
                        "frame_index": step_index, "end": frame_end,
                        "events": sorted(inboxes[region_id], key=lambda event: (
                            event["crossing_time"], event["from_region"])),
                        "expected": batch["events"],
                    }) for region_id, batch in zip(ids, batches)
                ])
                if all(result["matches"] for result in applied):
                    cursor = frame_end
                    for batch in batches:
                        for event in batch["events"]:
                            pair = "{}->{}".format(event["from_region"], event["to_region"])
                            record["handoff_pairs"][pair] = record["handoff_pairs"].get(pair, 0) + 1
                            record["handoff_count"] += 1
                else:
                    await asyncio.gather(*[
                        _post(endpoint(region_id, "batch/rollback"), {}) for region_id in ids
                    ])
                    fallback_frames += 1
            while cursor < frame_end - epsilon:
                remaining = frame_end - cursor
                prepared = await asyncio.gather(*[
                    _post(endpoint(region_id, "events/prepare"), {
                        "frame_index": step_index,
                        "sim_time": cursor,
                        "max_duration": remaining,
                    }) for region_id in record["expected_region_ids"]
                ])
                candidates = [
                    float(result["next_event_time"])
                    for result in prepared
                    if result.get("next_event_time") is not None
                ]
                deadline = min([frame_end] + candidates)
                if deadline < cursor - epsilon or deadline > frame_end + epsilon:
                    raise RuntimeError(
                        "region reported an event outside the prepared frame")
                if abs(deadline-cursor) <= epsilon:
                    deadline = cursor
                if abs(deadline-frame_end) <= epsilon:
                    deadline = frame_end
                committed = await asyncio.gather(*[
                    _post(endpoint(region_id, "events/commit"), {
                        "prepare_token": result["prepare_token"],
                        "advance_to": deadline,
                    }) for region_id, result in zip(
                        record["expected_region_ids"], prepared)
                ])
                handoff_count = await relay_handoffs(committed)
                frame_substeps += 1
                event_substep_total += 1
                if frame_substeps > int(record.get("max_event_substeps_per_frame", 10000)):
                    raise RuntimeError("event substep limit exceeded in one output frame")
                if deadline <= cursor + epsilon and handoff_count == 0:
                    raise RuntimeError("zero-time event cycle made no handoff progress")
                cursor = deadline

            await asyncio.gather(*[
                _post(endpoint(region_id, "events/frame"), {
                    "frame_index": step_index,
                    "frame_time": frame_end,
                }) for region_id in record["expected_region_ids"]
            ])
            event_substeps_per_frame.append(frame_substeps)
            step_wall_seconds.append(time.perf_counter() - step_started)
            ordered = sorted(step_wall_seconds)
            percentile = lambda q: ordered[min(
                len(ordered) - 1, max(0, int(round((len(ordered) - 1) * q))))]
            record["timing"] = {
                "completed_frame_count": len(step_wall_seconds),
                "simulation_wall_seconds": time.perf_counter() - drive_started,
                "frame_wall_seconds_average": sum(step_wall_seconds) / len(step_wall_seconds),
                "frame_wall_seconds_min": ordered[0],
                "frame_wall_seconds_p50": percentile(0.50),
                "frame_wall_seconds_p95": percentile(0.95),
                "frame_wall_seconds_max": ordered[-1],
                "handoff_relay_wall_seconds_total": handoff_wall_total,
                "scheduler": "frame_batch" if os.getenv("SIM_FRAME_BATCH") == "1" else "strict_event_barrier",
                "batch_fallback_frames": fallback_frames,
                "event_substep_count": event_substep_total,
                "event_substeps_per_frame_average": (
                    sum(event_substeps_per_frame) /
                    len(event_substeps_per_frame)),
                "event_substeps_per_frame_max": max(event_substeps_per_frame),
            }
            record["completed_steps"] = step_index + 1
            record["sim_time"] = frame_end
        record["status"] = "simulation_completed"
        record["finished_at"] = datetime.now(timezone.utc).isoformat()
    except Exception as exc:
        record["status"] = "failed"
        record["error"] = "{}: {}".format(type(exc).__name__, exc)
        record["traceback"] = traceback.format_exc()
        record["finished_at"] = datetime.now(timezone.utc).isoformat()


@asynccontextmanager
async def lifespan(_app: FastAPI):
    """在应用启动和关闭时管理服务资源。

    输入
        _app：交由生命周期函数管理的 FastAPI 应用。
    输出
        异步上下文，进入时交出控制权，退出时清理。

    原有说明
        Own one connection pool for the lifetime of the coordinator process.
    """
    global HTTP_CLIENT
    HTTP_CLIENT = httpx.AsyncClient(
        timeout=httpx.Timeout(HTTP_TIMEOUT_SECONDS),
        limits=httpx.Limits(
            max_connections=HTTP_MAX_CONNECTIONS,
            max_keepalive_connections=HTTP_MAX_KEEPALIVE_CONNECTIONS,
            keepalive_expiry=HTTP_KEEPALIVE_SECONDS,
        ),
        headers={"Content-Type": "application/json; charset=utf-8"},
    )
    try:
        yield
    finally:
        for task in TASKS.values():
            if not task.done():
                task.cancel()
        if TASKS:
            await asyncio.gather(*TASKS.values(), return_exceptions=True)
        await HTTP_CLIENT.aclose()
        HTTP_CLIENT = None


app = FastAPI(
    title="Regional DT Simulation Synchronizer",
    version="0.4.0",
    lifespan=lifespan,
)


@app.get("/health")
def health():
    """报告服务健康状态和可用性。

    输入
        无显式参数。
    输出
        健康检查字典。
    """
    return {
        "status": "ready",
        "role": "strict-event-barrier-and-handoff-relay",
        "scheduler": "frame_batch" if os.getenv("SIM_FRAME_BATCH") == "1" else "strict_event_barrier",
        "region_transport": "http/1.1-keep-alive",
        "max_connections": HTTP_MAX_CONNECTIONS,
        "max_keepalive_connections": HTTP_MAX_KEEPALIVE_CONNECTIONS,
        "keepalive_seconds": HTTP_KEEPALIVE_SECONDS,
    }


@app.post(
    "/api/v1/runs/{run_id}/attempts/{attempt_id}/regions/{region_id}/register",
    status_code=status.HTTP_202_ACCEPTED,
)
async def register_region(run_id: str, attempt_id: int, region_id: int, payload: dict):
    """登记参与仿真的区域，在参与方齐备时启动协调任务。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        region_id：区域编号。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        当前尝试的公开状态。
    """
    key = _key(run_id, attempt_id)
    expected = sorted(int(value) for value in payload["expected_region_ids"])
    record = ATTEMPTS.get(key)
    if record is None:
        record = {
            "run_id": run_id, "attempt_id": int(attempt_id),
            "status": "waiting_regions", "expected_region_ids": expected,
            "regions": {}, "registered_region_ids": [],
            "horizon_seconds": int(payload["horizon_seconds"]),
            "step_seconds": float(payload["step_seconds"]),
            "max_event_substeps_per_frame": int(
                payload.get("max_event_substeps_per_frame", 10000)),
            "completed_steps": 0, "sim_time": 0.0, "handoff_count": 0,
            "handoff_pairs": {},
            "timing": {},
            "decisions": {}, "collective_decision": None, "error": None,
            "created_at": datetime.now(timezone.utc).isoformat(),
        }
        ATTEMPTS[key] = record
    if record["expected_region_ids"] != expected:
        raise HTTPException(status_code=409, detail="expected region set mismatch")
    if int(region_id) not in expected:
        raise HTTPException(status_code=422, detail="region is not part of this attempt")
    record["regions"][int(region_id)] = {"endpoint": str(payload["endpoint"])}
    record["registered_region_ids"] = sorted(record["regions"])
    if (record["status"] == "waiting_regions" and
            record["registered_region_ids"] == record["expected_region_ids"]):
        TASKS[key] = asyncio.create_task(drive_attempt(record))
    return public_attempt(record)


@app.get("/api/v1/runs/{run_id}/attempts/{attempt_id}")
def get_attempt(run_id: str, attempt_id: int):
    """按运行与尝试编号读取协调器状态。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        尝试状态字典。
    """
    record = ATTEMPTS.get(_key(run_id, attempt_id))
    if record is None:
        raise HTTPException(status_code=404, detail="attempt not found")
    return public_attempt(record)


@app.post(
    "/api/v1/runs/{run_id}/attempts/{attempt_id}/regions/{region_id}/decision",
    status_code=status.HTTP_202_ACCEPTED,
)
def report_decision(run_id: str, attempt_id: int, region_id: int, payload: dict):
    """上报区域评估结果，收齐后归并集体决策。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        region_id：区域编号。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        当前尝试状态或协调器响应。
    """
    record = ATTEMPTS.get(_key(run_id, attempt_id))
    if record is None:
        raise HTTPException(status_code=404, detail="attempt not found")
    if int(region_id) not in record["expected_region_ids"]:
        raise HTTPException(status_code=422, detail="region is not part of this attempt")
    if record["status"] not in ("simulation_completed", "decisions_completed"):
        raise HTTPException(status_code=409, detail="simulation is not complete")
    decision = str(payload["decision"])
    if decision not in ("acceptable", "adjustment_required"):
        raise HTTPException(status_code=422, detail="unsupported decision")
    record["decisions"][str(region_id)] = decision
    if len(record["decisions"]) == len(record["expected_region_ids"]):
        record["collective_decision"] = (
            "rerun_required"
            if "adjustment_required" in record["decisions"].values()
            else "acceptable")
        record["status"] = "decisions_completed"
    return public_attempt(record)
