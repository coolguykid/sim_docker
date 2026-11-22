from __future__ import annotations

import asyncio
import json
from contextlib import asynccontextmanager
from datetime import datetime

from fastapi import FastAPI, HTTPException, Request, Response, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse

from models import GuidanceRequest, HandoffInput
from repository import (
    REGION_ID,
    database_ready,
    enqueue_request,
    get_task,
    has_active_task,
    initialize_control_schema,
    list_outbox,
    list_queued_keys,
    list_workflow_events,
    mark_status,
    requeue_task,
    store_inbox,
    waiting_integration_status,
)
from workflow import run_workflow
from topology_reader import load_topology
from simulation_result_reader import (
    frame_times_after,
    latest_result,
    load_snapshot as load_result_snapshot,
)
from distributed_runtime import (
    batch_frame,
    advance as advance_distributed,
    commit_substep as commit_distributed_substep,
    finish_frame as finish_distributed_frame,
    get_result as distributed_result,
    get_summary as distributed_summary,
    prepare_substep as prepare_distributed_substep,
    queue_handoffs as queue_distributed_handoffs,
)
from coordinator_client import close_client as close_coordinator_client


queue: asyncio.Queue[tuple[str, int]] = asyncio.Queue()
worker_task: asyncio.Task | None = None


def public_task(row: dict) -> dict:
    """提取区域数据库任务的公开状态字段。

    输入
        row：待投影、读取或转换的一条记录。
    输出
        公开任务字典。
    """
    return {
        key: row.get(key)
        for key in (
            "run_id", "attempt_id", "region_id", "status", "snapshot_summary",
            "result_summary", "error_message", "created_at", "started_at", "finished_at",
        )
    }


def execute(run_id: str, attempt_id: int) -> None:
    """从数据库载入指定任务并执行区域工作流。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        None。
    """
    task = get_task(run_id, attempt_id)
    if task is not None:
        run_workflow(dict(task["request"]))


async def worker_loop() -> None:
    """从异步工作队列取任务，在线程中顺序执行。

    输入
        无显式参数。
    输出
        持续运行，取消时退出。
    """
    while True:
        key = await queue.get()
        try:
            await asyncio.to_thread(execute, *key)
        finally:
            queue.task_done()


@asynccontextmanager
async def lifespan(_app: FastAPI):
    """在应用启动和关闭时管理服务资源。

    输入
        _app：交由生命周期函数管理的 FastAPI 应用。
    输出
        异步上下文，进入时交出控制权，退出时清理。
    """
    global worker_task
    initialize_control_schema()
    for key in list_queued_keys():
        queue.put_nowait(key)
    worker_task = asyncio.create_task(worker_loop())
    try:
        yield
    finally:
        worker_task.cancel()
        try:
            await worker_task
        except asyncio.CancelledError:
            pass
        close_coordinator_client()


app = FastAPI(title="Regional DT Service", version="0.1.0", lifespan=lifespan)

# REGION_DB_TOPOLOGY: the review frontend is served by a local static server on
# another port.  Only read-only GET access is required for this first test.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:8767", "http://localhost:8767"],
    allow_methods=["GET"],
    allow_headers=["*"],
)


@app.get("/health")
def health():
    """报告服务健康状态和可用性。

    输入
        无显式参数。
    输出
        健康检查字典。
    """
    if not database_ready():
        raise HTTPException(status_code=503, detail="database unavailable")
    active = has_active_task()
    waiting = waiting_integration_status()
    simulator_status = "busy" if active else (waiting or "waiting_guidance")
    return {
        "status": simulator_status,
        "region_id": REGION_ID,
        "database": "ready",
        "simulator": simulator_status,
    }


@app.get("/api/v1/topology")
def regional_topology(version_id: int | None = None, at: datetime | None = None):
    """按版本和状态时刻查询区域拓扑。

    输入
        version_id：数据库拓扑版本号。
        at：查询设施状态的时间。
    输出
        拓扑 DTO。

    原有说明
        Expose this container's database topology without starting simulation.
    """
    try:
        return load_topology(version_id, at)
    except LookupError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.get("/api/v1/simulation-results/latest")
def latest_simulation_result():
    """查询本区域最新仿真结果的元数据。

    输入
        无显式参数。
    输出
        最新尝试摘要。

    原有说明
        Return the newest regional CSV attempt without loading all frames.
    """
    try:
        return latest_result()
    except LookupError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@app.get("/api/v1/simulation-results/{run_id}/attempts/{attempt_id}/snapshot")
def simulation_result_snapshot(run_id: str, attempt_id: int, time: float):
    """读取指定尝试在请求时刻的 CSV 帧和统计。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        time：请求的仿真时刻，单位秒。
    输出
        结果快照。

    原有说明
        Return one UAV frame and the latest statistics batch at that time.
    """
    try:
        return load_result_snapshot(run_id, attempt_id, time)
    except LookupError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.get("/api/v1/simulation-results/stream")
async def stream_simulation_results(
        request: Request,
        run_id: str | None = None,
        attempt_id: int | None = None,
        after: float | None = None):
    """通过 SSE 推送运行结果中的帧更新。

    输入
        request：用于检查连接是否仍存活的 HTTP 请求。
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        after：下界仿真时刻，不含该时刻。
    输出
        流式 HTTP 响应。

    原有说明
        Push each completed regional CSV frame as a Server-Sent Event.
        
        The stream also watches for a newer run, so a frontend opened before
        guidance arrives automatically starts displaying that simulation.
    """
    async def events():
        """生成携带帧内容的 SSE 事件。

        输入
            无显式参数。
        输出
            异步迭代的事件文本。
        """
        current = ((str(run_id), int(attempt_id))
                   if run_id is not None and attempt_id is not None else None)
        last_time = float(after) if after is not None else None
        announced = None
        while not await request.is_disconnected():
            try:
                metadata = await asyncio.to_thread(latest_result)
                newest = (str(metadata["runId"]), int(metadata["attemptId"]))
                if current != newest:
                    current = newest
                    last_time = None
                if announced != current:
                    announced = current
                    yield "event: run\ndata: {}\n\n".format(
                        json.dumps(metadata, ensure_ascii=False, separators=(",", ":")))
                times = await asyncio.to_thread(
                    frame_times_after, current[0], current[1], last_time)
                for sim_time in times:
                    snapshot = await asyncio.to_thread(
                        load_result_snapshot, current[0], current[1], sim_time)
                    yield "event: snapshot\ndata: {}\n\n".format(
                        json.dumps(snapshot, ensure_ascii=False, separators=(",", ":")))
                    last_time = sim_time
            except LookupError:
                pass
            yield ": keepalive\n\n"
            await asyncio.sleep(0.2)
    return StreamingResponse(
        events(), media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.post("/api/v1/guidance", status_code=status.HTTP_202_ACCEPTED)
async def receive_guidance(request: GuidanceRequest, response: Response):
    """校验并登记导引请求，必要时排入执行队列。

    输入
        request：区域任务请求及运行配置。
        response：用于设置状态码的 HTTP 响应对象。
    输出
        登记结果，HTTP 状态随请求处理结果更新。
    """
    if request.region_id != REGION_ID:
        raise HTTPException(
            status_code=422,
            detail=f"guidance region_id={request.region_id} does not match container region_id={REGION_ID}",
        )
    payload = request.model_dump(mode="json")
    try:
        created, existing = enqueue_request(payload)
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    if created:
        await queue.put((request.run_id, request.attempt_id))
    else:
        response.status_code = status.HTTP_200_OK
    return {
        "accepted": True,
        "created": created,
        "region_id": REGION_ID,
        "run_id": request.run_id,
        "attempt_id": request.attempt_id,
        "status": "queued" if created else existing["status"],
    }


@app.get("/api/v1/simulations/{run_id}/attempts/{attempt_id}")
def simulation_status(run_id: str, attempt_id: int):
    """读取指定区域仿真任务状态。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        公开任务字典。
    """
    task = get_task(run_id, attempt_id)
    if task is None:
        raise HTTPException(status_code=404, detail="simulation attempt not found")
    return public_task(task)


@app.get("/api/v1/simulations/{run_id}/attempts/{attempt_id}/handoffs")
def simulation_handoffs(run_id: str, attempt_id: int):
    """查询指定尝试的出站移交记录。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        移交记录列表。
    """
    return {"events": list_outbox(run_id, attempt_id)}


@app.get("/api/v1/simulations/{run_id}/attempts/{attempt_id}/events")
def simulation_events(run_id: str, attempt_id: int):
    """查询指定尝试的工作流事件。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        事件记录。
    """
    return {"events": list_workflow_events(run_id, attempt_id)}


@app.post("/api/v1/simulations/{run_id}/attempts/{attempt_id}/resume", status_code=202)
async def resume_simulation(run_id: str, attempt_id: int):
    """将可恢复的任务重新加入执行队列。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        恢复状态。
    """
    if not requeue_task(run_id, attempt_id):
        raise HTTPException(
            status_code=409,
            detail="only failed or waiting-integration tasks can be resumed",
        )
    await queue.put((run_id, attempt_id))
    return {"accepted": True, "run_id": run_id, "attempt_id": attempt_id, "status": "queued"}


@app.post("/api/v1/handoffs", status_code=status.HTTP_202_ACCEPTED)
def receive_handoff(event: HandoffInput, response: Response):
    """校验并幂等登记外部移交。

    输入
        event：单条移交或工作流事件。
        response：用于设置状态码的 HTTP 响应对象。
    输出
        登记状态。
    """
    if event.to_region != REGION_ID:
        raise HTTPException(status_code=422, detail="handoff target does not match this region")
    created = store_inbox(event.model_dump(mode="json"))
    if not created:
        response.status_code = status.HTTP_200_OK
    return {
        "accepted": True,
        "created": created,
        "event_id": event.event_id,
        "status": "received",
        "note": "phase 1 persists inbound handoffs; synchronized insertion is implemented with region-to-region scheduling",
    }


@app.post("/api/v1/distributed/runs/{run_id}/attempts/{attempt_id}/handoffs", status_code=202)
def submit_distributed_handoffs(run_id: str, attempt_id: int, payload: dict):
    """向本区内存会话提交一批跨区移交。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        接收状态和会话摘要。
    """
    try:
        summary = queue_distributed_handoffs(
            run_id, attempt_id, payload.get("events", ()))
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    return {"accepted": True, "summary": summary}


@app.post("/api/v1/distributed/runs/{run_id}/attempts/{attempt_id}/step")
def step_distributed_run(run_id: str, attempt_id: int, payload: dict):
    """通过 HTTP 推进本区一个单步。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        区域推进结果。
    """
    try:
        return advance_distributed(
            run_id, attempt_id, int(payload["step_index"]), float(payload["sim_time"]))
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/v1/distributed/runs/{run_id}/attempts/{attempt_id}/events/prepare")
def prepare_distributed_event(run_id: str, attempt_id: int, payload: dict):
    """通过 HTTP 请求本区准备一个事件子步。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        准备令牌与时间预测。
    """
    try:
        return prepare_distributed_substep(
            run_id, attempt_id, int(payload["frame_index"]),
            float(payload["sim_time"]), float(payload["max_duration"]))
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/v1/distributed/runs/{run_id}/attempts/{attempt_id}/events/commit")
def commit_distributed_event(run_id: str, attempt_id: int, payload: dict):
    """通过 HTTP 提交区域已准备的子步。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        区域提交结果与出境事件。
    """
    try:
        return commit_distributed_substep(
            run_id, attempt_id, str(payload["prepare_token"]),
            float(payload["advance_to"]))
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/v1/distributed/runs/{run_id}/attempts/{attempt_id}/batch/{phase}")
def batch_distributed_frame(run_id: str, attempt_id: int, phase: str, payload: dict):
    """通过 HTTP 执行批量帧的准备、应用或回滚。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        phase：批量帧操作阶段，取 prepare、apply 或 rollback。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        对应阶段的区域响应。
    """
    try:
        return batch_frame(run_id, attempt_id, phase, payload)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc

@app.post("/api/v1/distributed/runs/{run_id}/attempts/{attempt_id}/events/frame")
def finish_distributed_event_frame(run_id: str, attempt_id: int, payload: dict):
    """通过 HTTP 结束区域当前事件帧。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        帧摘要。
    """
    try:
        return finish_distributed_frame(
            run_id, attempt_id, int(payload["frame_index"]),
            float(payload["frame_time"]))
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.get("/api/v1/distributed/runs/{run_id}/attempts/{attempt_id}")
def get_distributed_run(run_id: str, attempt_id: int):
    """通过 HTTP 查询活动区域会话。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        区域会话摘要。
    """
    summary = distributed_summary(run_id, attempt_id)
    if summary is None:
        raise HTTPException(status_code=404, detail="regional run is not initialized")
    return summary


@app.get("/api/v1/distributed/runs/{run_id}/attempts/{attempt_id}/result")
def get_distributed_result(run_id: str, attempt_id: int):
    """通过 HTTP 查询区域会话结果。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        区域结果字典。
    """
    result = distributed_result(run_id, attempt_id)
    if result is None:
        raise HTTPException(status_code=404, detail="regional attempt is not initialized")
    return result
