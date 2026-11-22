"""In-memory local sessions prepared and owned by the regional workflow."""
from __future__ import annotations

from threading import RLock

from repository import REGION_ID
from simulation_engine import LocalSimulationSession


_sessions = {}
_lock = RLock()


def _key(run_id, attempt_id):
    """规范化运行 ID 与尝试编号，供内存索引使用。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        二元组 (run_id, attempt_id)。
    """
    return str(run_id), int(attempt_id)


def initialize_session(config):
    """在锁保护下创建或复用指定尝试的区域会话。

    输入
        config：区域会话配置，包含拓扑、任务、导引及输出目录。
    输出
        会话摘要及是否新建的布尔值。
    """
    if int(config["region_id"]) != REGION_ID:
        raise ValueError("session region does not match container")
    key = _key(config["run_id"], config["attempt_id"])
    with _lock:
        existing = _sessions.get(key)
        if existing is not None:
            return existing.summary(), False
        session = LocalSimulationSession(config)
        _sessions[key] = session
        return session.summary(), True


def queue_handoffs(run_id, attempt_id, events):
    """在锁保护下向已有区域会话排入移交。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        events：待处理的事件序列。
    输出
        更新后的会话摘要。
    """
    with _lock:
        session = _sessions.get(_key(run_id, attempt_id))
        if session is None:
            raise KeyError("regional attempt is not initialized")
        session.queue_handoffs(events)
        return session.summary()


def advance(run_id, attempt_id, step_index, sim_time):
    """在锁保护下调用区域单步推进。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        step_index：从零开始的仿真步编号。
        sim_time：当前仿真时刻，单位秒。
    输出
        区域单步结果。
    """
    with _lock:
        session = _sessions.get(_key(run_id, attempt_id))
        if session is None:
            raise KeyError("regional attempt is not initialized")
        return session.step(step_index, sim_time)


def prepare_substep(run_id, attempt_id, frame_index, sim_time, max_duration):
    """在区域会话锁内委托准备事件子步。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        frame_index：从零开始的输出帧编号。
        sim_time：当前仿真时刻，单位秒。
        max_duration：该子步允许推进的最长仿真秒数。
    输出
        准备令牌和最早事件时刻。
    """
    with _lock:
        session = _sessions.get(_key(run_id, attempt_id))
        if session is None:
            raise KeyError("regional attempt is not initialized")
        return session.prepare_substep(
            frame_index, sim_time, max_duration)


def commit_substep(run_id, attempt_id, prepare_token, advance_to):
    """在区域会话锁内委托提交已准备的子步。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        prepare_token：区域准备阶段生成的提交令牌。
        advance_to：本事件子步统一推进至的仿真时刻。
    输出
        推进结果和出境事件。
    """
    with _lock:
        session = _sessions.get(_key(run_id, attempt_id))
        if session is None:
            raise KeyError("regional attempt is not initialized")
        return session.commit_substep(prepare_token, advance_to)


def batch_frame(run_id, attempt_id, phase, payload):
    """将批量帧请求分派至准备、应用或回滚阶段。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        phase：批量帧操作阶段，取 prepare、apply 或 rollback。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        对应阶段的区域结果。
    """
    with _lock:
        session = _sessions[_key(run_id, attempt_id)]
        if phase == "prepare":
            return session.prepare_batch_frame(
                int(payload["frame_index"]), float(payload["start"]), float(payload["end"]))
        if phase == "apply":
            return session.apply_batch_frame(
                int(payload["frame_index"]), float(payload["end"]),
                payload["events"], payload["expected"])
        if phase == "rollback":
            return session.discard_batch_frame(rollback=True)
        raise ValueError("unknown batch frame phase")

def finish_frame(run_id, attempt_id, frame_index, frame_time):
    """在锁保护下通知区域会话结算指定帧。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        frame_index：从零开始的输出帧编号。
        frame_time：帧结束时刻，单位秒。
    输出
        帧摘要。
    """
    with _lock:
        session = _sessions.get(_key(run_id, attempt_id))
        if session is None:
            raise KeyError("regional attempt is not initialized")
        return session.finish_frame(frame_index, frame_time)


def get_summary(run_id, attempt_id):
    """查询已登记区域会话的摘要。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        摘要字典，找不到时为 None。
    """
    with _lock:
        session = _sessions.get(_key(run_id, attempt_id))
        return None if session is None else session.summary()


def get_result(run_id, attempt_id):
    """查询已登记区域会话的结果。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        结果字典，找不到时为 None。
    """
    with _lock:
        session = _sessions.get(_key(run_id, attempt_id))
        return None if session is None else session.result()


def finalize(run_id, attempt_id):
    """从注册表移除区域会话并关闭运行资源。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        关闭前取得的区域结果。
    """
    with _lock:
        session = _sessions.pop(_key(run_id, attempt_id), None)
        if session is None:
            raise KeyError("regional attempt is not initialized")
        result = session.result()
        session.close()
        return result
