from __future__ import annotations

from typing import Any, Protocol


class IntegrationPending(RuntimeError):
    """A required external component has not been connected yet."""

    def __init__(self, component: str, message: str | None = None):
        """为尚未接入的组件构造异常。

        输入
            component：尚未接入的组件标识。
            message：补充错误说明。
        输出
            None。
        """
        self.component = component
        super().__init__(message or f"{component} is not integrated")


class SimulationEngine(Protocol):
    def simulate(self, simulation_input: dict, output_sink: "SimulationOutputSink") -> dict:
        """定义仿真引擎执行及向输出接收器发送结果的接口契约。

        输入
            simulation_input：旧版单次仿真引擎输入。
            output_sink：接收帧、统计和移交的输出对象。
        输出
            引擎结果字典。

        原有说明
            Run planning, motion, avoidance, statistics, and handoff generation.
        """


class SimulationOutputSink(Protocol):
    def on_frame(self, frame: dict) -> None:
        """接收一帧无人机状态并交给具体输出实现。

        输入
            frame：当前帧的无人机状态记录。
        输出
            None。
        """
        ...
    def on_statistics(self, statistics: dict) -> None:
        """接收一批道路统计并交给具体输出实现。

        输入
            statistics：道路统计记录或历史批次。
        输出
            None。
        """
        ...
    def on_handoff(self, handoff: dict) -> None:
        """接收一次移交并交给具体输出实现。

        输入
            handoff：待记录的跨区移交载荷。
        输出
            None。
        """
        ...


def build_simulation_input(
    request: dict,
    snapshot: dict,
    topology_adjustments: dict[str, Any],
    round_index: int,
) -> dict:
    """从有效快照、导引和运行参数组装单次引擎输入。

    输入
        request：区域任务请求及运行配置。
        snapshot：区域数据库快照。
        topology_adjustments：本次仿真采用的网络调整。
        round_index：该任务内从零开始的调整轮次。
    输出
        仿真输入字典。

    原有说明
        Build the stable DTO handed to the external simulation engine.
    """
    return {
        "contract_version": "region-dt-simulation-input/v1",
        "run_id": request["run_id"],
        "attempt_id": int(request["attempt_id"]),
        "round_index": int(round_index),
        "region_id": int(request["region_id"]),
        "snapshot_time": str(snapshot["snapshot_time"]),
        "topology_version": int(snapshot["topology_version"]),
        "horizon_seconds": int(request["horizon_seconds"]),
        "step_seconds": float(request["step_seconds"]),
        "random_seed": int(request.get("random_seed", 42)),
        "network": snapshot["network"],
        "uavs": snapshot["uavs"],
        "guidance": {"transfers": request.get("transfers", [])},
        "topology_adjustments": topology_adjustments,
    }


def validate_engine_result(result: dict) -> dict:
    """检查引擎结果类型及所需字段。

    输入
        result：待检查或整理的仿真结果。
    输出
        通过校验的结果字典。

    原有说明
        Validate only the outer result envelope; algorithms own inner schemas.
    """
    if not isinstance(result, dict):
        raise ValueError("simulation engine must return a dictionary")
    required = ("statistics", "handoff_events", "diagnostics")
    missing = [name for name in required if name not in result]
    if missing:
        raise ValueError(f"simulation engine result is missing fields: {missing}")
    if not isinstance(result["handoff_events"], list):
        raise ValueError("handoff_events must be a list")
    return result
