from __future__ import annotations

from engine_contract import IntegrationPending
from network_adjustment_adapter import invoke_capacity_control


def invoke_simulation_engine(simulation_input: dict, output_sink) -> dict:
    """标记旧版单次引擎入口未接入，分布式运行由协调器推进。

    输入
        simulation_input：旧版单次仿真引擎输入。
        output_sink：接收帧、统计和移交的输出对象。
    输出
        抛出 IntegrationPending。

    原有说明
        Legacy one-shot workflow port; distributed runs use coordinator steps.
    """
    raise IntegrationPending(
        "simulation_engine",
        "one-shot engine is disabled; submit distributed runs to simulation-coordinator",
    )


def evaluate_statistics(engine_result: dict, request: dict, *, consecutive=None) -> dict:
    """依据完整统计窗口和拥堵阈值判断区域是否需要调整。

    输入
        engine_result：区域仿真结果和规划失败记录。
        request：区域任务请求及运行配置。
    输出
        评估决策、严重拥堵记录与策略参数。

    原有说明
        Evaluate sustained low-speed, high-occupancy congestion locally.
    """
    policy = request.get("congestion_policy") or {}
    capacity_ratio = float(policy.get("capacity_ratio", 0.8))
    speed_ratio = float(policy.get("speed_ratio", 0.5))
    duration = float(policy.get("duration_seconds", 15.0))
    interval = float(policy.get("statistics_interval_seconds", 5.0))
    if consecutive is None:
        consecutive = {}
    severe = []
    for batch in engine_result.get("statistics", ()):
        active = set()
        # A partial window is useful for display, but it must not trigger a
        # decision that is defined on a complete 60-second sliding window.
        if batch.get("warmup"):
            continue
        for edge in batch.get("edges", ()):
            capacity = max(1.0, float(edge.get("max_capacity", 50)))
            speed_limit = max(1e-9, float(edge.get("speed_limit", 15.0)))
            average_speed = edge.get("window_space_mean_speed")
            congested = (
                float(edge.get("window_average_uav_count", 0.0)) / capacity >= capacity_ratio
                and average_speed is not None
                and float(average_speed) / speed_limit <= speed_ratio)
            key = "{}:{}".format(edge["edge_type"], edge["edge_id"])
            if congested:
                active.add(key)
                consecutive[key] = consecutive.get(key, 0.0) + interval
                if consecutive[key] >= duration:
                    severe.append({
                        "edge": key,
                        "duration_seconds": consecutive[key],
                        "time_s": float(batch["time"]),
                    })
            else:
                consecutive[key] = 0.0
        for key in set(consecutive) - active:
            consecutive[key] = 0.0
    if engine_result.get("planning_failures"):
        severe.append({"edge": None, "reason": "planning_failures"})
    return {
        "decision": "adjustment_required" if severe else "acceptable",
        "severe_congestion": severe,
        "policy": {
            "capacity_ratio": capacity_ratio, "speed_ratio": speed_ratio,
            "duration_seconds": duration,
        },
    }


def invoke_network_adjustment(adjustment_input: dict) -> dict:
    """调用区域航道容量调整适配器。

    输入
        adjustment_input：容量控制输入，包含有效拓扑与仿真统计。
    输出
        调整结果。

    原有说明
        Invoke the adapted airway-capacity controller on one regional result.
    """
    return invoke_capacity_control(adjustment_input)


def dispatch_guidance(dispatch_input: dict) -> dict:
    """预留将通过评估的导引下发至运行系统的接口。

    输入
        dispatch_input：待下发的导引和有效拓扑。
    输出
        当前抛出 IntegrationPending。

    原有说明
        Integration point that sends accepted guidance to the running subsystem.
    """
    raise IntegrationPending(
        "guidance_dispatch",
        "running-system guidance endpoint is not connected; implement dispatch_guidance()",
    )
