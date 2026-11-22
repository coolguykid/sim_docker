"""算法 [1]：纯 Dijkstra + 规则避障，串行与分布式共用入口。"""


def _run_simulation(*args, **kwargs):
    from uav_sim.simulation.runner import run_simulation

    return run_simulation(*args, **kwargs)


def _trace_kwargs(kwargs):
    return {
        "trace_uav_ids": kwargs.get("trace_uav_ids"),
        "trace_log_path": kwargs.get("trace_log_path"),
        "trace_algo_id": kwargs.get("trace_algo_id"),
    }


def _metric_kwargs(kwargs):
    return {
        "enable_kl_divergence": kwargs.get("enable_kl_divergence", False),
        "enable_channel_gain": kwargs.get("enable_channel_gain", False),
    }


def _output_kwargs(kwargs):
    return {
        "run_output_dir": kwargs.get("run_output_dir"),
        "frame_export_enabled": kwargs.get("frame_export_enabled", False),
        "run_id": kwargs.get("run_id"),
        "algorithm_id": kwargs.get("trace_algo_id"),
    }


def run_dijkstra_rule(env, uavs, pickup_points, target_uav_ids, **kwargs):
    """运行纯 Dijkstra + 规则避障。"""
    return _run_simulation(
        env,
        uavs,
        pickup_points,
        mode="DIJKSTRA",
        max_turns=kwargs.get("max_turns", 3500),
        track_uav_ids=target_uav_ids,
        **_trace_kwargs(kwargs),
        **_metric_kwargs(kwargs),
        **_output_kwargs(kwargs),
    )


ALGORITHM_REGISTRY = {
    "1": ("纯 Dijkstra + 规则避障", "Pure Dijkstra + Rule Avoidance", run_dijkstra_rule, [])
}
ALGO_DISTRIBUTED_META = {"1": {"mode": "DIJKSTRA"}}
