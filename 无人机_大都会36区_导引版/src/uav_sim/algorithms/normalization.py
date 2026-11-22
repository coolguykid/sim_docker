"""
normalization.py — 空域尺度无关的特征归一化工具
"""
import math


def _get_attr(env, name, default):
    return float(getattr(env, name, default))


def xy_span(env):
    return max(1.0, _get_attr(env, "xy_span", _get_attr(env, "x_max", 5000.0) - _get_attr(env, "x_min", 0.0)))


def xy_diagonal(env):
    if hasattr(env, "xy_diagonal"):
        return max(1.0, float(env.xy_diagonal))
    width = _get_attr(env, "x_max", 5000.0) - _get_attr(env, "x_min", 0.0)
    height = _get_attr(env, "y_max", 5000.0) - _get_attr(env, "y_min", 0.0)
    return max(1.0, math.hypot(width, height))


def z_span(env):
    if hasattr(env, "z_span"):
        return max(1.0, float(env.z_span))
    return max(1.0, _get_attr(env, "z_max", 600.0) - _get_attr(env, "z_min", 0.0))


def normalize_distance(env, distance):
    return min(3.0, max(0.0, float(distance) / xy_diagonal(env)))


def normalize_height_diff(env, dz):
    return min(1.0, max(0.0, abs(float(dz)) / z_span(env)))


def normalize_edge_time(_env, edge_time):
    return min(3.0, max(0.0, float(edge_time) / 100.0))


def normalize_pipe_traffic(env, count, pipe_id=None, window_turns=100):
    if pipe_id is not None and hasattr(env, "pipelines") and 0 <= pipe_id < len(env.pipelines):
        pipe = env.pipelines[pipe_id]
        expected = max(10.0, float(getattr(pipe, "length", 1000.0)) / 100.0)
    else:
        expected = max(10.0, float(window_turns) * 0.5)
    return min(1.0, max(0.0, float(count) / expected))


def normalize_congestion_variance(env, variance):
    pipe_count = max(1, len(getattr(env, "pipelines", [])))
    # A loose scale that grows with topology size; keeps old 60-pipe behavior close.
    expected = max(1.0, 2500.0 * (pipe_count / 60.0))
    return min(1.0, max(0.0, float(variance) / expected))


def airspace_scale_norm(env):
    side = xy_span(env)
    if side <= 5000.0:
        return 0.0
    return min(1.0, math.log(side / 5000.0) / math.log(30000.0 / 5000.0))
