"""Apply provisional changes in memory and commit accepted changes to status tables."""
from __future__ import annotations

from copy import deepcopy

from repository import commit_to_database as commit_to_database


def merge_adjustments(current, new_values):
    """按设施编号合并多轮网络调整。

    输入
        current：已累积的当前调整。
        new_values：本轮新增调整。
    输出
        合并后的调整字典。
    """
    merged = deepcopy(current or {"pipes": [], "ramps": []})
    for kind, identity in (("pipes", "pipe_id"), ("ramps", "ramp_id")):
        indexed = {int(item[identity]): dict(item) for item in merged.get(kind, ())}
        for item in (new_values or {}).get(kind, ()):
            key = int(item[identity])
            indexed[key] = {**indexed.get(key, {}), **dict(item)}
        merged[kind] = list(indexed.values())
    return merged


def apply_to_snapshot(original_snapshot, adjustments):
    """将网络调整应用到快照副本。

    输入
        original_snapshot：应用网络调整前的原始快照。
        adjustments：待应用或持久化的设施调整。
    输出
        调整后的有效快照。
    """
    snapshot = deepcopy(original_snapshot)
    network = snapshot["network"]
    pipe_changes = {int(item["pipe_id"]): item
                    for item in (adjustments or {}).get("pipes", ())}
    ramp_changes = {int(item["ramp_id"]): item
                    for item in (adjustments or {}).get("ramps", ())}
    for pipe in network["pipes"]:
        change = pipe_changes.get(int(pipe["pipe_id"]))
        if change:
            for name in ("is_open", "speed_limit", "d_safe", "max_capacity"):
                if name in change:
                    pipe[name] = change[name]
    for ramp in network["ramps"]:
        change = ramp_changes.get(int(ramp["ramp_id"]))
        if change and "is_open" in change:
            ramp["is_open"] = change["is_open"]
    return snapshot
