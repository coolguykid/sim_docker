from __future__ import annotations

from repository import load_snapshot as load_snapshot


def snapshot_summary(snapshot: dict) -> dict:
    """为数据库任务记录汇总快照标识和规模。

    输入
        snapshot：区域数据库快照。
    输出
        摘要字典。
    """
    network = snapshot["network"]
    return {
        "snapshot_time": str(snapshot["snapshot_time"]),
        "topology_version": snapshot["topology_version"],
        "nodes": len(network["nodes"]),
        "pipes": len(network["pipes"]),
        "ramps": len(network["ramps"]),
        "lanes": len(network["lanes"]),
        "uavs": len(snapshot["uavs"]),
    }
