"""Read-only database topology projection for visualization clients.

This module deliberately does not load UAVs and does not start a simulation.
It converts the regional PostgreSQL schema into a stable JSON-friendly DTO.
"""

from __future__ import annotations

from datetime import datetime, timezone

from repository import REGION_ID, fetch_topology_rows


def attempt_topology(snapshot):
    at = snapshot["snapshot_time"]
    region, version, layers, *_ = fetch_topology_rows(snapshot["topology_version"], at)
    network = snapshot["network"]
    return project_topology(
        region, version, layers, network["nodes"], network["pipes"],
        network["ramps"], network["lanes"], at,
    )


def _point_path(values) -> list[list[float]]:
    """将数据库路径坐标转换为三维点列表。

    输入
        values：组成路径的数据库坐标值。
    输出
        三维坐标列表。
    """
    raw = list(values or ())
    # PostgreSQL REAL[] keeps its two-dimensional shape in psycopg.  The
    # current database stores ramp_path as {{x,y,z},{x,y,z}}, while older
    # exports may expose a flattened array; accept both representations.
    if raw and isinstance(raw[0], (list, tuple)):
        if any(len(point) != 3 for point in raw):
            raise ValueError("ramp_path points must contain x/y/z")
        return [list(map(float, point)) for point in raw]
    if len(raw) % 3:
        raise ValueError("ramp_path must contain complete x/y/z triples")
    return [list(map(float, raw[index:index + 3])) for index in range(0, len(raw), 3)]


def load_topology(version_id: int | None = None, state_time: datetime | None = None) -> dict:
    """按版本和状态时间投影数据库中的区域拓扑。

    输入
        version_id：数据库拓扑版本号。
        state_time：设施状态查询时间。
    输出
        拓扑 DTO。

    原有说明
        Return one regional topology version and latest facility states.
        
        The result is intended for rendering only.  Simulation snapshot loading
        remains in snapshot_loader.py so a GET request cannot trigger DT work.
    """
    at = state_time or datetime.now(timezone.utc)
    rows = fetch_topology_rows(version_id, at)
    region, version, layers, nodes, pipes, ramps, lanes = rows
    return project_topology(region, version, layers, nodes, pipes, ramps, lanes, at)


def project_topology(region, version, layers, nodes, pipes, ramps, lanes, at):
    selected_version = int(version["version_id"])

    return {
        "schemaVersion": "regional-topology-1.0",
        "regionId": str(REGION_ID),
        "topologyVersion": str(selected_version),
        "validFrom": version["valid_from"],
        "validTo": version["valid_to"],
        "stateTime": at,
        "coordinate": {
            "coordinateSystem": "local-ENU",
            "axisOrder": ["east", "north", "up"],
            "unit": "m",
        },
        "range": list(region["region_range"]),
        "neighbourhood": [int(value) for value in region["neighbourhood"]],
        "layers": [
            {
                "layerId": str(row["layer_id"]),
                "angle": float(row["angle"]),
                "height": float(row["height"]),
                "pipeNumber": int(row["pipe_number"]),
            }
            for row in layers
        ],
        "nodes": [
            {
                "nodeId": str(row["node_id"]),
                "layerId": None if row["layer_id"] is None else str(row["layer_id"]),
                "position": list(map(float, row["position"])),
                "regionIds": [str(value) for value in row["region_ids"]],
                "nodeType": int(row["node_type"]),
                "boundaryType": int(row["boundary_type"]),
                "isPortal": len(row["region_ids"]) > 1,
            }
            for row in nodes
        ],
        "pipes": [
            {
                "pipeId": str(row["pipe_id"]),
                "mainPipeId": str(row["main_pipe_id"]),
                "layerId": str(row["layer_id"]),
                "type": "reserve" if int(row["type"]) == 1 else "main",
                "angle": float(row["angle"]),
                "length": float(row["length"]),
                "laneNumber": int(row["lane_number"]),
                "pipeWidth": float(row["pipe_width"]),
                "laneWidth": float(row["lane_width"]),
                "startNodeId": str(row["start_node_id"]),
                "endNodeId": str(row["end_node_id"]),
                "state": {
                    "isOpen": bool(row["is_open"]),
                    "speedLimit": float(row["speed_limit"]),
                    "safeDistance": float(row["d_safe"]),
                    "maxCapacity": int(row["max_capacity"]),
                    "sampleTime": row.get("status_time"),
                },
            }
            for row in pipes
        ],
        "ramps": [
            {
                "rampId": str(row["ramp_id"]),
                "type": {0: "same-layer-reverse", 1: "inter-layer", 2: "funnel"}[
                    int(row["type"])
                ],
                "length": float(row["length"]),
                "rampWidth": None if row["ramp_width"] is None else float(row["ramp_width"]),
                "laneNumber": int(row["lane_number"]),
                "laneWidth": float(row["lane_width"]),
                "startNodeId": str(row["start_node_id"]),
                "endNodeId": str(row["end_node_id"]),
                "path": _point_path(row["ramp_path"]),
                "state": {
                    "isOpen": bool(row["is_open"]),
                    "sampleTime": row.get("status_time"),
                },
            }
            for row in ramps
        ],
        "lanes": [
            {
                "laneId": str(row["lane_id"]),
                "layerId": str(row["layer_id"]),
                "pipeId": str(row["pipe_id"]),
                "length": float(row["length"]),
                "startPoint": list(map(float, row["start_pt"])),
                "endPoint": list(map(float, row["end_pt"])),
            }
            for row in lanes
        ],
    }
