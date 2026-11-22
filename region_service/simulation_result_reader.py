"""Read regional DT CSV artifacts as indexed, frame-sized frontend snapshots."""
from __future__ import annotations

import bisect
import csv
import io
import os
import re
from pathlib import Path
from threading import RLock

from repository import REGION_ID


OUTPUT_ROOT = Path(os.environ.get("REGION_OUTPUT_DIR", "/app/data")).resolve()
ATTEMPT_PATTERN = re.compile(r"attempt_(\d+)$")
_lock = RLock()
_indexes: dict[Path, "CsvTimeIndex"] = {}


class CsvTimeIndex:
    """Small time-to-byte-range index that also follows a growing CSV file."""

    def __init__(self, path: Path):
        """为一个按仿真时间组织的 CSV 建立读索引。

        输入
            path：CSV 文件路径。
        输出
            None。
        """
        self.path = path
        self.size = -1
        self.scan_offset = 0
        self.header: list[str] = []
        self.ranges: dict[float, list[int]] = {}
        self.times: list[float] = []

    def refresh(self) -> None:
        """增量刷新 CSV 的时间索引。

        输入
            无显式参数。
        输出
            None。
        """
        size = self.path.stat().st_size
        if size < self.size:
            self.scan_offset = 0
            self.header = []
            self.ranges.clear()
            self.times.clear()
        if size == self.size:
            return
        with self.path.open("rb") as stream:
            if not self.header:
                raw_header = stream.readline().decode("utf-8-sig").rstrip("\r\n")
                self.header = next(csv.reader([raw_header]))
                self.scan_offset = stream.tell()
            stream.seek(self.scan_offset)
            while True:
                start = stream.tell()
                line = stream.readline()
                if not line:
                    break
                try:
                    sim_time = float(line.split(b",", 1)[0])
                except ValueError:
                    continue
                end = stream.tell()
                if sim_time not in self.ranges:
                    bisect.insort(self.times, sim_time)
                    self.ranges[sim_time] = [start, end]
                else:
                    self.ranges[sim_time][1] = end
            self.scan_offset = stream.tell()
        self.size = size

    def latest_time(self) -> float | None:
        """读取当前 CSV 已索引的最新帧时刻。

        输入
            无显式参数。
        输出
            仿真秒数或 None。
        """
        self.refresh()
        return self.times[-1] if self.times else None

    def rows_at_or_before(self, requested: float) -> tuple[float | None, list[dict]]:
        """找出不晚于请求时刻的最近批次。

        输入
            requested：请求查询的仿真秒数。
        输出
            匹配时刻和对应记录列表。
        """
        self.refresh()
        index = bisect.bisect_right(self.times, float(requested)) - 1
        if index < 0:
            return None, []
        sim_time = self.times[index]
        start, end = self.ranges[sim_time]
        with self.path.open("rb") as stream:
            stream.seek(start)
            text = stream.read(end - start).decode("utf-8")
        return sim_time, list(csv.DictReader(io.StringIO(text), fieldnames=self.header))

    def rows_exact(self, requested: float) -> list[dict]:
        """读取指定时刻的全部记录。

        输入
            requested：请求查询的仿真秒数。
        输出
            匹配的 CSV 记录列表。

        原有说明
            Return rows for exactly one known timestamp, or an empty frame.
        """
        self.refresh()
        sim_time = float(requested)
        byte_range = self.ranges.get(sim_time)
        if byte_range is None:
            return []
        start, end = byte_range
        with self.path.open("rb") as stream:
            stream.seek(start)
            text = stream.read(end - start).decode("utf-8")
        return list(csv.DictReader(io.StringIO(text), fieldnames=self.header))


def _index(path: Path) -> CsvTimeIndex:
    """取得指定路径对应的缓存 CSV 时间索引。

    输入
        path：CSV 文件路径。
    输出
        CsvTimeIndex。
    """
    with _lock:
        value = _indexes.get(path)
        if value is None:
            value = CsvTimeIndex(path)
            _indexes[path] = value
        return value


def _attempts():
    """枚举区域输出目录中存在状态 CSV 的仿真尝试。

    输入
        无显式参数。
    输出
        包含修改时间、运行 ID、尝试编号及目录的列表。
    """
    if not OUTPUT_ROOT.exists():
        return []
    result = []
    for run_dir in OUTPUT_ROOT.iterdir():
        if not run_dir.is_dir():
            continue
        for attempt_dir in run_dir.iterdir():
            match = ATTEMPT_PATTERN.fullmatch(attempt_dir.name)
            frame_path = attempt_dir / "uav_states.csv"
            if match and frame_path.is_file():
                result.append((frame_path.stat().st_mtime, run_dir.name,
                               int(match.group(1)), attempt_dir))
    return result


def latest_result() -> dict:
    """从输出目录解析最新结果元数据。

    输入
        无显式参数。
    输出
        最新尝试摘要。
    """
    attempts = _attempts()
    if not attempts:
        raise LookupError("no regional simulation CSV result is available")
    _mtime, run_id, attempt_id, directory = max(attempts)
    timeline_path = directory / "frame_times.csv"
    frame_index = _index(timeline_path if timeline_path.is_file()
                         else directory / "uav_states.csv")
    max_time = frame_index.latest_time() or 0.0
    return {
        "schemaVersion": "regional-simulation-result-1.0",
        "regionId": REGION_ID,
        "runId": run_id,
        "attemptId": attempt_id,
        "minTime": frame_index.times[0] if frame_index.times else 0.0,
        "maxTime": max_time,
        "hasStatistics": (directory / "edge_statistics.csv").is_file(),
    }


def _safe_attempt(run_id: str, attempt_id: int) -> Path:
    """解析并校验指定尝试的区域输出目录。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        区域输出目录 Path。
    """
    if not run_id or Path(run_id).name != run_id or run_id in (".", ".."):
        raise ValueError("invalid run id")
    directory = (OUTPUT_ROOT / run_id / "attempt_{:03d}".format(int(attempt_id))).resolve()
    if OUTPUT_ROOT not in directory.parents:
        raise ValueError("simulation result path escapes output root")
    if not directory.is_dir():
        raise LookupError("simulation result does not exist")
    return directory


def _number(row, name, integer=False):
    """从 CSV 字段读取并转换数值。

    输入
        row：待投影、读取或转换的一条记录。
        name：要读取的 CSV 字段名。
        integer：是否转换为整数。
    输出
        整数、浮点数或 None。
    """
    value = row.get(name)
    if value in (None, ""):
        return None
    return int(float(value)) if integer else float(value)


def load_snapshot(run_id: str, attempt_id: int, requested_time: float) -> dict:
    """读取指定尝试在给定时刻的无人机状态及道路统计。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        requested_time：请求查询的仿真秒数。
    输出
        CSV 支持的仿真快照。
    """
    directory = _safe_attempt(run_id, attempt_id)
    timeline_path = directory / "frame_times.csv"
    if timeline_path.is_file():
        frame_time, _timeline = _index(timeline_path).rows_at_or_before(requested_time)
        raw_uavs = (_index(directory / "uav_states.csv").rows_exact(frame_time)
                    if frame_time is not None else [])
    else:
        frame_time, raw_uavs = _index(
            directory / "uav_states.csv").rows_at_or_before(requested_time)
    if frame_time is None:
        raise LookupError("no UAV frame exists at or before requested time")
    uavs = [{
        "uavId": str(row["uav_id"]),
        "regionId": _number(row, "region_id", True),
        "topologyVersion": _number(row, "topology_version", True),
        "active": bool(_number(row, "active", True)),
        "departed": bool(_number(row, "departed", True)),
        "arrived": bool(_number(row, "arrived", True)),
        "edgeType": "pipe" if _number(row, "edge_type", True) == 0 else "ramp",
        "pipeId": _number(row, "pipe_id", True),
        "rampId": _number(row, "ramp_id", True),
        "laneId": _number(row, "lane_id", True),
        "distance": _number(row, "distance"),
        "edgeLength": _number(row, "edge_length"),
        "speed": _number(row, "current_speed"),
        "position": [_number(row, "x"), _number(row, "y"), _number(row, "z")],
        "acceleration": _number(row, "acceleration"),
    } for row in raw_uavs]

    statistics_time, statistics = None, []
    statistics_path = directory / "edge_statistics.csv"
    if statistics_path.is_file():
        statistics_time, rows = _index(statistics_path).rows_at_or_before(frame_time)
        for row in rows:
            statistics.append({
                "edgeType": row["edge_type"],
                "edgeId": _number(row, "edge_id", True),
                "windowSeconds": _number(row, "window_seconds"),
                "warmup": str(row.get("warmup", "")).lower() in ("true", "1"),
                "instantUavCount": _number(row, "instant_uav_count", True),
                "instantStorageLoadRatio": (
                    _number(row, "instant_storage_load_ratio")
                    if row.get("instant_storage_load_ratio") not in (None, "")
                    else ((_number(row, "instant_uav_count") or 0.0)
                          / _number(row, "max_capacity")
                          if _number(row, "max_capacity") else None)),
                "windowAverageUavCount": _number(row, "window_average_uav_count"),
                "windowStorageLoadRatio": (
                    _number(row, "window_storage_load_ratio")
                    if row.get("window_storage_load_ratio") not in (None, "")
                    else ((_number(row, "window_average_uav_count") or 0.0)
                          / _number(row, "max_capacity")
                          if _number(row, "max_capacity") else None)),
                "instantSpaceMeanSpeed": _number(row, "instant_space_mean_speed"),
                "windowSpaceMeanSpeed": _number(row, "window_space_mean_speed"),
                "windowMinSpacingM": _number(row, "window_min_spacing_m"),
                "windowMaxRunningSpeed": _number(row, "window_max_running_speed"),
                "windowInCount": _number(row, "window_in_count", True),
                "windowOutCount": _number(row, "window_out_count", True),
                "inFlowPerMinute": _number(row, "in_flow_per_minute"),
                "outFlowPerMinute": _number(row, "out_flow_per_minute"),
                "instantDensityPerKm": _number(row, "instant_density_per_km"),
                "windowDensityPerKm": _number(row, "window_density_per_km"),
                "windowDensityPerKmLane": _number(row, "window_density_per_km_lane"),
                "speedLimit": _number(row, "speed_limit"),
                "maxCapacity": _number(row, "max_capacity", True),
            })
    return {
        "schemaVersion": "regional-simulation-snapshot-1.0",
        "regionId": REGION_ID,
        "runId": run_id,
        "attemptId": int(attempt_id),
        "requestedTime": float(requested_time),
        "frameTime": frame_time,
        "statisticsTime": statistics_time,
        "uavs": uavs,
        "statistics": statistics,
    }


def frame_times_after(run_id: str, attempt_id: int, after: float | None) -> list[float]:
    """列出指定时刻之后可用的仿真帧。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        after：下界仿真时刻，不含该时刻。
    输出
        升序仿真时刻列表。

    原有说明
        Return newly completed CSV frame times in ascending order.
    """
    directory = _safe_attempt(run_id, attempt_id)
    timeline_path = directory / "frame_times.csv"
    index = _index(timeline_path if timeline_path.is_file()
                   else directory / "uav_states.csv")
    index.refresh()
    if after is None:
        return list(index.times)
    position = bisect.bisect_right(index.times, float(after) + 1e-9)
    return index.times[position:]
