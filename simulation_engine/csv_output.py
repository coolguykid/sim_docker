"""Per-region CSV outputs for one distributed simulation attempt."""
from __future__ import annotations

import csv
from itertools import chain, groupby
from pathlib import Path


def iter_rolling_statistics(path):
    """按时间从完整道路统计 CSV 流式读取，每次只加载一个批次。

    输入
        path：滑动窗口统计 CSV 的路径。
    输出
        按写出顺序产生含时间、窗口长度、预热状态和边统计的批次。
    """
    integer_fields = {
        "edge_id", "lane_count", "instant_uav_count", "window_in_count",
        "window_out_count", "max_capacity",
    }
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        for timestamp, rows in groupby(reader, key=lambda row: row["time"]):
            first = next(rows)
            batch = {
                "time": float(timestamp),
                "window_seconds": float(first["window_seconds"]),
                "warmup": first["warmup"].lower() in ("true", "1"),
                "edges": [],
            }
            for row in chain((first,), rows):
                edge = {"edge_type": row["edge_type"]}
                for name in RegionalCsvOutput.STAT_HEADER[4:]:
                    value = row[name]
                    if value == "":
                        edge[name] = None
                    else:
                        edge[name] = int(value) if name in integer_fields else float(value)
                batch["edges"].append(edge)
            yield batch


class RegionalCsvOutput:
    FRAME_HEADER = (
        "time", "uav_id", "region_id", "topology_version",
        "active", "departed", "arrived", "edge_type", "node_id",
        "pipe_id", "ramp_id", "lane_id", "distance", "edge_length",
        "current_speed", "x", "y", "z", "acceleration",
    )
    STAT_HEADER = (
        "time", "window_seconds", "warmup", "edge_type", "edge_id", "length_m",
        "lane_count", "instant_uav_count", "instant_storage_load_ratio",
        "window_average_uav_count",
        "window_storage_load_ratio",
        "instant_space_mean_speed", "window_space_mean_speed",
        "window_min_spacing_m", "window_max_running_speed", "window_in_count",
        "window_out_count", "in_flow_per_minute", "out_flow_per_minute",
        "instant_density_per_km", "window_density_per_km",
        "window_density_per_km_lane", "speed_limit", "max_capacity",
    )
    FIXED_STAT_HEADER = (
        "time", "period_start", "period_end", "interval_seconds",
        "edge_type", "edge_id", "length_m", "lane_count",
        "average_uav_count", "max_uav_count", "min_uav_count",
        "space_mean_speed", "in_count", "out_count",
        "in_flow_per_minute", "out_flow_per_minute",
        "average_density_per_km", "max_density_per_km",
        "average_density_per_km_lane", "average_storage_load_ratio",
        "max_storage_load_ratio", "speed_limit", "max_capacity",
    )
    FRAME_TIME_HEADER = ("time", "uav_count")

    def __init__(self, output_dir):
        """创建输出目录、CSV 文件流和表头。

        输入
            output_dir：区域输出目录。
        输出
            None。
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.frame_path = self.output_dir / "uav_states.csv"
        # A frame can legitimately contain no UAV rows.  Keep an explicit
        # timeline so live consumers can still clear the previous frame.
        self.frame_time_path = self.output_dir / "frame_times.csv"
        self.statistics_path = self.output_dir / "edge_statistics.csv"
        self.fixed_statistics_path = (
            self.output_dir / "edge_statistics_fixed_60s.csv")
        self._frame_stream = self.frame_path.open("w", encoding="utf-8-sig", newline="")
        self._frame_time_stream = self.frame_time_path.open(
            "w", encoding="utf-8-sig", newline="")
        self._statistics_stream = self.statistics_path.open(
            "w", encoding="utf-8-sig", newline="")
        self._frames = csv.DictWriter(
            self._frame_stream, fieldnames=self.FRAME_HEADER, extrasaction="ignore")
        self._frame_times = csv.DictWriter(
            self._frame_time_stream, fieldnames=self.FRAME_TIME_HEADER)
        self._statistics = csv.DictWriter(
            self._statistics_stream, fieldnames=self.STAT_HEADER, extrasaction="ignore")
        self._fixed_statistics_stream = self.fixed_statistics_path.open(
            "w", encoding="utf-8-sig", newline="")
        self._fixed_statistics = csv.DictWriter(
            self._fixed_statistics_stream,
            fieldnames=self.FIXED_STAT_HEADER,
            extrasaction="ignore")
        self._frames.writeheader()
        self._frame_times.writeheader()
        self._statistics.writeheader()
        self._fixed_statistics.writeheader()

    def write_frame(self, frame):
        """写入一帧的无人机状态和帧索引，并刷新输出流。

        输入
            frame：当前帧的无人机状态记录。
        输出
            None。
        """
        for state in frame:
            position = list(state.get("position") or (None, None, None))
            lifecycle = str(state.get("state") or "")
            edge_type = str(state.get("edge_type") or "")
            edge_id = int(state.get("edge_id", -1))
            row = {
                "time": state["time"],
                "uav_id": state["uav_id"],
                "region_id": state["region_id"],
                "topology_version": state["topology_version"],
                # Keep the lifecycle flags compatible with the C++ uav_states.csv.
                "active": int(lifecycle != "ARRIVED"),
                "departed": int(lifecycle != "WAITING"),
                "arrived": int(lifecycle == "ARRIVED"),
                "edge_type": 0 if edge_type == "pipe" else 1,
                "node_id": state.get("node_id", -1),
                "pipe_id": edge_id if edge_type == "pipe" else -1,
                "ramp_id": edge_id if edge_type == "ramp" else -1,
                "lane_id": state.get("lane_id", -1) if edge_type == "pipe" else -1,
                "distance": state.get("distance_on_edge", -1),
                "edge_length": state.get("edge_length", -1),
                "current_speed": state.get("speed", 0.0),
                "x": position[0],
                "y": position[1],
                "z": position[2],
                "acceleration": state.get("acceleration", 0.0),
            }
            self._frames.writerow(row)
        self._frame_stream.flush()
        # LocalSession always emits one frame per simulation step.  Empty
        # frames have no uav_states.csv row, so record their time separately.
        frame_time = frame[0]["time"] if frame else getattr(self, "_next_frame_time", 0.0)
        self._frame_times.writerow({"time": frame_time, "uav_count": len(frame)})
        self._frame_time_stream.flush()

    def set_frame_time(self, sim_time):
        """设置空帧写入时使用的仿真时刻。

        输入
            sim_time：当前仿真时刻，单位秒。
        输出
            None。

        原有说明
            Supply the timestamp used when the next frame is empty.
        """
        self._next_frame_time = float(sim_time)

    def write_statistics(self, batch):
        """将滑动窗口统计批次写入道路统计 CSV。

        输入
            batch：待写入的一批道路统计。
        输出
            None。
        """
        if batch is None:
            return
        for edge in batch["edges"]:
            self._statistics.writerow({
                "time": batch["time"],
                "window_seconds": batch["window_seconds"],
                "warmup": batch["warmup"],
                **edge,
            })
        self._statistics_stream.flush()

    def write_fixed_statistics(self, batch):
        """将固定周期统计批次写入对应 CSV。

        输入
            batch：待写入的一批道路统计。
        输出
            None。
        """
        if batch is None:
            return
        common = {
            "time": batch["time"],
            "period_start": batch["period_start"],
            "period_end": batch["period_end"],
            "interval_seconds": batch["interval_seconds"],
        }
        for edge in batch["edges"]:
            self._fixed_statistics.writerow({**common, **edge})
        self._fixed_statistics_stream.flush()

    def close(self):
        """关闭当前区域尝试的所有 CSV 文件流。

        输入
            无显式参数。
        输出
            None。
        """
        if not self._frame_stream.closed:
            self._frame_stream.close()
        if not self._frame_time_stream.closed:
            self._frame_time_stream.close()
        if not self._statistics_stream.closed:
            self._statistics_stream.close()
        if not self._fixed_statistics_stream.closed:
            self._fixed_statistics_stream.close()
