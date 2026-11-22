"""Sliding-window edge statistics owned by one regional simulation session."""
from __future__ import annotations

from collections import defaultdict, deque


class EdgeStatisticsTracker:
    """Sliding-window statistics emitted at a shorter update interval."""

    def __init__(self, env, window_seconds=60.0, update_interval=5.0):
        """初始化滑动窗口、输出节奏和边统计索引。

        输入
            env：可按边引用读取属性的统计环境。
            window_seconds：滑动统计窗口长度，单位秒。
            update_interval：滑动统计输出间隔，单位秒。
        输出
            None。
        """
        self.env = env
        self.window_seconds = float(window_seconds)
        self.update_interval = float(update_interval)
        self.samples = deque()
        self.transitions = deque()
        self.previous_edges = {}
        self.next_output_time = self.update_interval
        self.last_observation = None

    @staticmethod
    def _key(edge_ref):
        """将道路边引用转成用于聚合的类型和编号。

        输入
            edge_ref：数据库边引用及设施类型。
        输出
            边类型和编号组成的键。
        """
        return "{}:{}".format(edge_ref.edge_type, edge_ref.edge_id)

    def observe(self, sim_time, uavs, explicit_in=None, explicit_out=None):
        """采样当前飞机状态与进出边事件，并按配置节奏生成窗口统计。

        输入
            sim_time：当前仿真时刻，单位秒。
            uavs：当前区域的无人机集合。
            explicit_in：已显式确认的入边计数事件。
            explicit_out：已显式确认的出边计数事件。
        输出
            统计批次或 None。
        """
        counts, speed_sums = defaultdict(int), defaultdict(float)
        lane_positions = defaultdict(list)
        maximum_speeds = {}
        current_edges = {}
        for uav in uavs:
            if uav.state not in ("FLYING", "WAITING"):
                continue
            key = self._key(uav.current_edge_ref)
            speed = float(uav.speed)
            counts[key] += 1
            speed_sums[key] += speed
            maximum_speeds[key] = max(maximum_speeds.get(key, speed), speed)
            lane_positions[(key, int(uav.current_lane_id))].append(float(uav.pos_1d))
            current_edges[int(uav.uav_id)] = key

        minimum_spacings = {}
        for (key, _lane_id), positions in lane_positions.items():
            if len(positions) < 2:
                continue
            positions.sort()
            spacing = min(
                positions[index] - positions[index - 1]
                for index in range(1, len(positions)))
            minimum_spacings[key] = min(
                minimum_spacings.get(key, spacing), spacing)

        entries, exits = defaultdict(int), defaultdict(int)
        for uav_id, key in current_edges.items():
            previous = self.previous_edges.get(uav_id)
            if previous is not None and previous != key:
                exits[previous] += 1
                entries[key] += 1
        for key in set(self.previous_edges) - set(current_edges):
            exits[self.previous_edges[key]] += 1
        explicit_entries, explicit_exits = defaultdict(int), defaultdict(int)
        for edge_ref in explicit_in or ():
            explicit_entries[self._key(edge_ref)] += 1
        for edge_ref in explicit_out or ():
            explicit_exits[self._key(edge_ref)] += 1
        # Explicit handoff events supplement inferred changes.  max() avoids
        # double-counting outbound UAVs that also disappeared from this region,
        # while preserving multiple handoffs through the same edge in one step.
        for key, value in explicit_entries.items():
            entries[key] = max(entries[key], value)
        for key, value in explicit_exits.items():
            exits[key] = max(exits[key], value)

        self.previous_edges = current_edges
        self.last_observation = {
            "time": float(sim_time),
            "counts": dict(counts),
            "speed_sums": dict(speed_sums),
            "minimum_spacings": dict(minimum_spacings),
            "maximum_speeds": dict(maximum_speeds),
            "entries": dict(entries),
            "exits": dict(exits),
        }
        self.samples.append((
            float(sim_time), dict(counts), dict(speed_sums),
            dict(minimum_spacings), dict(maximum_speeds)))
        self.transitions.append((float(sim_time), dict(entries), dict(exits)))
        cutoff = float(sim_time) - self.window_seconds
        while self.samples and self.samples[0][0] <= cutoff:
            self.samples.popleft()
        while self.transitions and self.transitions[0][0] <= cutoff:
            self.transitions.popleft()

        due = float(sim_time) + 1e-9 >= self.next_output_time
        if not due:
            return None
        while self.next_output_time <= float(sim_time) + 1e-9:
            self.next_output_time += self.update_interval
        return self.snapshot(float(sim_time), counts, speed_sums)

    def snapshot(self, sim_time, instant_counts, instant_speed_sums):
        """从瞬时采样与历史窗口生成道路统计。

        输入
            sim_time：当前仿真时刻，单位秒。
            instant_counts：按道路边统计的当前无人机数。
            instant_speed_sums：按道路边累计的当前无人机速率。
        输出
            道路统计字典。
        """
        observation_count = max(1, len(self.samples))
        window_duration = min(self.window_seconds, max(0.0, sim_time))
        window_duration = max(window_duration, 1.0)
        # Emit every local edge, including zero-traffic edges.  Consumers must
        # distinguish a measured 0% load from a missing statistics record.
        edge_keys = {self._key(ref) for ref in self.env.edges}
        edge_keys.update(instant_counts)
        for _time, counts, _speeds, _spacings, _maximums in self.samples:
            edge_keys.update(counts)
        for _time, entries, exits in self.transitions:
            edge_keys.update(entries)
            edge_keys.update(exits)

        result = []
        for key in sorted(edge_keys):
            edge_type, edge_id_text = key.split(":", 1)
            edge_ref = next(ref for ref in self.env.edges
                            if ref.edge_type == edge_type and ref.edge_id == int(edge_id_text))
            edge = self.env.edge(edge_ref)
            count_sum = sum(
                counts.get(key, 0)
                for _t, counts, _speeds, _spacings, _maximums in self.samples)
            speed_sum = sum(
                speeds.get(key, 0.0)
                for _t, _counts, speeds, _spacings, _maximums in self.samples)
            spacing_values = [
                spacings[key]
                for _t, _counts, _speeds, spacings, _maximums in self.samples
                if key in spacings]
            maximum_speed_values = [
                maximums[key]
                for _t, _counts, _speeds, _spacings, maximums in self.samples
                if key in maximums]
            entries = sum(values.get(key, 0) for _t, values, _o in self.transitions)
            exits = sum(values.get(key, 0) for _t, _i, values in self.transitions)
            average_count = count_sum / observation_count
            capacity = int(getattr(edge, "max_capacity", 50) or 50)
            length_km = max(float(edge.length) / 1000.0, 1e-9)
            instant_count = int(instant_counts.get(key, 0))
            result.append({
                "edge_type": edge_type,
                "edge_id": int(edge_id_text),
                "length_m": float(edge.length),
                "lane_count": int(edge.num_lanes),
                "instant_uav_count": instant_count,
                "instant_storage_load_ratio": (
                    instant_count / capacity if capacity else None),
                "window_average_uav_count": average_count,
                # Rolling occupancy remains available for sustained-congestion
                # evaluation; the live trunk ranking uses the instantaneous
                # ratio above to match the offline display convention.
                "window_storage_load_ratio": (
                    average_count / capacity if capacity else None),
                "instant_space_mean_speed": (
                    instant_speed_sums.get(key, 0.0) / instant_count
                    if instant_count else None),
                "window_space_mean_speed": speed_sum / count_sum if count_sum else None,
                "window_min_spacing_m": (
                    min(spacing_values) if spacing_values else None),
                "window_max_running_speed": (
                    max(maximum_speed_values) if maximum_speed_values else None),
                "window_in_count": entries,
                "window_out_count": exits,
                "in_flow_per_minute": 60.0 * entries / window_duration,
                "out_flow_per_minute": 60.0 * exits / window_duration,
                "instant_density_per_km": instant_count / length_km,
                "window_density_per_km": average_count / length_km,
                "window_density_per_km_lane": (
                    average_count / length_km / max(1, int(edge.num_lanes))),
                "speed_limit": float(edge.max_speed),
                "max_capacity": capacity,
            })
        return {
            "time": sim_time,
            "window_seconds": self.window_seconds,
            "warmup": sim_time < self.window_seconds,
            "edges": result,
        }


class FixedIntervalEdgeStatisticsTracker:
    """Non-overlapping fixed-period statistics built from one-second samples.

    Unlike the sliding tracker, this tracker resets after every completed
    interval.  It consumes the already-collected observation from the sliding
    tracker, so adding fixed reports does not scan all UAVs a second time.
    """

    def __init__(self, env, interval_seconds=60.0):
        """初始化固定周期累计统计。

        输入
            env：可按边引用读取属性的统计环境。
            interval_seconds：固定统计周期，单位秒。
        输出
            None。
        """
        self.env = env
        self.interval_seconds = float(interval_seconds)
        if self.interval_seconds <= 0:
            raise ValueError("fixed statistics interval must be positive")
        self.period_start = 0.0
        self.samples = []
        self.entries = defaultdict(int)
        self.exits = defaultdict(int)

    @staticmethod
    def _key(edge_ref):
        """将道路边引用转成用于聚合的类型和编号。

        输入
            edge_ref：数据库边引用及设施类型。
        输出
            边类型和编号组成的键。
        """
        return "{}:{}".format(edge_ref.edge_type, edge_ref.edge_id)

    def observe(self, observation):
        """累计帧观测，完整周期到达时输出统计。

        输入
            observation：滑动统计器产生的本帧观测。
        输出
            固定周期统计或 None。
        """
        if observation is None:
            return None
        sim_time = float(observation["time"])
        self.samples.append((
            sim_time,
            dict(observation["counts"]),
            dict(observation["speed_sums"]),
        ))
        for key, value in observation["entries"].items():
            self.entries[key] += int(value)
        for key, value in observation["exits"].items():
            self.exits[key] += int(value)

        period_end = self.period_start + self.interval_seconds
        if sim_time + 1e-9 < period_end:
            return None
        result = self._snapshot(period_end)
        self.period_start = period_end
        self.samples.clear()
        self.entries.clear()
        self.exits.clear()
        return result

    def _snapshot(self, period_end):
        """构造指定周期末的固定周期统计。

        输入
            period_end：固定周期的结束时刻。
        输出
            道路统计字典。
        """
        sample_count = len(self.samples)
        if sample_count <= 0:
            return None
        rows = []
        for edge_ref in sorted(
                self.env.edges,
                key=lambda ref: (str(ref.edge_type), int(ref.edge_id))):
            key = self._key(edge_ref)
            edge = self.env.edge(edge_ref)
            counts = [sample_counts.get(key, 0)
                      for _time, sample_counts, _speeds in self.samples]
            count_sum = sum(counts)
            speed_sum = sum(
                speed_sums.get(key, 0.0)
                for _time, _counts, speed_sums in self.samples)
            average_count = count_sum / sample_count
            length_km = max(float(edge.length) / 1000.0, 1e-9)
            lane_count = max(1, int(edge.num_lanes))
            capacity = int(getattr(edge, "max_capacity", 50) or 50)
            rows.append({
                "edge_type": edge_ref.edge_type,
                "edge_id": int(edge_ref.edge_id),
                "length_m": float(edge.length),
                "lane_count": lane_count,
                "average_uav_count": average_count,
                "max_uav_count": max(counts),
                "min_uav_count": min(counts),
                "space_mean_speed": speed_sum / count_sum if count_sum else None,
                "in_count": int(self.entries.get(key, 0)),
                "out_count": int(self.exits.get(key, 0)),
                "in_flow_per_minute": (
                    60.0 * self.entries.get(key, 0) / self.interval_seconds),
                "out_flow_per_minute": (
                    60.0 * self.exits.get(key, 0) / self.interval_seconds),
                "average_density_per_km": average_count / length_km,
                "max_density_per_km": max(counts) / length_km,
                "average_density_per_km_lane": (
                    average_count / length_km / lane_count),
                "average_storage_load_ratio": (
                    average_count / capacity if capacity else None),
                "max_storage_load_ratio": (
                    max(counts) / capacity if capacity else None),
                "speed_limit": float(edge.max_speed),
                "max_capacity": capacity,
            })
        return {
            "time": period_end,
            "period_start": self.period_start,
            "period_end": period_end,
            "interval_seconds": self.interval_seconds,
            "edges": rows,
        }
