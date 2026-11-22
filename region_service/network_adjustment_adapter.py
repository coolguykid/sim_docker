"""Adapt regional database snapshots to the airway-capacity controller model.

The route-adjustment provider owns Config and ThroughputModel.  This module
only projects one effective regional topology plus one completed 60-second
statistics window into that provider's Network, invokes Controller.step(),
and converts the mutated model back to provisional topology adjustments.
"""
from __future__ import annotations

from collections import defaultdict, deque
from copy import deepcopy
from dataclasses import asdict, dataclass
from math import cos, radians, sin

from airway.control import Controller
from airway.model import (
    Config,
    Layer,
    Network,
    PairLayout,
    Pipe,
    PipePair,
    Ramp,
    Region,
    Segment,
    SplitNode,
    ThroughputModel,
)


@dataclass
class NetworkDatabaseMapping:
    segment_to_pipe_row: dict[str, dict]
    pipe_to_segment_ids: dict[str, list[str]]
    ramp_to_row: dict[str, dict]


def _point(value):
    """将坐标分量转换为浮点数。

    输入
        value：待转换的三维坐标序列。
    输出
        浮点坐标元组或列表，以所在模块约定为准。
    """
    values = tuple(float(item) for item in value)
    if len(values) != 3:
        raise ValueError("topology point must contain x/y/z")
    return values


def _ordered_rows(rows):
    """对网络记录执行确定性排序供适配使用。

    输入
        rows：待处理的记录集合。
    输出
        排序后的记录。

    原有说明
        Order one main pipe's local segments by directed node continuity.
    """
    remaining = {int(row["pipe_id"]): row for row in rows}
    end_nodes = {int(row["end_node_id"]) for row in rows}
    starts = sorted(
        (row for row in rows if int(row["start_node_id"]) not in end_nodes),
        key=lambda row: int(row["pipe_id"]),
    )
    result = []
    while remaining:
        current = next((row for row in starts if int(row["pipe_id"]) in remaining), None)
        if current is None:
            current = remaining[min(remaining)]
        while current is not None and int(current["pipe_id"]) in remaining:
            result.append(current)
            remaining.pop(int(current["pipe_id"]))
            end_node = int(current["end_node_id"])
            following = sorted(
                (row for row in remaining.values()
                 if int(row["start_node_id"]) == end_node),
                key=lambda row: int(row["pipe_id"]),
            )
            current = following[0] if following else None
    return result


def _statistics_batch(statistics, evaluation):
    """从仿真统计和评估中选择供网络调整使用的批次。

    输入
        statistics：道路统计记录或历史批次。
        evaluation：拥堵评估结果。
    输出
        所选统计批次。

    原有说明
        Select the completed rolling window that caused the adjustment call.
    """
    trigger = max(
        (float(item["time_s"]) for item in evaluation.get("severe_congestion", ())
         if item.get("time_s") is not None),
        default=None,
    )
    latest = matching = None
    for batch in statistics:
        if batch.get("warmup"):
            continue
        latest = batch
        if trigger is not None and abs(float(batch["time"]) - trigger) <= 1e-9:
            matching = batch
    if latest is None:
        raise ValueError("route adjustment requires a completed statistics window")
    return matching if matching is not None else latest


class DatabaseControlNetworkAdapter:
    """Build and diff the provider's mutable control network for one region."""

    def __init__(self):
        # These are deliberately provider-owned defaults, not database values.
        """建立容量控制适配器依赖。

        输入
            无显式参数。
        输出
            None。
        """
        self.config = Config()
        self.throughput_model = ThroughputModel()

    @staticmethod
    def _derived_pairs(pipe_rows):
        """为数据库航段推导容量控制使用的管道配对。

        输入
            pipe_rows：数据库航道记录集合。
        输出
            配对元数据、主段索引和成员映射。

        原有说明
            Temporarily pair adjacent main-pipe IDs inside each layer.
            
            This rule is isolated here so a future main_pipe_pair_id column can
            replace it without changing the controller integration.
        """
        metadata = {}
        for row in pipe_rows:
            main_id = int(row["main_pipe_id"])
            item = metadata.setdefault(main_id, {
                "layer_id": int(row["layer_id"]),
                "types": set(),
            })
            if item["layer_id"] != int(row["layer_id"]):
                raise ValueError("one main_pipe_id occurs in multiple layers")
            item["types"].add(int(row.get("type", 0)))

        by_layer = defaultdict(list)
        for main_id, item in metadata.items():
            by_layer[item["layer_id"]].append(main_id)

        main_to_pair = {}
        pair_members = {}
        for layer_id, main_ids in by_layer.items():
            ordered = sorted(main_ids)
            for index in range(0, len(ordered), 2):
                members = ordered[index:index + 2]
                # The provider model currently requires exactly two members.
                # A final unpaired main pipe is represented as a singleton
                # group with a repeated ID; state changes are de-duplicated.
                if len(members) == 1:
                    members.append(members[0])
                pair_id = "derived:L{}:G{}".format(layer_id, index // 2 + 1)
                pair_members[pair_id] = tuple(members)
                for main_id in set(members):
                    main_to_pair[main_id] = pair_id
        return metadata, main_to_pair, pair_members

    @staticmethod
    def _build_split_tree(network, ordered_pair_ids, initial_pair_ids):
        """按有序配对构造层内切分树。

        输入
            network：区域网络及设施记录。
            ordered_pair_ids：按空间顺序排列的航道配对编号。
            initial_pair_ids：作为切分起点的配对编号。
        输出
            切分树节点。
        """
        offsets = {pair_id: network.pair_offset(pair_id)
                   for pair_id in ordered_pair_ids}

        def split(left, right, parent=None, depth=0):
            """递归划分配对区间并登记父子结构。

            输入
                left：当前切分区间的左端。
                right：当前切分区间的右端。
                parent：父切分节点。
                depth：当前递归深度。
            输出
                None，通过副作用构造树。
            """
            available = [pair_id for pair_id in ordered_pair_ids
                         if offsets[left] < offsets[pair_id] < offsets[right]]
            if not available:
                return
            midpoint = (offsets[left] + offsets[right]) / 2.0
            pair_id = min(
                available,
                key=lambda value: (round(abs(offsets[value] - midpoint), 8), value),
            )
            network.split_nodes[pair_id] = SplitNode(
                pair_id, left, right, parent, depth)
            split(left, pair_id, pair_id, depth + 1)
            split(pair_id, right, pair_id, depth + 1)

        ordered_initial = sorted(initial_pair_ids, key=lambda value: offsets[value])
        for left, right in zip(ordered_initial, ordered_initial[1:]):
            split(left, right)

    @staticmethod
    def _reachable_pipe(rows_by_start, rows_by_end, ramps_by_start, ramps_by_end,
                        node_id, forward):
        """沿节点和坡道连接查找可达航段。

        输入
            rows_by_start：按起点索引的航道记录。
            rows_by_end：按终点索引的航道记录。
            ramps_by_start：按起点索引的坡道记录。
            ramps_by_end：按终点索引的坡道记录。
            node_id：起始节点编号。
            forward：为真则沿出边查找，为假则沿入边查找。
        输出
            按编号排序的候选航段。
        """
        queue, visited, candidates = deque([int(node_id)]), set(), set()
        while queue:
            node = queue.popleft()
            if node in visited:
                continue
            visited.add(node)
            pipes = rows_by_start.get(node, ()) if forward else rows_by_end.get(node, ())
            candidates.update(int(row["pipe_id"]) for row in pipes)
            if candidates:
                continue
            ramps = ramps_by_start.get(node, ()) if forward else ramps_by_end.get(node, ())
            for ramp in ramps:
                queue.append(int(ramp["end_node_id"] if forward else ramp["start_node_id"]))
        return sorted(candidates)

    def build_network(self, effective_snapshot):
        """将当前数据库有效拓扑转换为容量控制网络。

        输入
            effective_snapshot：当前尝试使用的有效快照。
        输出
            网络以及数据库映射。
        """
        region_id = int(effective_snapshot["region_id"])
        version_id = int(effective_snapshot["topology_version"])
        source = effective_snapshot["network"]
        node_rows = [dict(row) for row in source["nodes"]]
        pipe_rows = [dict(row) for row in source["pipes"]]
        ramp_rows = [dict(row) for row in source["ramps"]]
        nodes = {int(row["node_id"]): row for row in node_rows}

        network = Network(self.config, self.throughput_model)
        for row in source.get("layers", ()):
            layer_id = int(row["layer_id"])
            network.layers[layer_id] = Layer(
                layer_id, float(row.get("angle", 0.0)), float(row.get("height", 0.0)))
        if not network.layers:
            for row in pipe_rows:
                layer_id = int(row["layer_id"])
                network.layers.setdefault(
                    layer_id,
                    Layer(layer_id, float(row.get("angle", 0.0)), 0.0),
                )

        region_key = str(region_id)
        network.regions[region_key] = Region(region_key, 0, 0)
        metadata, main_to_pair, pair_members = self._derived_pairs(pipe_rows)

        rows_by_main = defaultdict(list)
        for row in pipe_rows:
            rows_by_main[int(row["main_pipe_id"])].append(row)

        segment_to_row = {}
        pipe_to_segments = {}
        pipe_id_to_segment = {}
        number_by_layer = defaultdict(int)
        for main_id in sorted(rows_by_main, key=lambda value: (
                metadata[value]["layer_id"], value)):
            ordered = _ordered_rows(rows_by_main[main_id])
            layer_id = metadata[main_id]["layer_id"]
            number_by_layer[layer_id] += 1
            algorithm_pipe_id = "main:{}".format(main_id)
            segment_ids = []
            for row in ordered:
                database_pipe_id = int(row["pipe_id"])
                segment_id = "segment:{}:{}:{}".format(
                    region_id, version_id, database_pipe_id)
                start = _point(nodes[int(row["start_node_id"])]["position"])
                end = _point(nodes[int(row["end_node_id"])]["position"])
                network.segments[segment_id] = Segment(
                    segment_id, algorithm_pipe_id, region_key, start, end,
                    float(row["length"]),
                )
                network.regions[region_key].segment_ids.append(segment_id)
                network.incoming[segment_id] = []
                segment_ids.append(segment_id)
                segment_to_row[segment_id] = row
                pipe_id_to_segment[database_pipe_id] = segment_id

            speeds = [float(row.get("speed_limit", 15.0)) for row in ordered]
            states = [bool(row.get("is_open", True)) for row in ordered]
            network.pipes[algorithm_pipe_id] = Pipe(
                algorithm_pipe_id,
                layer_id,
                number_by_layer[layer_id],
                main_to_pair[main_id],
                1,
                network.segments[segment_ids[0]].start,
                network.segments[segment_ids[-1]].end,
                min(speeds),
                all(states),
                segment_ids,
            )
            pipe_to_segments[algorithm_pipe_id] = segment_ids

        for pair_id, members in pair_members.items():
            network.pairs[pair_id] = PipePair(
                pair_id, tuple("main:{}".format(value) for value in members))

        # Build one dynamic layout for each local layer.  Currently-open main
        # groups are the anchors; backup groups between anchors become split
        # candidates.  Other open groups remain manual references.
        for layer_id in sorted(network.layers):
            pair_ids = [pair_id for pair_id, pair in network.pairs.items()
                        if network.pipes[pair.pipe_ids[0]].layer_id == layer_id]
            pair_ids.sort(key=network.pair_offset)
            initial = []
            for pair_id in pair_ids:
                pair = network.pairs[pair_id]
                member_main_ids = {int(pipe_id.split(":", 1)[1])
                                   for pipe_id in pair.pipe_ids}
                is_primary = any(0 in metadata[value]["types"]
                                 for value in member_main_ids)
                is_open = network.pipes[pair.pipe_ids[0]].is_open
                if is_primary and is_open:
                    initial.append(pair_id)
            self._build_split_tree(network, pair_ids, initial)
            interior = [pair_id for pair_id in pair_ids
                        if pair_id in network.split_nodes]
            open_pairs = {pair_id for pair_id in pair_ids
                          if network.pipes[network.pairs[pair_id].pipe_ids[0]].is_open}
            selected = list(initial) + [pair_id for pair_id in interior
                                        if pair_id in open_pairs and pair_id not in initial]
            layout_id = "{}/L{}".format(region_key, layer_id)
            network.layouts[layout_id] = PairLayout(
                layout_id, region_key, layer_id, 0.0, 0.0, pair_ids,
                list(initial), interior, len(set(selected) - set(initial)),
                sorted(set(selected), key=network.pair_offset),
            )
            network.manual_pair_ids.extend(
                pair_id for pair_id in pair_ids
                if pair_id in open_pairs and pair_id not in selected
                and pair_id not in network.manual_pair_ids)

        rows_by_start, rows_by_end = defaultdict(list), defaultdict(list)
        ramps_by_start, ramps_by_end = defaultdict(list), defaultdict(list)
        for row in pipe_rows:
            rows_by_start[int(row["start_node_id"])].append(row)
            rows_by_end[int(row["end_node_id"])].append(row)
        for row in ramp_rows:
            ramps_by_start[int(row["start_node_id"])].append(row)
            ramps_by_end[int(row["end_node_id"])].append(row)

        ramp_to_row = {}
        for row in ramp_rows:
            # Funnel guideways are required for entering/leaving the airway
            # network.  Closing one can make funnel merge/diverge operations
            # fail, so they are deliberately outside the adjustment network.
            if int(row.get("type", -1)) == 2:
                continue
            source_ids = self._reachable_pipe(
                rows_by_start, rows_by_end, ramps_by_start, ramps_by_end,
                row["start_node_id"], False)
            target_ids = self._reachable_pipe(
                rows_by_start, rows_by_end, ramps_by_start, ramps_by_end,
                row["end_node_id"], True)
            if not source_ids or not target_ids:
                # A connector with no pipe on one side cannot affect a pipe
                # bottleneck in this controller projection.
                continue
            source_segment = pipe_id_to_segment[source_ids[0]]
            target_segment = pipe_id_to_segment[target_ids[0]]
            ramp_id = int(row["ramp_id"])
            algorithm_ramp_id = "ramp:{}:{}:{}".format(
                region_id, version_id, ramp_id)
            path = row.get("ramp_path") or (
                nodes[int(row["start_node_id"])]["position"],
                nodes[int(row["end_node_id"])]["position"],
            )
            network.ramps[algorithm_ramp_id] = Ramp(
                algorithm_ramp_id,
                source_segment,
                target_segment,
                {0: "reverse", 1: "interlayer", 2: "funnel"}.get(
                    int(row.get("type", -1)), "other"),
                _point(path[0]),
                _point(path[-1]),
                bool(row.get("is_open", True)),
                False,
            )
            network.incoming[target_segment].append(algorithm_ramp_id)
            ramp_to_row[algorithm_ramp_id] = row

        network.initialized = True
        return network, NetworkDatabaseMapping(
            segment_to_row, pipe_to_segments, ramp_to_row)

    @staticmethod
    def build_observations(statistics_batch, mapping):
        """将统计批次转换成容量控制的通行与占用观测。

        输入
            statistics_batch：用于容量控制的道路统计批次。
            mapping：容量控制网络与数据库设施的对照表。
        输出
            通行观测和计数。
        """
        by_pipe_id = {
            int(row["edge_id"]): row
            for row in statistics_batch.get("edges", ())
            if row.get("edge_type") == "pipe"
        }
        passed, counts = {}, {}
        for segment_id, pipe_row in mapping.segment_to_pipe_row.items():
            stats = by_pipe_id.get(int(pipe_row["pipe_id"]), {})
            passed[segment_id] = float(stats.get("window_out_count", 0.0))
            counts[segment_id] = int(stats.get("instant_uav_count", 0))
        return passed, counts

    @staticmethod
    def diff(before, after, mapping):
        """比较调整前后的网络并生成数据库更新建议。

        输入
            before：调整前网络。
            after：下界仿真时刻，不含该时刻。
            mapping：容量控制网络与数据库设施的对照表。
        输出
            航道、坡道与控制调整字典。
        """
        pipes, ramps = [], []
        seen_pipe_ids = set()
        for algorithm_pipe_id, after_pipe in after.pipes.items():
            before_pipe = before.pipes[algorithm_pipe_id]
            open_changed = before_pipe.is_open != after_pipe.is_open
            speed_changed = abs(before_pipe.speed_mps - after_pipe.speed_mps) > 1e-9
            if not open_changed and not speed_changed:
                continue
            for segment_id in mapping.pipe_to_segment_ids[algorithm_pipe_id]:
                row = mapping.segment_to_pipe_row[segment_id]
                pipe_id = int(row["pipe_id"])
                if pipe_id in seen_pipe_ids:
                    continue
                seen_pipe_ids.add(pipe_id)
                # Emit a PATCH-like update: a field is present only when the
                # controller changed that field during this step.  In
                # particular, d_safe and max_capacity are not controlled by
                # the current algorithm and must not be reported as changes.
                change = {"pipe_id": pipe_id}
                if open_changed:
                    change["is_open"] = bool(after_pipe.is_open)
                if speed_changed:
                    change["speed_limit"] = float(after_pipe.speed_mps)
                pipes.append(change)

        for algorithm_ramp_id, after_ramp in after.ramps.items():
            before_ramp = before.ramps[algorithm_ramp_id]
            before_open = before_ramp.enabled and not before_ramp.congestion_blocked
            after_open = after_ramp.enabled and not after_ramp.congestion_blocked
            if before_open == after_open:
                continue
            row = mapping.ramp_to_row[algorithm_ramp_id]
            ramps.append({
                "ramp_id": int(row["ramp_id"]),
                "is_open": bool(after_open),
            })
        return {
            "pipes": sorted(pipes, key=lambda row: row["pipe_id"]),
            "ramps": sorted(ramps, key=lambda row: row["ramp_id"]),
        }

    def invoke(self, adjustment_input):
        """构造容量控制输入、执行调整并把结果转回数据库字段。

        输入
            adjustment_input：容量控制输入，包含有效拓扑与仿真统计。
        输出
            数据库调整结果。
        """
        statistics_batch = _statistics_batch(
            adjustment_input["statistics"], adjustment_input["evaluation"])
        network, mapping = self.build_network(
            adjustment_input["effective_snapshot"])
        before = deepcopy(network)
        passed, counts = self.build_observations(statistics_batch, mapping)
        controller = Controller(network)
        events = controller.step(
            float(statistics_batch["time"]), passed, counts=counts)
        result = self.diff(before, network, mapping)
        result["control"] = {
            "time_s": float(statistics_batch["time"]),
            "window_seconds": float(statistics_batch.get("window_seconds", 60.0)),
            "derived_pair_rule": "adjacent_main_pipe_ids_per_layer",
            "events": [asdict(event) for event in events],
        }
        return result


def invoke_capacity_control(adjustment_input):
    """创建适配器并执行航道容量调整。

    输入
        adjustment_input：容量控制输入，包含有效拓扑与仿真统计。
    输出
        数据库调整结果。
    """
    return DatabaseControlNetworkAdapter().invoke(adjustment_input)
