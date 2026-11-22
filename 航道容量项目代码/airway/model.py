from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field, fields, is_dataclass
from math import cos, sin, radians, isfinite, sqrt
from typing import Callable

Point = tuple[float, float, float]


@dataclass(frozen=True)
class Config:
    throughput_window_s: float = 60.0
    boundary_m: float = 12000.0
    grid_n: int = 4
    reference_width_m: float = 3000.0
    pipes_per_reference_width: int = 84
    initial_pair_spacing_m: float | None = None
    reverse_ramp_spacing_m: float = 1000.0
    reopen_ratio: float = 0.8
    diameter_m: float = 30.0
    horizontal_clearance_m: float = 5.0
    vertical_clearance_m: float = 100.0
    bottom_m: float = 120.0
    initial_speed_mps: float = 20.0
    min_speed_mps: float = 10.0
    max_speed_mps: float = 50.0
    speed_recovery_ratio: float = 0.6
    speed_recovery_step_mps: float = 2.0

    def __post_init__(self):
        for name in ("throughput_window_s", "boundary_m", "reference_width_m", "diameter_m",
                     "initial_speed_mps", "min_speed_mps", "max_speed_mps", "reverse_ramp_spacing_m",
                     "speed_recovery_step_mps"):
            value = getattr(self, name)
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} 必须是有限正数")
        if self.initial_pair_spacing_m is not None and (not isfinite(self.initial_pair_spacing_m) or self.initial_pair_spacing_m <= 0):
            raise ValueError("initial_pair_spacing_m 必须为有限正数或 None")
        for name in ("grid_n", "pipes_per_reference_width"):
            value = getattr(self, name)
            if type(value) is not int:
                raise ValueError(f"{name} 必须为整数")
        if self.grid_n < 1 or self.pipes_per_reference_width < 2 or self.pipes_per_reference_width % 2:
            raise ValueError("网格数须为正，参考管道数须为正偶数")
        for name in ("horizontal_clearance_m", "vertical_clearance_m", "bottom_m"):
            if not isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} 必须是有限非负数")
        if self.pitch_m < self.diameter_m + self.horizontal_clearance_m:
            raise ValueError("管道密度不满足管径与水平安全净距约束")
        if not 0 < self.reopen_ratio < 1:
            raise ValueError("重开比例须位于 (0, 1)")
        if not 0 < self.speed_recovery_ratio < self.reopen_ratio:
            raise ValueError("调速恢复阈值须大于0且小于匝道恢复阈值")
        if not self.min_speed_mps <= self.initial_speed_mps <= self.max_speed_mps:
            raise ValueError("初始速度超出允许范围")

    @property
    def pair_spacing_m(self) -> float:
        return self.initial_pair_spacing_m if self.initial_pair_spacing_m is not None else self.region_side_m/2

    @property
    def pitch_m(self) -> float:
        return self.reference_width_m / self.pipes_per_reference_width

    @property
    def region_side_m(self) -> float:
        return self.boundary_m / self.grid_n


@dataclass(frozen=True)
class ThroughputModel:
    """安全距离可替换的吞吐模型，默认复现原MATLAB关系。

    spacing_function(v) 接收速度 m/s，返回包含机长在内的完整安全间距 m。
    自定义关系使用全区间数值搜索；有限采样不能保证识别任意窄的峰值。
    """
    base_spacing_m: float = 30.0
    reaction_time_s: float = 1.2
    deceleration_mps2: float = 2.0
    vehicle_length_m: float = 5.0
    braking_gap_factor: float = 0.6
    main_spacing_factor: float = 1.6
    spacing_function: Callable[[float], float] | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        parameters = (self.base_spacing_m, self.reaction_time_s, self.deceleration_mps2,
                      self.vehicle_length_m, self.braking_gap_factor, self.main_spacing_factor)
        if any(not isfinite(v) or v <= 0 for v in parameters):
            raise ValueError("安全间距参数必须为有限正数")
        if self.spacing_function is not None and not callable(self.spacing_function):
            raise ValueError("spacing_function 必须为可调用函数或 None")

    def metadata(self) -> dict:
        """可JSON导出的模型说明；函数本身由调用代码保存，不能从说明中恢复。"""
        result = {item.name: getattr(self, item.name) for item in fields(self)
                  if item.name != "spacing_function"}
        result["model_name"] = ("matlab_main_spacing" if self.spacing_function is None
                                else getattr(self.spacing_function, "__name__",
                                             type(self.spacing_function).__name__))
        return result

    @staticmethod
    def _validate_speed(speed_mps: float):
        try:
            valid = not isinstance(speed_mps, bool) and isfinite(speed_mps) and speed_mps > 0
        except (TypeError, ValueError):
            valid = False
        if not valid:
            raise ValueError("速度必须为有限正数")

    def safety_gap(self, speed_mps: float) -> float:
        """MATLAB中的safeGap：基础、反应和制动安全距离。"""
        self._validate_speed(speed_mps)
        return (self.base_spacing_m + self.reaction_time_s*speed_mps
                + self.braking_gap_factor*speed_mps**2/(2*self.deceleration_mps2))

    def safety_distance(self, speed_mps: float) -> float:
        """速度到完整机间距的主接口，统一供生成、放行、吞吐及调速使用。"""
        self._validate_speed(speed_mps)
        spacing = (self.spacing_function(speed_mps) if self.spacing_function is not None
                   else self.vehicle_length_m + self.main_spacing_factor*self.safety_gap(speed_mps))
        try:
            valid = not isinstance(spacing, bool) and isfinite(spacing) and spacing > 0
        except (TypeError, ValueError):
            valid = False
        if not valid:
            raise ValueError("安全距离接口须返回有限正数（米）")
        return float(spacing)

    def safe_spacing(self, speed_mps: float) -> float:
        """兼容旧调用的别名；新公式请通过 spacing_function 注入。"""
        return self.safety_distance(speed_mps)

    def headway(self, speed_mps: float) -> float:
        interval = self.safety_distance(speed_mps)/speed_mps
        if not isfinite(interval) or interval <= 0:
            raise ValueError("安全时间间隔必须为有限正数")
        return interval

    def throughput(self, speed_mps: float, window_s: float) -> float:
        if not isfinite(window_s) or window_s <= 0:
            raise ValueError("统计窗口必须为有限正数")
        return window_s/self.headway(speed_mps)

    def _validate_speed_bounds(self, lower: float, upper: float):
        self._validate_speed(lower)
        self._validate_speed(upper)
        if lower > upper:
            raise ValueError("速度区间无效")

    def _numerical_candidates(self, lower: float, upper: float) -> list[tuple[float, float]]:
        """全区间2048格采样，分别细化每个采样峰，返回(速度,架/秒)。"""
        if lower == upper:
            return [(lower, self.throughput(lower, 1))]
        points = [lower+(upper-lower)*i/2048 for i in range(2049)]
        rates = [self.throughput(v, 1) for v in points]
        candidates = list(zip(points, rates))
        golden = (sqrt(5)-1)/2
        for i in range(1, len(points)-1):
            if not (rates[i] >= rates[i-1] and rates[i] >= rates[i+1]
                    and (rates[i] > rates[i-1] or rates[i] > rates[i+1])):
                continue
            lo, hi = points[i-1], points[i+1]
            left, right = hi-golden*(hi-lo), lo+golden*(hi-lo)
            f_left, f_right = self.throughput(left, 1), self.throughput(right, 1)
            for _ in range(50):
                if f_left < f_right:
                    lo, left, f_left = left, right, f_right
                    right = lo+golden*(hi-lo)
                    f_right = self.throughput(right, 1)
                else:
                    hi, right, f_right = right, left, f_left
                    left = hi-golden*(hi-lo)
                    f_left = self.throughput(left, 1)
            candidates.extend(((left, f_left), (right, f_right)))
        return candidates

    def best_speed(self, lower: float, upper: float) -> float:
        """默认公式解析求解；自定义关系全区间采样并细化各峰值。"""
        self._validate_speed_bounds(lower, upper)
        if self.spacing_function is not None:
            return max(self._numerical_candidates(lower, upper), key=lambda p: (p[1], p[0]))[0]
        optimum = sqrt(2*self.deceleration_mps2
                       * (self.vehicle_length_m+self.main_spacing_factor*self.base_spacing_m)
                       / (self.main_spacing_factor*self.braking_gap_factor))
        return max(lower, min(upper, optimum))

    def speed_for_demand(self, demand: float, window_s: float, lower: float, upper: float) -> float:
        """返回满足需求的最快速度；不可行则返回吞吐最大速度。"""
        if not isfinite(demand) or demand < 0:
            raise ValueError("通过需求必须为有限非负数")
        self._validate_speed_bounds(lower, upper)
        if self.throughput(upper, window_s) >= demand:
            return upper
        if self.spacing_function is not None:
            candidates = self._numerical_candidates(lower, upper)
            best = max(candidates, key=lambda p: (p[1], p[0]))[0]
            feasible = [p for p in candidates if p[1]*window_s >= demand]
            if not feasible:
                return best
            lo = max(p[0] for p in feasible)
            hi = min(p[0] for p in candidates if p[0] > lo)
            for _ in range(50):
                mid = (lo+hi)/2
                if self.throughput(mid, window_s) >= demand:
                    lo = mid
                else:
                    hi = mid
            return lo
        best = self.best_speed(lower, upper)
        if self.throughput(best, window_s) < demand:
            return best
        lo, hi = best, upper
        for _ in range(50):
            mid = (lo+hi)/2
            if self.throughput(mid, window_s) >= demand:
                lo = mid
            else:
                hi = mid
        return lo


@dataclass
class Layer:
    id: int
    angle_deg: float
    height_m: float


@dataclass
class Pipe:
    id: str
    layer_id: int
    number: int
    pair_id: str
    direction: int
    start: Point
    end: Point
    speed_mps: float
    is_open: bool = False
    segment_ids: list[str] = field(default_factory=list)


@dataclass
class PipePair:
    id: str
    pipe_ids: tuple[str, str]


@dataclass
class SplitNode:
    """一个间隙的中点候选；打开后，其左右间隙成为两个子节点。"""
    pair_id: str
    left_pair_id: str
    right_pair_id: str
    parent_pair_id: str | None
    depth: int


@dataclass
class PairLayout:
    """区域层的初始引用及二叉细分新增引用。"""
    id: str
    region_id: str
    layer_id: int
    center_offset_m: float
    spacing_m: float
    candidate_pair_ids: list[str]
    initial_pair_ids: list[str]
    interior_pair_ids: list[str]
    extra_pairs: int = 0
    selected_pair_ids: list[str] = field(default_factory=list)


@dataclass
class Segment:
    id: str
    pipe_id: str
    region_id: str
    start: Point
    end: Point
    length_m: float
    # 当前时刻实际位于该段的无人机数量，不追踪单机位置。
    count: int = 0
    passed: float = 0.0  # 过去滚动窗口内实际离段架次；外部接口可传归一化观测值
    demand: float = 0.0  # 同长度未来窗口的预测通过需求


@dataclass
class Region:
    id: str
    row: int
    col: int
    segment_ids: list[str] = field(default_factory=list)


@dataclass
class Ramp:
    id: str
    source_segment: str
    target_segment: str
    kind: str
    start: Point
    end: Point
    enabled: bool = True
    congestion_blocked: bool = False


@dataclass
class Network:
    config: Config
    throughput_model: ThroughputModel
    layers: dict[int, Layer] = field(default_factory=dict)
    pipes: dict[str, Pipe] = field(default_factory=dict)
    pairs: dict[str, PipePair] = field(default_factory=dict)
    segments: dict[str, Segment] = field(default_factory=dict)
    regions: dict[str, Region] = field(default_factory=dict)
    ramps: dict[str, Ramp] = field(default_factory=dict)
    # 连续管道的跨区边界接续，不是可关闭匝道。
    continuations: list[tuple[str, str]] = field(default_factory=list)
    incoming: dict[str, list[str]] = field(default_factory=dict)
    layouts: dict[str, PairLayout] = field(default_factory=dict)
    split_nodes: dict[str, SplitNode] = field(default_factory=dict)
    manual_pair_ids: list[str] = field(default_factory=list)
    retained_pair_ids: list[str] = field(default_factory=list)
    initialized: bool = False

    def pair_offset(self, pair_id: str) -> float:
        pair = self.pairs[pair_id]
        pipes = [self.pipes[pid] for pid in pair.pipe_ids]
        theta = radians(self.layers[pipes[0].layer_id].angle_deg)
        return sum(-sin(theta)*p.start[0]+cos(theta)*p.start[1] for p in pipes)/2

    def prepare_layouts(self):
        """建立区域横向位置索引；由构建器调用，不改变开关。"""
        side = self.config.region_side_m
        half = self.config.boundary_m/2
        self.layouts.clear()
        self.split_nodes.clear()
        for region in self.regions.values():
            ids = set(region.segment_ids)
            cx, cy = -half+(region.col+.5)*side, -half+(region.row+.5)*side
            for layer in self.layers.values():
                theta = radians(layer.angle_deg)
                center = -sin(theta)*cx+cos(theta)*cy
                candidates = [p.id for p in self.pairs.values()
                              if self.pipes[p.pipe_ids[0]].layer_id == layer.id
                              and all(f"{pid}@{region.id}" in ids for pid in p.pipe_ids)]
                candidates.sort(key=lambda pid: self.pair_offset(pid))
                lid = f"{region.id}/L{layer.id+1}"
                self.layouts[lid] = PairLayout(lid, region.id, layer.id, center, self.config.pair_spacing_m,
                                                candidates, [], [])
        from .initialization import select_initial_pairs
        for layer in self.layers.values():
            selected = set(select_initial_pairs(self, layer.id))
            ordered = sorted((p.id for p in self.pairs.values()
                              if self.pipes[p.pipe_ids[0]].layer_id == layer.id), key=self.pair_offset)
            offsets = {pid: self.pair_offset(pid) for pid in ordered}
            def split(left, right, parent=None, depth=0):
                available = [pid for pid in ordered if offsets[left] < offsets[pid] < offsets[right]]
                if not available:
                    return
                midpoint = (offsets[left]+offsets[right])/2
                pid = min(available, key=lambda p: (round(abs(offsets[p]-midpoint), 8), p))
                self.split_nodes[pid] = SplitNode(pid, left, right, parent, depth)
                split(left, pid, pid, depth+1)
                split(pid, right, pid, depth+1)
            initial = sorted(selected, key=self.pair_offset)
            for left, right in zip(initial, initial[1:]):
                split(left, right)
            for layout in self.layouts.values():
                if layout.layer_id != layer.id:
                    continue
                # 初始引用包含任何一根穿过本格的完整全域管道对。
                layout.initial_pair_ids = sorted(
                    (pid for pid in selected if any(
                        f"{pipe_id}@{layout.region_id}" in self.segments
                        for pipe_id in self.pairs[pid].pipe_ids)), key=self.pair_offset)
                layout.interior_pair_ids = [pid for pid in layout.candidate_pair_ids
                                           if pid in self.split_nodes]

    def _with_ancestors(self, pairs: set[str]) -> set[str]:
        result = set(pairs)
        for pid in pairs:
            node = self.split_nodes.get(pid)
            while node and node.parent_pair_id:
                result.add(node.parent_pair_id)
                node = self.split_nodes[node.parent_pair_id]
        return result

    def expansion_candidates(self, layout_id: str, opened: set[str] | None = None) -> list[str]:
        """两根边界已开放、两条候选管道均穿过本区的叶间隙，浅层优先。"""
        if opened is None:
            opened = {p.id for p in self.pairs.values() if self.pipes[p.pipe_ids[0]].is_open}
        return sorted((pid for pid in self.layouts[layout_id].interior_pair_ids
                       if pid not in opened and self.split_nodes[pid].left_pair_id in opened
                       and self.split_nodes[pid].right_pair_id in opened),
                      key=lambda pid: (self.split_nodes[pid].depth, pid))

    def has_open_children(self, pair_id: str, opened: set[str] | None = None) -> bool:
        if opened is None:
            opened = {p.id for p in self.pairs.values() if self.pipes[p.pipe_ids[0]].is_open}
        return any(n.parent_pair_id == pair_id and pid in opened for pid, n in self.split_nodes.items())

    def layout_pair_ids(self, layout_id: str, extra_pairs: int = 0) -> list[str]:
        """从当前引用增减到k对；新增浅层优先，撤回深层优先，不移动已有位置。

        这是低层引用规划接口；无人机、树依赖及跨区通过能力由应用/控制器保护。
        """
        layout = self.layouts[layout_id]
        if type(extra_pairs) is not int or not 0 <= extra_pairs <= len(layout.interior_pair_ids):
            raise ValueError("新增对数k超出该区域层的可用范围")
        chosen = set(layout.selected_pair_ids or layout.initial_pair_ids)
        extras = chosen-set(layout.initial_pair_ids)
        while len(extras) > extra_pairs:
            pid = max(extras, key=lambda p: (self.split_nodes[p].depth, p))
            chosen.remove(pid)
            extras.remove(pid)
        # 规划时也允许引用其他区域已打开的对；同一全域节点只会实际打开一次。
        opened = self.planned_open_pairs() | set(layout.initial_pair_ids)
        while len(extras) < extra_pairs:
            available = [p for p in layout.interior_pair_ids if p not in chosen
                         and self.split_nodes[p].left_pair_id in opened|chosen
                         and self.split_nodes[p].right_pair_id in opened|chosen]
            if not available:
                raise ValueError("本区域没有可细分的叶间隙；须先开放其边界管道对")
            pid = min(available, key=lambda p: (self.split_nodes[p].depth, p))
            chosen.add(pid)
            extras.add(pid)
        return sorted(chosen, key=self.pair_offset)

    def pair_has_count(self, pair_id: str) -> bool:
        return any(self.segments[sid].count > 0 for pid in self.pairs[pair_id].pipe_ids
                   for sid in self.pipes[pid].segment_ids)

    def planned_open_pairs(self, override: tuple[str, int | list[str]] | None = None) -> set[str]:
        desired = set(self.manual_pair_ids)
        for lid, layout in self.layouts.items():
            selected = layout.selected_pair_ids
            if override and override[0] == lid:
                selected = self.layout_pair_ids(lid, override[1]) if isinstance(override[1], int) else override[1]
            desired.update(self._with_ancestors(set(selected)))
        return desired

    def apply_layout(self, layout_id: str, extra_pairs: int, *, selected_pair_ids: list[str] | None = None) -> dict:
        """更新引用，整对连续开关；占用或子树需要的旧节点继续保留。"""
        selected = self.layout_pair_ids(layout_id, extra_pairs) if selected_pair_ids is None else selected_pair_ids
        before = {p.id for p in self.pairs.values() if self.pipes[p.pipe_ids[0]].is_open}
        layout = self.layouts[layout_id]
        initial = set(layout.initial_pair_ids)
        if (type(extra_pairs) is not int or extra_pairs < 0 or len(set(selected)) != len(selected)
                or not initial <= set(selected) or not set(selected)-initial <= set(layout.interior_pair_ids)
                or len(set(selected)-initial) != extra_pairs):
            raise ValueError("新增引用与k或该区域候选不一致")
        selected = sorted(selected, key=self.pair_offset)
        proposed = self.planned_open_pairs((layout_id, selected))
        for pid in proposed-before:
            node = self.split_nodes.get(pid)
            if node and not {node.left_pair_id, node.right_pair_id} <= proposed:
                raise ValueError("二叉细分节点的边界管道对尚未开放")
        layout.extra_pairs, layout.selected_pair_ids = extra_pairs, selected
        desired = self.planned_open_pairs()
        retained = self._with_ancestors({pid for pid in before-desired
                                         if self.pair_has_count(pid) or pid in self.retained_pair_ids})-desired
        after = desired | retained
        for pid in before ^ after:
            for pipe_id in self.pairs[pid].pipe_ids:
                self.pipes[pipe_id].is_open = pid in after
        self.retained_pair_ids = sorted(retained)
        return dict(opened=sorted(after-before), closed=sorted(before-after), retained=sorted(retained))

    def set_pair_open(self, pair_id: str, opened: bool):
        if type(opened) is not bool:
            raise ValueError("opened 必须为布尔值")
        pair = self.pairs[pair_id]
        if not opened and self.pair_has_count(pair_id):
            raise ValueError("管道对仍有无人机，不能关闭")
        if not opened and any(pair_id in layout.selected_pair_ids for layout in self.layouts.values()):
            raise ValueError("该管道对仍被区域方案引用；请先调整相应区域方案")
        if not opened and self.has_open_children(pair_id):
            raise ValueError("该管道对仍有开放子节点，不能关闭")
        if opened and pair_id not in self.manual_pair_ids:
            self.manual_pair_ids.append(pair_id)
        if not opened and pair_id in self.manual_pair_ids:
            self.manual_pair_ids.remove(pair_id)
        if not opened and pair_id in self.retained_pair_ids:
            self.retained_pair_ids.remove(pair_id)
        for pid in pair.pipe_ids:
            self.pipes[pid].is_open = opened

    def initialize(self, open_pair_ids: list[str] | None = None):
        """默认采用以左上角区域为参考的全域等间距方案，整根连续开关。

        显式传 [] 表示全关闭；传编号列表可完全指定开放方案。
        人工 disabled 的匝道保留其 enabled 配置。
        """
        missing = set(open_pair_ids or []) - self.pairs.keys()
        if missing:
            raise ValueError(f"未知管道对: {sorted(missing)}")
        self.manual_pair_ids = list(dict.fromkeys(open_pair_ids or []))
        self.retained_pair_ids = []
        for layout in self.layouts.values():
            layout.extra_pairs = 0
            layout.selected_pair_ids = list(layout.initial_pair_ids) if open_pair_ids is None else []
        desired = self.planned_open_pairs()
        for pair in self.pairs.values():
            for pid in pair.pipe_ids:
                self.pipes[pid].is_open = pair.id in desired
        for pipe in self.pipes.values():
            pipe.speed_mps = self.config.initial_speed_mps
        for segment in self.segments.values():
            segment.count = 0
            segment.passed = 0
            segment.demand = 0.0
        for ramp in self.ramps.values():
            ramp.congestion_blocked = False
        self.initialized = True

    def segment_throughput(self, segment_id: str) -> float:
        pipe = self.pipes[self.segments[segment_id].pipe_id]
        return self.throughput_model.throughput(pipe.speed_mps, self.config.throughput_window_s) if pipe.is_open else 0.0

    def ramp_is_open(self, ramp_id: str) -> bool:
        ramp = self.ramps[ramp_id]
        return (ramp.enabled and not ramp.congestion_blocked
                and self.pipes[self.segments[ramp.source_segment].pipe_id].is_open
                and self.pipes[self.segments[ramp.target_segment].pipe_id].is_open)

    def region_totals(self, region_id: str, layer_id: int | None = None) -> tuple[float, float]:
        """当前滚动窗口实际通过量、当前开放结构的可支持通过量。"""
        ids = [sid for sid in self.regions[region_id].segment_ids
               if layer_id is None or self.pipes[self.segments[sid].pipe_id].layer_id == layer_id]
        return sum(self.segments[s].passed for s in ids), sum(self.segment_throughput(s) for s in ids)

    def to_dict(self) -> dict:
        def serialize(value):
            if isinstance(value, ThroughputModel):
                return value.metadata()
            if is_dataclass(value):
                return {item.name: serialize(getattr(value, item.name)) for item in fields(value)}
            if isinstance(value, dict):
                return {key: serialize(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return type(value)(serialize(item) for item in value)
            return deepcopy(value)
        return serialize(self)
