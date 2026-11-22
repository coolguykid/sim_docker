"""
airspace.py — 飞行管道、匝道结构与空域路由图
"""
import numpy as np
import heapq
from uav_sim.airspace.config import get_airspace_config
from uav_sim.airspace.ramp_topology import (
    get_ramp_topology_profile,
    select_spatially_diverse_candidates,
)
from uav_sim.airspace.ramp_cost import switch_edge_cost


class Lane:
    """单个飞行车道"""

    def __init__(self, lane_id, start_pt, end_pt, external_lane_id=None):
        self.lane_id = lane_id
        self.external_lane_id = (
            lane_id if external_lane_id is None else external_lane_id
        )
        self.start_pt = np.array(start_pt)
        self.end_pt = np.array(end_pt)
        self.length = np.linalg.norm(self.end_pt - self.start_pt)


class Pipeline:
    """飞行管道，包含多个车道"""

    def __init__(
        self,
        pipe_id,
        start_pt,
        end_pt,
        height,
        max_speed,
        num_lanes=10,
        lane_width=10,
        region_id=0,
        region_row=0,
        region_col=0,
        local_pipe_id=None,
        local_layer_index=None,
        layer_index=None,
        d_safe=None,
        max_capacity=None,
        pipe_width=None,
        external_pipe_id=None,
        external_layer_id=None,
        external_start_node_id=None,
        external_end_node_id=None,
        start_node_regions=None,
        end_node_regions=None,
    ):
        self.pipe_id = pipe_id
        self.start_pt = np.array(start_pt)
        self.end_pt = np.array(end_pt)
        self.height = height
        self.max_speed = max_speed
        self.num_lanes = num_lanes
        self.lane_width = lane_width
        self.region_id = region_id
        self.region_row = region_row
        self.region_col = region_col
        self.local_pipe_id = pipe_id if local_pipe_id is None else local_pipe_id
        self.local_layer_index = 0 if local_layer_index is None else local_layer_index
        self.layer_index = 0 if layer_index is None else layer_index
        self.d_safe = None if d_safe is None else float(d_safe)
        self.max_capacity = max_capacity
        self.pipe_width = (
            float(pipe_width)
            if pipe_width is not None
            else float(num_lanes) * float(lane_width)
        )
        self.external_pipe_id = (
            pipe_id if external_pipe_id is None else external_pipe_id
        )
        self.external_layer_id = external_layer_id
        self.external_start_node_id = external_start_node_id
        self.external_end_node_id = external_end_node_id
        self.start_node_regions = tuple(
            start_node_regions if start_node_regions is not None else (region_id,)
        )
        self.end_node_regions = tuple(
            end_node_regions if end_node_regions is not None else (region_id,)
        )
        self._lane_specs = None
        vector = self.end_pt - self.start_pt
        self.length = np.linalg.norm(vector)
        self.direction = vector / self.length if self.length > 0 else np.zeros(3)
        self.lanes = self._generate_lanes()

    def _generate_lanes(self):
        if self._lane_specs is not None:
            return [
                Lane(
                    lane_id=lane_id,
                    start_pt=start_pt,
                    end_pt=end_pt,
                    external_lane_id=external_lane_id,
                )
                for lane_id, external_lane_id, start_pt, end_pt in self._lane_specs
            ]
        lanes = []
        perp_dir = np.array([-self.direction[1], self.direction[0], 0])
        offsets = np.linspace(
            -(self.num_lanes - 1) / 2, (self.num_lanes - 1) / 2, self.num_lanes
        )
        for i, offset in enumerate(offsets):
            shift_vector = perp_dir * offset * self.lane_width
            lanes.append(
                Lane(
                    lane_id=i,
                    start_pt=self.start_pt + shift_vector,
                    end_pt=self.end_pt + shift_vector,
                )
            )
        return lanes

    def set_database_lanes(self, lane_specs):
        self._lane_specs = [
            (
                int(spec["lane_id"]),
                spec.get("external_lane_id", spec["lane_id"]),
                tuple((float(value) for value in spec["start_pt"])),
                tuple((float(value) for value in spec["end_pt"])),
            )
            for spec in lane_specs
        ]
        self.num_lanes = len(self._lane_specs)
        self.lanes = self._generate_lanes()

    def __getstate__(self):
        state = self.__dict__.copy()
        state.pop("lanes", None)
        for key in ("start_pt", "end_pt", "direction"):
            state[key] = tuple((float(v) for v in state[key]))
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        for key in ("start_pt", "end_pt", "direction"):
            self.__dict__[key] = np.array(self.__dict__[key], dtype=float)
        self.lanes = self._generate_lanes()


class Ramp:
    """飞行匝道：连接同层或跨层管道"""

    def __init__(
        self,
        ramp_id,
        pipe_a,
        pipe_b,
        xy_pos=None,
        pos_a_3d=None,
        pos_b_3d=None,
        switch_radius=50.0,
        time_penalty=None,
        is_dynamic=False,
        ramp_type="inter_layer",
        base_station_pos=None,
        is_cross_region=False,
        boundary_side=None,
        is_bidirectional=False,
        external_ramp_id=None,
        pair_id=None,
        external_entry_node_id=None,
        external_exit_node_id=None,
        external_region_id=None,
    ):
        self.ramp_id = ramp_id
        self.pipe_a_id = pipe_a.pipe_id
        self.pipe_b_id = pipe_b.pipe_id
        self.is_bidirectional = bool(is_bidirectional)
        self.external_ramp_id = (
            ramp_id if external_ramp_id is None else external_ramp_id
        )
        self.pair_id = pair_id
        self.external_entry_node_id = external_entry_node_id
        self.external_exit_node_id = external_exit_node_id
        self.external_region_id = external_region_id
        self.ramp_type = ramp_type
        self.region_a_id = getattr(pipe_a, "region_id", 0)
        self.region_b_id = getattr(pipe_b, "region_id", 0)
        self.is_cross_region = is_cross_region
        self.boundary_side = boundary_side
        if pos_a_3d is not None and pos_b_3d is not None:
            self.pos_a = np.array(pos_a_3d, dtype=float)
            self.pos_b = np.array(pos_b_3d, dtype=float)
        else:
            self.pos_a = np.array([xy_pos[0], xy_pos[1], pipe_a.height])
            self.pos_b = np.array([xy_pos[0], xy_pos[1], pipe_b.height])
        self.switch_radius = switch_radius
        self.length = float(np.linalg.norm(self.pos_b - self.pos_a))
        nominal_speed = min(
            max(1.0, float(pipe_a.max_speed)), max(1.0, float(pipe_b.max_speed))
        )
        self.time_penalty = (
            self.length / nominal_speed if time_penalty is None else float(time_penalty)
        )
        self.is_dynamic = is_dynamic
        self.is_active = True
        self.dist_from_start_a = np.linalg.norm(self.pos_a - pipe_a.start_pt)
        self.dist_from_start_b = np.linalg.norm(self.pos_b - pipe_b.start_pt)
        self.mid_pt = (self.pos_a + self.pos_b) / 2.0
        if base_station_pos is None:
            base_station_pos = [2500.0, 2500.0, 0.0]
        self.base_station_pos = np.array(base_station_pos, dtype=float)
        vec = self.mid_pt - self.base_station_pos
        self.distance_to_bs = np.linalg.norm(vec)
        self.azimuth = np.arctan2(vec[1], vec[0])
        self.elevation = (
            np.arcsin(vec[2] / self.distance_to_bs) if self.distance_to_bs > 0 else 0.0
        )

    @property
    def source_pipe_id(self):
        """单向匝道的入口管道内部 ID。"""
        return self.pipe_a_id

    @property
    def target_pipe_id(self):
        """单向匝道的出口管道内部 ID。"""
        return self.pipe_b_id

    def travel_time_seconds(self, speed_mps, minimum_speed_mps=1.0):
        """返回按预计算长度得到的匝道通行时间。"""
        effective_speed = max(float(minimum_speed_mps), float(speed_mps))
        return self.length / effective_speed

    def __getstate__(self):
        state = self.__dict__.copy()
        for key in ("pos_a", "pos_b", "base_station_pos"):
            state[key] = tuple((float(v) for v in state[key]))
        state.pop("mid_pt", None)
        state.pop("distance_to_bs", None)
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        for key in ("pos_a", "pos_b", "base_station_pos"):
            self.__dict__[key] = np.array(self.__dict__[key], dtype=float)
        if not hasattr(self, "is_bidirectional"):
            self.is_bidirectional = False
        if not hasattr(self, "external_ramp_id"):
            self.external_ramp_id = self.ramp_id
        if not hasattr(self, "pair_id"):
            self.pair_id = None
        if not hasattr(self, "external_entry_node_id"):
            self.external_entry_node_id = None
        if not hasattr(self, "external_exit_node_id"):
            self.external_exit_node_id = None
        if not hasattr(self, "external_region_id"):
            self.external_region_id = None
        if not hasattr(self, "length"):
            self.length = float(np.linalg.norm(self.pos_b - self.pos_a))
        self.mid_pt = (self.pos_a + self.pos_b) / 2.0
        vec = self.mid_pt - self.base_station_pos
        self.distance_to_bs = np.linalg.norm(vec)
        self.azimuth = np.arctan2(vec[1], vec[0])
        self.elevation = (
            np.arcsin(vec[2] / self.distance_to_bs) if self.distance_to_bs > 0 else 0.0
        )


class PickupPoint:
    """乘客上车点（运力需求点）"""

    def __init__(self, pt_id, pipe_id, pos_1d, capacity=20, region_id=None):
        self.pt_id = pt_id
        self.pipe_id = pipe_id
        self.pos_1d = pos_1d
        self.capacity = capacity
        self.remaining = capacity
        self.region_id = region_id

    def reset(self):
        self.remaining = self.capacity


class Airspace:
    """飞行空域模型"""

    INTER_DYNAMIC_LEVELS = ("none", "low", "medium", "high", "ultra")
    INTRA_DYNAMIC_PER_LAYER = {
        "none": 0,
        "low": 10,
        "medium": 20,
        "high": 40,
        "ultra": 80,
    }

    def __init__(
        self,
        inter_dynamic_pct=None,
        intra_dynamic_pct=None,
        inter_dynamic_level="low",
        intra_dynamic_level="low",
        include_dynamic=True,
        random_seed=42,
        mode="standard",
        scale_config=None,
        topology_profile=None,
        num_pipes_per_layer=None,
        num_lanes_per_pipe=None,
        ramp_same_layer_degree=2,
        ramp_adjacent_layer_degree=4,
        ramp_express_layer_degree=1,
    ):
        self.scale_config = scale_config or get_airspace_config(mode)
        if num_pipes_per_layer is not None or num_lanes_per_pipe is not None:
            from dataclasses import replace
            overrides = {}
            for key, value in (("num_pipes_per_layer", num_pipes_per_layer),
                               ("num_lanes_per_pipe", num_lanes_per_pipe)):
                if value is not None:
                    if int(value) != value or int(value) < 1:
                        raise ValueError("{} must be a positive integer".format(key))
                    overrides[key] = int(value)
            self.scale_config = replace(self.scale_config, **overrides)
        self.airspace_mode = self.scale_config.mode.value
        self.topology_profile = get_ramp_topology_profile(
            topology_profile, airspace_mode=self.airspace_mode
        )
        for name, value in (("ramp_same_layer_degree", ramp_same_layer_degree),
                            ("ramp_adjacent_layer_degree", ramp_adjacent_layer_degree),
                            ("ramp_express_layer_degree", ramp_express_layer_degree)):
            if int(value) != value or int(value) < 0 or int(value) > 8:
                raise ValueError("{} must be an integer from 0 to 8".format(name))
            setattr(self, name, int(value))
        self.x_min, self.x_max = (self.scale_config.x_min, self.scale_config.x_max)
        self.y_min, self.y_max = (self.scale_config.y_min, self.scale_config.y_max)
        self.z_min, self.z_max = (0.0, self.scale_config.z_max)
        self.xy_span = self.x_max - self.x_min
        self.xy_diagonal = float(
            np.hypot(self.x_max - self.x_min, self.y_max - self.y_min)
        )
        self.z_span = max(1.0, self.z_max - self.z_min)
        self.base_station_pos = np.array(
            [
                self.x_min + self.xy_span / 2.0,
                self.y_min + (self.y_max - self.y_min) / 2.0,
                0.0,
            ]
        )
        self.margin = 50
        self.inter_dynamic_pct = inter_dynamic_pct
        self.intra_dynamic_pct = intra_dynamic_pct
        self.inter_dynamic_level = self._normalize_dynamic_level(
            inter_dynamic_level, inter_dynamic_pct
        )
        self.intra_dynamic_level = self._normalize_dynamic_level(
            intra_dynamic_level, intra_dynamic_pct
        )
        self.include_dynamic = include_dynamic
        self.random_seed = random_seed
        self.max_inter_ramp_xy_distance = 600.0
        self.max_intra_ramp_xy_distance = 500.0
        self.min_ramp_spacing = 60.0
        self.ramp_stats = {}
        self.ramp_generation_diagnostics = []
        self.continuation_edges = []
        self.external_id_maps = None
        self.topology_source = "generated"
        self.topology_version = 0
        self.region_rows = int(getattr(self.scale_config, "region_rows", 1) or 1)
        self.region_cols = int(getattr(self.scale_config, "region_cols", 1) or 1)
        configured_region_side = float(
            getattr(self.scale_config, "region_side_length", 0.0) or 0.0
        )
        self.region_side_length = configured_region_side or float(
            self.scale_config.side_length
        )
        self.num_lanes_per_pipe = int(
            getattr(self.scale_config, "num_lanes_per_pipe", 10)
        )
        self.lane_width = float(getattr(self.scale_config, "lane_width", 10.0))
        if self.airspace_mode == "metropolis":
            # Clip the full corridor, keeping the existing 50 m exterior
            # margin beyond its outermost actual lane in every direction.
            if self.lane_width <= 0.:
                raise ValueError("metropolis lane_width must be positive")
            self.margin += (self.num_lanes_per_pipe - 1) * self.lane_width / 2.
            if 2. * self.margin >= min(self.x_max - self.x_min, self.y_max - self.y_min):
                raise ValueError("metropolis lane corridor is wider than the world")
        self.regions = []
        self._pipes_by_region = {}
        self._pipe_lookup_by_region_layer = {}
        self.pipe_nodes = {}
        num_pipes = self.scale_config.num_pipes_per_layer
        self.layer_configs = [
            {"height": 100, "angle": 0, "speed": 20, "num_pipes": num_pipes},
            {"height": 200, "angle": 30, "speed": 30, "num_pipes": num_pipes},
            {"height": 300, "angle": 60, "speed": 40, "num_pipes": num_pipes},
            {"height": 400, "angle": 90, "speed": 60, "num_pipes": num_pipes},
            {"height": 500, "angle": 120, "speed": 80, "num_pipes": num_pipes},
            {"height": 600, "angle": 150, "speed": 100, "num_pipes": num_pipes},
        ]
        if self.airspace_mode == "metropolis":
            for layer, speed in zip(self.layer_configs, (19, 40, 60, 80, 100, 120)):
                layer["speed"] = speed
        self.pipelines = []
        self.ramps = []
        self._build_airspace()
        if self.airspace_mode == "metropolis":
            self._validate_launch_slot_clearance()
        self._build_ramps()
        self._build_routing_graph()
        self._initialize_mutable_runtime_state()

    @classmethod
    def from_adapted_topology(cls, topology):
        env = cls.__new__(cls)
        env.scale_config = None
        env.airspace_mode = "regional_6x6" if len(topology.regions) > 1 else "database"
        env.topology_profile = get_ramp_topology_profile(
            "balanced_sparse_v1", airspace_mode="regional_6x6"
        )
        env.topology_source = "postgres"
        env.topology_version = int(topology.topology_version)
        env.external_id_maps = topology.id_maps
        env.continuation_edges = list(topology.continuation_edges)
        env.pipelines = list(topology.pipelines)
        env.ramps = list(topology.ramps)
        env.regions = list(topology.regions)
        env.layer_configs = list(topology.layer_configs)
        env.ramp_generation_diagnostics = list(topology.diagnostics)
        env.ramp_stats = {
            "intra_static": sum(
                (
                    1
                    for ramp in env.ramps
                    if ramp.ramp_type == "intra_layer" and (not ramp.is_dynamic)
                )
            ),
            "inter_static": sum(
                (
                    1
                    for ramp in env.ramps
                    if ramp.ramp_type == "inter_layer" and (not ramp.is_dynamic)
                )
            ),
            "inter_static_pairs": 0,
            "inter_dynamic": sum(
                (
                    1
                    for ramp in env.ramps
                    if ramp.ramp_type == "inter_layer" and ramp.is_dynamic
                )
            ),
            "inter_dynamic_pairs": 0,
            "intra_dynamic": sum(
                (
                    1
                    for ramp in env.ramps
                    if ramp.ramp_type == "intra_layer" and ramp.is_dynamic
                )
            ),
            "cross_region": sum((1 for ramp in env.ramps if ramp.is_cross_region)),
        }
        env.inter_dynamic_pct = None
        env.intra_dynamic_pct = None
        env.inter_dynamic_level = "none"
        env.intra_dynamic_level = "none"
        env.include_dynamic = any((ramp.is_dynamic for ramp in env.ramps))
        env.random_seed = 0
        env.max_inter_ramp_xy_distance = 600.0
        env.max_intra_ramp_xy_distance = 500.0
        env.min_ramp_spacing = 60.0
        env.margin = 50.0
        env._pipes_by_region = {}
        env._pipe_lookup_by_region_layer = {}
        for pipe in env.pipelines:
            env._pipes_by_region.setdefault(int(pipe.region_id), []).append(pipe)
        env.region_rows = (
            max([int(region.get("row", 0)) for region in env.regions] or [0]) + 1
        )
        env.region_cols = (
            max([int(region.get("col", 0)) for region in env.regions] or [0]) + 1
        )
        env.region_side_length = 0.0
        env.num_lanes_per_pipe = max(
            [int(pipe.num_lanes) for pipe in env.pipelines] or [1]
        )
        env.lane_width = float(env.pipelines[0].lane_width if env.pipelines else 10.0)
        coordinates = [
            point for pipe in env.pipelines for point in (pipe.start_pt, pipe.end_pt)
        ]
        if coordinates:
            points = np.asarray(coordinates, dtype=float)
            env.x_min, env.y_min = np.min(points[:, :2], axis=0)
            env.x_max, env.y_max = np.max(points[:, :2], axis=0)
            env.z_min = min(0.0, float(np.min(points[:, 2])))
            env.z_max = float(np.max(points[:, 2]))
        else:
            env.x_min = env.y_min = env.z_min = 0.0
            env.x_max = env.y_max = env.z_max = 1.0
        env.xy_span = max(1.0, env.x_max - env.x_min)
        env.xy_diagonal = float(np.hypot(env.x_max - env.x_min, env.y_max - env.y_min))
        env.z_span = max(1.0, env.z_max - env.z_min)
        env.base_station_pos = np.array(
            [(env.x_min + env.x_max) / 2.0, (env.y_min + env.y_max) / 2.0, 0.0]
        )
        env._initialize_ramp_indexes()
        env._build_routing_graph()
        env._initialize_mutable_runtime_state()
        return env

    def _initialize_mutable_runtime_state(self):
        self.pipe_collisions_window = {p.pipe_id: 0 for p in self.pipelines}
        self.pipe_collisions_multiplier = {p.pipe_id: 1.0 for p in self.pipelines}
        self.pipe_traffic_load = {p.pipe_id: 0 for p in self.pipelines}
        self.routing_mode = "DIJKSTRA"

    def _normalize_dynamic_level(self, level, legacy_pct):
        """兼容旧百分比配置，同时把新配置统一成五档字符串。"""
        if legacy_pct is not None:
            if legacy_pct <= 0:
                return "none"
            if legacy_pct <= 0.1:
                return "low"
            if legacy_pct <= 0.3:
                return "medium"
            if legacy_pct <= 0.6:
                return "high"
            return "ultra"
        if level is None:
            return "low"
        normalized = str(level).strip().lower()
        aliases = {
            "无": "none",
            "低": "low",
            "中": "medium",
            "高": "high",
            "超高": "ultra",
            "none": "none",
            "low": "low",
            "medium": "medium",
            "mid": "medium",
            "high": "high",
            "ultra": "ultra",
        }
        normalized = aliases.get(normalized, normalized)
        if normalized not in self.INTER_DYNAMIC_LEVELS:
            raise ValueError(f"未知动态匝道密度档位: {level}")
        return normalized

    def _get_intersection_with_bounds(
        self, center_pt, direction_vec, x_min=None, x_max=None, y_min=None, y_max=None
    ):
        cx, cy = center_pt
        dx, dy = direction_vec
        dx = dx if abs(dx) > 1e-09 else 1e-09
        dy = dy if abs(dy) > 1e-09 else 1e-09
        x_min = self.x_min if x_min is None else x_min
        x_max = self.x_max if x_max is None else x_max
        y_min = self.y_min if y_min is None else y_min
        y_max = self.y_max if y_max is None else y_max
        eff_x_min, eff_x_max = (x_min + self.margin, x_max - self.margin)
        eff_y_min, eff_y_max = (y_min + self.margin, y_max - self.margin)
        t_x1, t_x2 = ((eff_x_min - cx) / dx, (eff_x_max - cx) / dx)
        t_x_min, t_x_max = (min(t_x1, t_x2), max(t_x1, t_x2))
        t_y1, t_y2 = ((eff_y_min - cy) / dy, (eff_y_max - cy) / dy)
        t_y_min, t_y_max = (min(t_y1, t_y2), max(t_y1, t_y2))
        t_enter = max(t_x_min, t_y_min)
        t_exit = min(t_x_max, t_y_max)
        if t_enter <= t_exit:
            return (
                np.array([cx + t_enter * dx, cy + t_enter * dy]),
                np.array([cx + t_exit * dx, cy + t_exit * dy]),
            )
        return (None, None)

    def _build_airspace(self):
        """生成6层飞行管道；区域模式下生成 6x6 个 5km 小空域。"""
        self.regions = []
        self._pipes_by_region = {}
        self._pipe_lookup_by_region_layer = {}
        pipe_counter = 0
        for row in range(self.region_rows):
            for col in range(self.region_cols):
                region_id = row * self.region_cols + col
                x0 = self.x_min + col * self.region_side_length
                y0 = self.y_min + row * self.region_side_length
                region = {
                    "region_id": region_id,
                    "row": row,
                    "col": col,
                    "x_min": x0,
                    "x_max": x0 + self.region_side_length,
                    "y_min": y0,
                    "y_max": y0 + self.region_side_length,
                    "center_x": x0 + self.region_side_length / 2.0,
                    "center_y": y0 + self.region_side_length / 2.0,
                }
                self.regions.append(region)
                pipe_counter = self._build_region_pipelines(region, pipe_counter)

    def _build_region_pipelines(self, region, pipe_counter):
        center_x, center_y = (region["center_x"], region["center_y"])
        region_id = region["region_id"]
        local_pipe_counter = 0
        for layer in self.layer_configs:
            layer_index = self.layer_configs.index(layer)
            h = layer["height"]
            angle_rad = np.radians(layer["angle"])
            speed = layer["speed"]
            num_pipes = layer["num_pipes"]
            dir_vec = np.array([np.cos(angle_rad), np.sin(angle_rad)])
            normal_vec = np.array([-np.sin(angle_rad), np.cos(angle_rad)])
            spacing_extent = self.region_side_length * 0.4
            spacings = np.linspace(-spacing_extent, spacing_extent, num_pipes)
            for i, d in enumerate(spacings):
                ref_pt = np.array([center_x, center_y]) + normal_vec * d
                pt1, pt2 = self._get_intersection_with_bounds(
                    ref_pt,
                    dir_vec,
                    x_min=region["x_min"],
                    x_max=region["x_max"],
                    y_min=region["y_min"],
                    y_max=region["y_max"],
                )
                if pt1 is not None and pt2 is not None:
                    start_3d = [pt1[0], pt1[1], h]
                    end_3d = [pt2[0], pt2[1], h]
                    if i % 2 == 1:
                        start_3d, end_3d = (end_3d, start_3d)
                    pipe = Pipeline(
                        pipe_id=pipe_counter,
                        start_pt=start_3d,
                        end_pt=end_3d,
                        height=h,
                        max_speed=speed,
                        num_lanes=self.num_lanes_per_pipe,
                        lane_width=self.lane_width,
                        region_id=region_id,
                        region_row=region["row"],
                        region_col=region["col"],
                        local_pipe_id=local_pipe_counter,
                        local_layer_index=i,
                        layer_index=layer_index,
                    )
                    self.pipelines.append(pipe)
                    self._pipes_by_region.setdefault(region_id, []).append(pipe)
                    self._pipe_lookup_by_region_layer[region_id, h, i] = pipe
                    pipe_counter += 1
                    local_pipe_counter += 1
        return pipe_counter

    def _validate_launch_slot_clearance(self):
        """Reject overlapping bottom-layer starts before publishing topology.

        Adjacent lanes establish the minimum lateral separation: lane_width.
        A spatial index only finds candidate pairs; the final check uses their
        actual unrounded positions, including starts on different pipes.
        """
        from collections import defaultdict
        from math import floor

        clearance = float(self.lane_width)
        bottom_height = min(layer["height"] for layer in self.layer_configs)
        cells = defaultdict(list)
        for pipe in self.pipelines:
            if pipe.height != bottom_height:
                continue
            for lane in pipe.lanes:
                position = lane.start_pt
                cell = (floor(float(position[0]) / clearance), floor(float(position[1]) / clearance))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for other_pipe, other_lane, other_position in cells[cell[0] + dx, cell[1] + dy]:
                            distance = float(np.linalg.norm(position - other_position))
                            if distance + 1.e-6 < clearance:
                                raise ValueError(
                                    "launch slot clearance violated: P{}/L{} and P{}/L{} are {:.9g} m apart; require {:.9g} m".format(
                                        other_pipe, other_lane, pipe.pipe_id, lane.lane_id, distance, clearance))
                cells[cell].append((pipe.pipe_id, lane.lane_id, position))

    def _get_segments_intersection_2d(self, p1, p2, p3, p4):
        x1, y1 = (p1[0], p1[1])
        x2, y2 = (p2[0], p2[1])
        x3, y3 = (p3[0], p3[1])
        x4, y4 = (p4[0], p4[1])
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-06:
            return None
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
        if 0 <= t <= 1 and 0 <= u <= 1:
            return np.array([x1 + t * (x2 - x1), y1 + t * (y2 - y1)])
        return None

    def _add_ramp(
        self,
        pipe_a,
        pipe_b,
        pos_a_3d,
        pos_b_3d,
        time_penalty,
        is_dynamic,
        ramp_type,
        is_cross_region=False,
        boundary_side=None,
        is_bidirectional=False,
        external_ramp_id=None,
        pair_id=None,
    ):
        ramp = Ramp(
            len(self.ramps),
            pipe_a,
            pipe_b,
            pos_a_3d=np.asarray(pos_a_3d, dtype=float).tolist(),
            pos_b_3d=np.asarray(pos_b_3d, dtype=float).tolist(),
            time_penalty=time_penalty,
            is_dynamic=is_dynamic,
            ramp_type=ramp_type,
            base_station_pos=self.base_station_pos,
            is_cross_region=is_cross_region,
            boundary_side=boundary_side,
            is_bidirectional=is_bidirectional,
            external_ramp_id=external_ramp_id,
            pair_id=pair_id,
        )
        self.ramps.append(ramp)
        ramp_positions = getattr(self, "_ramp_positions_during_build", None)
        if ramp_positions is not None:
            ramp_positions.setdefault(ramp.pipe_a_id, []).append(ramp.dist_from_start_a)
            ramp_positions.setdefault(ramp.pipe_b_id, []).append(ramp.dist_from_start_b)
        return ramp

    @staticmethod
    def _pair_key(pipe_a, pipe_b):
        return tuple(sorted((pipe_a.pipe_id, pipe_b.pipe_id)))

    @staticmethod
    def _logical_pair_key(pipe_a, pipe_b):
        return tuple(sorted((int(pipe_a.pipe_id), int(pipe_b.pipe_id))))

    @staticmethod
    def _directed_pair_key(source_pipe, target_pipe):
        return (int(source_pipe.pipe_id), int(target_pipe.pipe_id))

    @staticmethod
    def _clamp(value, low=0.0, high=1.0):
        return max(low, min(high, value))

    def _closest_points_between_pipes_2d(self, pipe_a, pipe_b):
        p0 = pipe_a.start_pt[:2].astype(float)
        p1 = pipe_a.end_pt[:2].astype(float)
        q0 = pipe_b.start_pt[:2].astype(float)
        q1 = pipe_b.end_pt[:2].astype(float)
        u = p1 - p0
        v = q1 - q0
        w = p0 - q0
        a = float(np.dot(u, u))
        b = float(np.dot(u, v))
        c = float(np.dot(v, v))
        d = float(np.dot(u, w))
        e = float(np.dot(v, w))
        denom = a * c - b * b
        if a <= 1e-09 or c <= 1e-09:
            return (0.0, 0.0, float(np.linalg.norm(p0 - q0)))
        if abs(denom) < 1e-09:
            s = 0.0
            t = self._clamp(e / c)
        else:
            s = self._clamp((b * e - c * d) / denom)
            t = (b * s + e) / c
            if t < 0.0:
                t = 0.0
                s = self._clamp(-d / a)
            elif t > 1.0:
                t = 1.0
                s = self._clamp((b - d) / a)
        xy_a = p0 + s * u
        xy_b = q0 + t * v
        return (
            s * pipe_a.length,
            t * pipe_b.length,
            float(np.linalg.norm(xy_a - xy_b)),
        )

    def _inter_layer_candidates(self, layer_pipes, adjacent=None, cross=None):
        heights = sorted(layer_pipes.keys())
        candidates = []
        for i, h1 in enumerate(heights):
            for h2 in heights[i + 1 :]:
                layer_gap = int(abs(h1 - h2) / 100)
                is_adjacent = layer_gap == 1
                if adjacent is not None and is_adjacent != adjacent:
                    continue
                if cross is not None and (layer_gap > 1) != cross:
                    continue
                for pipe_a in layer_pipes[h1]:
                    for pipe_b in layer_pipes[h2]:
                        sa, sb, xy_dist = self._closest_points_between_pipes_2d(
                            pipe_a, pipe_b
                        )
                        candidates.append(
                            {
                                "pipe_a": pipe_a,
                                "pipe_b": pipe_b,
                                "sa": sa,
                                "sb": sb,
                                "xy_dist": xy_dist,
                                "layer_gap": layer_gap,
                            }
                        )
        candidates.sort(
            key=lambda c: (c["xy_dist"], c["pipe_a"].pipe_id, c["pipe_b"].pipe_id)
        )
        return candidates

    def _record_ramp_positions(self):
        existing = {p.pipe_id: [] for p in self.pipelines}
        for ramp in self.ramps:
            existing[ramp.pipe_a_id].append(ramp.dist_from_start_a)
            existing[ramp.pipe_b_id].append(ramp.dist_from_start_b)
        return existing

    def _has_position_space(self, existing_ramp_pos, pipe_id, pos_1d):
        existing = existing_ramp_pos.get(pipe_id, [])
        return all((abs(pos_1d - old) > self.min_ramp_spacing for old in existing))

    @staticmethod
    def _position_on_pipe(pipe, distance):
        distance = max(0.0, min(float(pipe.length), float(distance)))
        return pipe.start_pt + pipe.direction * distance

    def _directed_geometry_from_candidate(
        self, candidate, reverse=False, existing_ramp_pos=None, forward_geometry=None
    ):
        """为一个逻辑层间连接选择某一方向的独立入口和出口。"""
        pipe_a = candidate["pipe_a"]
        pipe_b = candidate["pipe_b"]
        if not reverse:
            source_pipe, target_pipe = (pipe_a, pipe_b)
            source_dist = float(candidate["sa"])
            target_dist = float(candidate["sb"])
            return {
                "source_pipe": source_pipe,
                "target_pipe": target_pipe,
                "source_dist": source_dist,
                "target_dist": target_dist,
                "entry": self._position_on_pipe(source_pipe, source_dist),
                "exit": self._position_on_pipe(target_pipe, target_dist),
            }
        source_pipe, target_pipe = (pipe_b, pipe_a)
        source_base = float(candidate["sb"])
        target_base = float(candidate["sa"])
        spacing = float(self.min_ramp_spacing)
        offsets = (spacing, -spacing, 2.0 * spacing, -2.0 * spacing)
        forward_entry = np.asarray(forward_geometry["entry"], dtype=float)
        forward_exit = np.asarray(forward_geometry["exit"], dtype=float)
        existing_ramp_pos = existing_ramp_pos or {}
        choices = []
        for source_offset in offsets:
            source_dist = max(0.0, min(source_pipe.length, source_base + source_offset))
            if abs(source_dist - source_base) <= 1e-09:
                continue
            if not self._has_position_space(
                existing_ramp_pos, source_pipe.pipe_id, source_dist
            ):
                continue
            for target_offset in offsets:
                target_dist = max(
                    0.0, min(target_pipe.length, target_base + target_offset)
                )
                if abs(target_dist - target_base) <= 1e-09:
                    continue
                if not self._has_position_space(
                    existing_ramp_pos, target_pipe.pipe_id, target_dist
                ):
                    continue
                entry = self._position_on_pipe(source_pipe, source_dist)
                exit_pt = self._position_on_pipe(target_pipe, target_dist)
                separation = float(np.linalg.norm(entry - forward_exit)) + float(
                    np.linalg.norm(exit_pt - forward_entry)
                )
                length = float(np.linalg.norm(exit_pt - entry))
                choices.append(
                    (separation, -length, source_dist, target_dist, entry, exit_pt)
                )
        if choices:
            _separation, _neg_length, source_dist, target_dist, entry, exit_pt = max(
                choices, key=lambda item: item[:2]
            )
        else:
            source_candidates = (0.0, float(source_pipe.length))
            target_candidates = (0.0, float(target_pipe.length))
            source_dist = max(
                source_candidates, key=lambda value: abs(value - source_base)
            )
            target_dist = max(
                target_candidates, key=lambda value: abs(value - target_base)
            )
            entry = self._position_on_pipe(source_pipe, source_dist)
            exit_pt = self._position_on_pipe(target_pipe, target_dist)
            self.ramp_generation_diagnostics.append(
                {
                    "type": "directed_geometry_fallback",
                    "source_pipe_id": int(source_pipe.pipe_id),
                    "target_pipe_id": int(target_pipe.pipe_id),
                }
            )
        return {
            "source_pipe": source_pipe,
            "target_pipe": target_pipe,
            "source_dist": float(source_dist),
            "target_dist": float(target_dist),
            "entry": entry,
            "exit": exit_pt,
        }

    def _build_directed_inter_ramp_pair(
        self, candidate, is_dynamic, existing_ramp_pos=None, pair_id=None
    ):
        existing_ramp_pos = (
            existing_ramp_pos
            if existing_ramp_pos is not None
            else self._ramp_positions_during_build
        )
        pipe_a = candidate["pipe_a"]
        pipe_b = candidate["pipe_b"]
        if pair_id is None:
            left, right = self._logical_pair_key(pipe_a, pipe_b)
            pair_id = "generated:{}:{}".format(left, right)
        forward = self._directed_geometry_from_candidate(
            candidate, reverse=False, existing_ramp_pos=existing_ramp_pos
        )
        ramp_ab = self._add_ramp(
            forward["source_pipe"],
            forward["target_pipe"],
            forward["entry"],
            forward["exit"],
            None,
            is_dynamic,
            "inter_layer",
            is_bidirectional=False,
            pair_id=pair_id,
        )
        reverse = self._directed_geometry_from_candidate(
            candidate,
            reverse=True,
            existing_ramp_pos=existing_ramp_pos,
            forward_geometry=forward,
        )
        ramp_ba = self._add_ramp(
            reverse["source_pipe"],
            reverse["target_pipe"],
            reverse["entry"],
            reverse["exit"],
            None,
            is_dynamic,
            "inter_layer",
            is_bidirectional=False,
            pair_id=pair_id,
        )
        return (ramp_ab, ramp_ba)

    def _add_inter_ramp_from_candidate(
        self, candidate, is_dynamic, existing_ramp_pos=None
    ):
        pipe_a = candidate["pipe_a"]
        pipe_b = candidate["pipe_b"]
        left, right = self._logical_pair_key(pipe_a, pipe_b)
        pair_id = "generated:{}:{}".format(left, right)
        if self.topology_profile.directed_inter_layer:
            return self._build_directed_inter_ramp_pair(
                candidate,
                is_dynamic=is_dynamic,
                existing_ramp_pos=existing_ramp_pos,
                pair_id=pair_id,
            )
        pos_a = self._position_on_pipe(pipe_a, candidate["sa"])
        pos_b = self._position_on_pipe(pipe_b, candidate["sb"])
        ramp = self._add_ramp(
            pipe_a,
            pipe_b,
            pos_a,
            pos_b,
            None,
            is_dynamic,
            "inter_layer",
            is_bidirectional=True,
            pair_id=pair_id,
        )
        return (ramp,)

    def _build_static_intra_ramps(self, layer_pipes):
        count = 0
        for height in sorted(layer_pipes.keys()):
            pipes = sorted(layer_pipes[height], key=lambda p: p.pipe_id)
            n = len(pipes)
            for idx, pipe_a in enumerate(pipes):
                pipe_b = pipes[(idx + 1) % n]
                switch_time = 1.0 + 0.1 * min(1, n - 1)
                self._add_ramp(
                    pipe_a,
                    pipe_b,
                    pipe_a.end_pt,
                    pipe_b.start_pt,
                    switch_time,
                    False,
                    "intra_layer",
                )
                count += 1
        return count

    def _build_static_inter_ramps(self, layer_pipes):
        if not self.topology_profile.legacy_degree_builder:
            return self._build_sparse_static_inter_ramps(layer_pipes)
        existing_pairs = set()
        adjacent_count = self._satisfy_static_inter_degree(
            layer_pipes, existing_pairs, target_per_pipe=8, adjacent=True
        )
        cross_count = self._satisfy_static_inter_degree(
            layer_pipes, existing_pairs, target_per_pipe=2, adjacent=False
        )
        return adjacent_count + cross_count

    def _build_sparse_static_inter_ramps(self, layer_pipes):
        heights = sorted(layer_pipes)
        count = 0
        for left_index, height_a in enumerate(heights):
            for height_b in heights[left_index + 1 :]:
                layer_gap = int(abs(height_a - height_b) / 100)
                target = (
                    self.topology_profile.adjacent_layer_links_per_pair
                    if layer_gap == 1
                    else self.topology_profile.express_layer_links_per_pair
                )
                candidates = []
                for pipe_a in layer_pipes[height_a]:
                    for pipe_b in layer_pipes[height_b]:
                        sa, sb, xy_dist = self._closest_points_between_pipes_2d(
                            pipe_a, pipe_b
                        )
                        candidates.append(
                            {
                                "pipe_a": pipe_a,
                                "pipe_b": pipe_b,
                                "sa": sa,
                                "sb": sb,
                                "xy_dist": xy_dist,
                                "layer_gap": layer_gap,
                            }
                        )
                for candidate in select_spatially_diverse_candidates(
                    candidates, target
                ):
                    ramps = self._add_inter_ramp_from_candidate(
                        candidate,
                        is_dynamic=False,
                        existing_ramp_pos=self._ramp_positions_during_build,
                    )
                    count += len(ramps)
        return count

    def _satisfy_static_inter_degree(
        self, layer_pipes, existing_pairs, target_per_pipe, adjacent
    ):
        pipes = sorted(
            [p for pipes_in_layer in layer_pipes.values() for p in pipes_in_layer],
            key=lambda p: p.pipe_id,
        )
        counts = {p.pipe_id: 0 for p in pipes}
        added = 0
        made_progress = True
        while made_progress:
            made_progress = False
            for source in pipes:
                while counts[source.pipe_id] < target_per_pipe:
                    candidate = self._best_static_inter_candidate(
                        source, layer_pipes, existing_pairs, counts, adjacent
                    )
                    if candidate is None:
                        break
                    ramps = self._add_inter_ramp_from_candidate(
                        candidate,
                        is_dynamic=False,
                        existing_ramp_pos=self._ramp_positions_during_build,
                    )
                    pipe_a = candidate["pipe_a"]
                    pipe_b = candidate["pipe_b"]
                    existing_pairs.add(self._pair_key(pipe_a, pipe_b))
                    counts[pipe_a.pipe_id] += 1
                    counts[pipe_b.pipe_id] += 1
                    added += len(ramps)
                    made_progress = True
        return added

    def _best_static_inter_candidate(
        self, source, layer_pipes, existing_pairs, counts, adjacent
    ):
        candidates = []
        for height, target_pipes in layer_pipes.items():
            layer_gap = int(abs(source.height - height) / 100)
            if layer_gap == 0:
                continue
            if adjacent and layer_gap != 1:
                continue
            if not adjacent and layer_gap <= 1:
                continue
            for target in target_pipes:
                key = self._pair_key(source, target)
                if key in existing_pairs:
                    continue
                sa, sb, xy_dist = self._closest_points_between_pipes_2d(source, target)
                candidates.append(
                    {
                        "pipe_a": source,
                        "pipe_b": target,
                        "sa": sa,
                        "sb": sb,
                        "xy_dist": xy_dist,
                        "layer_gap": layer_gap,
                    }
                )
        if not candidates:
            return None
        candidates.sort(
            key=lambda c: (
                counts[c["pipe_b"].pipe_id],
                c["xy_dist"],
                c["pipe_b"].pipe_id,
            )
        )
        return candidates[0]

    def _inter_dynamic_target_count(self, static_total, missing_pair_count):
        if not self.include_dynamic or self.inter_dynamic_level == "none":
            return 0
        if self.inter_dynamic_level == "low":
            return min(static_total, missing_pair_count)
        if self.inter_dynamic_level == "medium":
            return min(static_total * 2, missing_pair_count)
        if self.inter_dynamic_level == "high":
            return min(static_total * 4, missing_pair_count)
        return missing_pair_count

    def _build_dynamic_inter_ramps(
        self, layer_pipes, existing_inter_pairs, static_total=None
    ):
        candidates = [
            c
            for c in self._inter_layer_candidates(layer_pipes)
            if self._pair_key(c["pipe_a"], c["pipe_b"]) not in existing_inter_pairs
        ]
        if static_total is None:
            static_total = self.ramp_stats.get("intra_static", 0) + self.ramp_stats.get(
                "inter_static", 0
            )
        target = self._inter_dynamic_target_count(static_total, len(candidates))
        count = 0
        for candidate in candidates[:target]:
            ramps = self._add_inter_ramp_from_candidate(
                candidate,
                is_dynamic=True,
                existing_ramp_pos=self._ramp_positions_during_build,
            )
            existing_inter_pairs.add(
                self._pair_key(candidate["pipe_a"], candidate["pipe_b"])
            )
            count += len(ramps)
        return count

    def _build_dynamic_intra_ramps(
        self, layer_pipes, pipe_layer_index, existing_ramp_pos, rng
    ):
        if not self.include_dynamic:
            return 0
        per_layer_target = self._intra_dynamic_per_layer_target()
        total_count = 0
        for height in sorted(layer_pipes.keys()):
            pipes = sorted(layer_pipes[height], key=lambda p: p.pipe_id)
            pairs = []
            for pipe_a in pipes:
                for pipe_b in pipes:
                    if pipe_a.pipe_id == pipe_b.pipe_id:
                        continue
                    if np.dot(pipe_a.direction[:2], pipe_b.direction[:2]) <= -0.3:
                        pairs.append((pipe_a, pipe_b))
            layer_count = 0
            attempts = 0
            max_attempts = max(3000, per_layer_target * 250)
            while layer_count < per_layer_target and attempts < max_attempts and pairs:
                attempts += 1
                pipe_a, pipe_b = rng.choice(pairs)
                sa = rng.uniform(200.0, max(201.0, pipe_a.length - 200.0))
                sb = rng.uniform(200.0, max(201.0, pipe_b.length - 200.0))
                pos_a_2d = pipe_a.start_pt[:2] + pipe_a.direction[:2] * sa
                pos_b_2d = pipe_b.start_pt[:2] + pipe_b.direction[:2] * sb
                if (
                    np.linalg.norm(pos_a_2d - pos_b_2d)
                    > self.max_intra_ramp_xy_distance
                ):
                    continue
                if not self._has_position_space(existing_ramp_pos, pipe_a.pipe_id, sa):
                    continue
                if not self._has_position_space(existing_ramp_pos, pipe_b.pipe_id, sb):
                    continue
                idx_a = pipe_layer_index[pipe_a.pipe_id]
                idx_b = pipe_layer_index[pipe_b.pipe_id]
                pipe_dist = abs(idx_a - idx_b)
                switch_time = 1.5 + 0.2 * pipe_dist
                pos_a_3d = pipe_a.start_pt + pipe_a.direction * sa
                pos_b_3d = pipe_b.start_pt + pipe_b.direction * sb
                self._add_ramp(
                    pipe_a, pipe_b, pos_a_3d, pos_b_3d, switch_time, True, "intra_layer"
                )
                existing_ramp_pos[pipe_a.pipe_id].append(sa)
                existing_ramp_pos[pipe_b.pipe_id].append(sb)
                layer_count += 1
                total_count += 1
            if layer_count < per_layer_target:
                print(f"警告：{height}m 层同层动态匝道只生成 {layer_count}/{per_layer_target} 条")
        return total_count

    def _intra_dynamic_per_layer_target(self):
        if self.intra_dynamic_level == "none":
            return 0
        factor = {"low": 1, "medium": 2, "high": 4, "ultra": 8}[
            self.intra_dynamic_level
        ]
        return min(factor * self.scale_config.num_pipes_per_layer, 480)

    def _build_ramps(self):
        """生成区域内稀疏静态骨架 + 动态匝道，再生成跨区域边界匝道。"""
        if self.topology_profile.name == "metropolis_partition_v1":
            self._build_partition_ramps()
            return
        import random as _rnd

        rng = _rnd.Random(self.random_seed)
        self.ramps = []
        self._ramp_positions_during_build = {
            pipe.pipe_id: [] for pipe in self.pipelines
        }
        total_stats = {
            "intra_static": 0,
            "inter_static": 0,
            "inter_static_pairs": 0,
            "inter_dynamic": 0,
            "inter_dynamic_pairs": 0,
            "intra_dynamic": 0,
            "cross_region": 0,
        }
        for region in self.regions:
            stats = self._build_internal_ramps_for_region(region["region_id"], rng)
            for key, value in stats.items():
                total_stats[key] += value
        total_stats["cross_region"] = self._build_cross_region_ramps()
        self._normalize_reference_topology(total_stats)
        self.ramp_stats = total_stats
        self._initialize_ramp_indexes()
        del self._ramp_positions_during_build
        print(
            f"匝道构建完成：同层静态 {total_stats['intra_static']} 条 | 层间静态 {total_stats['inter_static']} 条 | 跨层动态 {total_stats['inter_dynamic']} 条 | 同层动态 {total_stats['intra_dynamic']} 条 | 跨区域同层 {total_stats['cross_region']} 条 | 总计 {len(self.ramps)} 条"
        )

    def _build_partition_ramps(self):
        """Bounded extra choices on the unchanged six-layer physical geometry.

        End-to-start rings retain directed reachability even after all dynamic
        ramps close. Interior neighbor links, matched adjacent-layer links and
        sampled express links add choices throughout the original long pipes.
        """
        self.ramps = []
        layers = {}
        for pipe in self.pipelines:
            layers.setdefault(pipe.height, []).append(pipe)
        stats = dict(intra_static=0, inter_static=0, inter_static_pairs=0,
                     inter_dynamic=0, inter_dynamic_pairs=0, intra_dynamic=0,
                     cross_region=0)
        # Match the legacy density labels, but bound the extra road count by
        # eight choices per source. Increasing density preserves earlier
        # choices and the entire static backbone.
        quotas = {"none": 0, "low": 1, "medium": 2, "high": 4, "ultra": 8}
        intra_quota = quotas[self.intra_dynamic_level] if self.include_dynamic else 0
        inter_quota = quotas[self.inter_dynamic_level] if self.include_dynamic else 0

        def add(source, target, source_dist, target_dist, kind, dynamic=False):
            self._add_ramp(source, target, self._position_on_pipe(source, source_dist),
                           self._position_on_pipe(target, target_dist), None,
                           dynamic, kind)
            key = ("intra_" if kind == "intra_layer" else "inter_") + ("dynamic" if dynamic else "static")
            stats[key] += 1

        for pipes in layers.values():
            n = len(pipes)
            if n < 2:
                continue
            for index, source in enumerate(pipes):
                target = pipes[(index + 1) % n]
                add(source, target, source.length, 0., "intra_layer")
                for fraction in np.linspace(.2, .8, self.ramp_same_layer_degree):
                    target_fraction = fraction if np.dot(source.direction, target.direction) > 0 else 1. - fraction
                    add(source, target, source.length * fraction,
                        target.length * target_fraction, "intra_layer")
                for choice in range(intra_quota):
                    # Cycle through other pipes, using different longitudinal
                    # anchors if a small layer exhausts distinct targets.
                    offset = 1 + (choice + 1) % (n - 1)
                    dynamic_target = pipes[(index + offset) % n]
                    add(source, dynamic_target, source.length * (.55 + .02 * choice),
                        dynamic_target.length * (.45 - .02 * choice), "intra_layer", True)

        heights = sorted(layers)
        for left, height in enumerate(heights):
            for right in range(left + 1, len(heights)):
                source_pipes, target_pipes = layers[height], layers[heights[right]]
                adjacent = right == left + 1
                indexes = range(len(source_pipes)) if adjacent else range(0, len(source_pipes), 5)
                for index in indexes:
                    source = source_pipes[index]
                    degree = self.ramp_adjacent_layer_degree if adjacent else self.ramp_express_layer_degree
                    base_index = min(len(target_pipes) - 1, index * len(target_pipes) // len(source_pipes))
                    for offset in range(min(degree, len(target_pipes))):
                        target = target_pipes[(base_index + offset) % len(target_pipes)]
                        sa, sb, _ = self._closest_points_between_pipes_2d(source, target)
                        add(source, target, sa, sb, "inter_layer")
                        # Independent endpoints keep opposite directions distinct.
                        reverse_s = min(target.length, sb + 60.)
                        reverse_t = max(0., sa - 60.)
                        add(target, source, reverse_s, reverse_t, "inter_layer")
                        stats["inter_static_pairs"] += 1
                    if adjacent:
                        for choice in range(inter_quota):
                            alternate = target_pipes[(base_index + 1 + choice) % len(target_pipes)]
                            add(source, alternate, source.length * (.4 + .02 * choice),
                                alternate.length * (.6 - .02 * choice), "inter_layer", True)
                            add(alternate, source, alternate.length * (.65 - .02 * choice),
                                source.length * (.35 + .02 * choice), "inter_layer", True)
                            stats["inter_dynamic_pairs"] += 1
        self.ramp_stats = stats
        self._initialize_ramp_indexes()

    def _initialize_ramp_indexes(self):
        self.ramp_by_id = {int(ramp.ramp_id): ramp for ramp in self.ramps}
        self.directed_ramp_index = {}
        self.directed_ramps_by_pipe_pair = {}
        for ramp in self.ramps:
            directions = [(ramp.source_pipe_id, ramp.target_pipe_id)]
            if ramp.is_bidirectional:
                directions.append((ramp.target_pipe_id, ramp.source_pipe_id))
            for source_pipe_id, target_pipe_id in directions:
                key = (int(source_pipe_id), int(target_pipe_id))
                self.directed_ramp_index[key[0], key[1], int(ramp.ramp_id)] = int(
                    ramp.ramp_id
                )
                self.directed_ramps_by_pipe_pair.setdefault(key, []).append(
                    int(ramp.ramp_id)
                )

    def _normalize_reference_topology(self, total_stats):
        target = int(getattr(self.topology_profile, "reference_total_ramps", 0))
        if not (
            target > 0
            and self.airspace_mode == "regional_6x6"
            and self.include_dynamic
            and (self.inter_dynamic_level == "low")
            and (self.intra_dynamic_level == "low")
        ):
            return
        missing = target - len(self.ramps)
        if missing > 0:
            added = self._add_reference_dynamic_inter_ramps(missing)
            total_stats["inter_dynamic"] += added
            total_stats["inter_dynamic_pairs"] += (
                added // 2 if self.topology_profile.directed_inter_layer else added
            )
            if added != missing:
                raise RuntimeError(f"旧版稠密拓扑无法补齐到 {target} 条匝道：仅补充 {added}/{missing} 条")
        elif missing < 0:
            self._remove_reference_dynamic_ramps(-missing, total_stats)
        for ramp_id, ramp in enumerate(self.ramps):
            ramp.ramp_id = ramp_id
        if len(self.ramps) != target:
            raise RuntimeError(f"旧版稠密拓扑数量校准失败：{len(self.ramps)} != {target}")

    def _add_reference_dynamic_inter_ramps(self, count):
        existing_pairs = {
            self._pair_key(
                self.pipelines[ramp.pipe_a_id], self.pipelines[ramp.pipe_b_id]
            )
            for ramp in self.ramps
        }
        added = 0
        for region_id in sorted(self._pipes_by_region):
            layer_pipes = {}
            for pipe in self._pipes_by_region[region_id]:
                layer_pipes.setdefault(pipe.height, []).append(pipe)
            for candidate in self._inter_layer_candidates(layer_pipes):
                key = self._pair_key(candidate["pipe_a"], candidate["pipe_b"])
                if key in existing_pairs:
                    continue
                ramps = self._add_inter_ramp_from_candidate(
                    candidate,
                    is_dynamic=True,
                    existing_ramp_pos=self._ramp_positions_during_build,
                )
                existing_pairs.add(key)
                added += len(ramps)
                if added >= int(count):
                    return added
        return added

    def _remove_reference_dynamic_ramps(self, count, total_stats):
        removable = [
            index
            for index in range(len(self.ramps) - 1, -1, -1)
            if self.ramps[index].is_dynamic
            and (not getattr(self.ramps[index], "is_cross_region", False))
        ]
        if len(removable) < int(count):
            raise RuntimeError(f"旧版稠密拓扑缺少可裁剪动态匝道：{len(removable)} < {count}")
        for index in sorted(removable[: int(count)], reverse=True):
            ramp = self.ramps.pop(index)
            stats_key = (
                "intra_dynamic" if ramp.ramp_type == "intra_layer" else "inter_dynamic"
            )
            total_stats[stats_key] -= 1
            if ramp.ramp_type == "inter_layer":
                total_stats["inter_dynamic_pairs"] -= 1

    def _build_internal_ramps_for_region(self, region_id, rng):
        region_pipes = self._pipes_by_region.get(region_id, [])
        layer_pipes = {}
        pipe_layer_index = {}
        for pipe in region_pipes:
            group = layer_pipes.setdefault(pipe.height, [])
            pipe_layer_index[pipe.pipe_id] = len(group)
            group.append(pipe)
        intra_static_count = self._build_static_intra_ramps(layer_pipes)
        inter_static_count = self._build_static_inter_ramps(layer_pipes)
        inter_static_pairs = (
            inter_static_count // 2
            if self.topology_profile.directed_inter_layer
            else inter_static_count
        )
        existing_inter_pairs = {
            self._pair_key(self.pipelines[r.pipe_a_id], self.pipelines[r.pipe_b_id])
            for r in self.ramps
            if r.ramp_type == "inter_layer"
        }
        stats = {
            "intra_static": intra_static_count,
            "inter_static": inter_static_count,
            "inter_static_pairs": inter_static_pairs,
            "inter_dynamic": 0,
            "inter_dynamic_pairs": 0,
            "intra_dynamic": 0,
        }
        existing_ramp_pos = self._record_ramp_positions()
        inter_dynamic_count = self._build_dynamic_inter_ramps(
            layer_pipes,
            existing_inter_pairs,
            static_total=intra_static_count + inter_static_pairs,
        )
        existing_ramp_pos = self._record_ramp_positions()
        intra_dynamic_count = self._build_dynamic_intra_ramps(
            layer_pipes, pipe_layer_index, existing_ramp_pos, rng
        )
        stats["inter_dynamic"] = inter_dynamic_count
        stats["inter_dynamic_pairs"] = (
            inter_dynamic_count // 2
            if self.topology_profile.directed_inter_layer
            else inter_dynamic_count
        )
        stats["intra_dynamic"] = intra_dynamic_count
        return stats

    def _region_by_coord(self):
        return {(r["row"], r["col"]): r for r in self.regions}

    def _boundary_endpoint(self, pipe, side):
        endpoints = [pipe.start_pt, pipe.end_pt]
        if side == "east":
            return max(endpoints, key=lambda pt: pt[0])
        if side == "west":
            return min(endpoints, key=lambda pt: pt[0])
        if side == "north":
            return min(endpoints, key=lambda pt: pt[1])
        if side == "south":
            return max(endpoints, key=lambda pt: pt[1])
        return endpoints[0]

    def _add_cross_region_ramp_pair(self, region_a, region_b, side_a, side_b):
        count = 0
        for layer in self.layer_configs:
            height = layer["height"]
            selected_pairs = self._select_cross_region_pipe_pairs(
                region_a,
                region_b,
                height,
                side_a,
                side_b,
                self.topology_profile.cross_region_portals_per_layer,
            )
            for pipe_a, pipe_b, pos_a, pos_b, distance in selected_pairs:
                switch_time = max(
                    1.0, distance / max(1.0, min(pipe_a.max_speed, pipe_b.max_speed))
                )
                self._add_ramp(
                    pipe_a,
                    pipe_b,
                    pos_a,
                    pos_b,
                    switch_time,
                    False,
                    "cross_region_intra_layer",
                    is_cross_region=True,
                    boundary_side=f"{side_a}_to_{side_b}",
                    is_bidirectional=True,
                )
                count += 1
        return count

    def _select_cross_region_pipe_pairs(
        self, region_a, region_b, height, side_a, side_b, count
    ):
        pipes_a = [
            self._pipe_lookup_by_region_layer.get(
                (region_a["region_id"], height, local_idx)
            )
            for local_idx in range(self.scale_config.num_pipes_per_layer)
        ]
        pipes_b = [
            self._pipe_lookup_by_region_layer.get(
                (region_b["region_id"], height, local_idx)
            )
            for local_idx in range(self.scale_config.num_pipes_per_layer)
        ]
        candidates = []
        for pipe_a in pipes_a:
            if pipe_a is None:
                continue
            pos_a = self._boundary_endpoint(pipe_a, side_a)
            for pipe_b in pipes_b:
                if pipe_b is None:
                    continue
                pos_b = self._boundary_endpoint(pipe_b, side_b)
                distance = float(np.linalg.norm(pos_a - pos_b))
                candidates.append(
                    (
                        distance,
                        int(getattr(pipe_a, "local_layer_index", 0)),
                        int(getattr(pipe_b, "local_layer_index", 0)),
                        pipe_a,
                        pipe_b,
                        pos_a,
                        pos_b,
                    )
                )
        candidates.sort(key=lambda item: item[:3])
        selected = []
        used_a = set()
        used_b = set()
        for distance, _idx_a, _idx_b, pipe_a, pipe_b, pos_a, pos_b in candidates:
            if pipe_a.pipe_id in used_a or pipe_b.pipe_id in used_b:
                continue
            selected.append((pipe_a, pipe_b, pos_a, pos_b, distance))
            used_a.add(pipe_a.pipe_id)
            used_b.add(pipe_b.pipe_id)
            if len(selected) >= int(count):
                break
        return selected

    def _build_cross_region_ramps(self):
        if self.region_rows <= 1 and self.region_cols <= 1:
            return 0
        regions = self._region_by_coord()
        count = 0
        for row in range(self.region_rows):
            for col in range(self.region_cols):
                region = regions[row, col]
                if col + 1 < self.region_cols:
                    count += self._add_cross_region_ramp_pair(
                        region, regions[row, col + 1], "east", "west"
                    )
                if row + 1 < self.region_rows:
                    count += self._add_cross_region_ramp_pair(
                        region, regions[row + 1, col], "south", "north"
                    )
        return count

    def visualize(self):
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection="3d")
        colors = {
            100: "#4CAF50",
            200: "#00BCD4",
            300: "#2196F3",
            400: "#9C27B0",
            500: "#607D8B",
            600: "#FF9800",
        }
        for pipe in self.pipelines:
            c = colors.get(pipe.height, "black")
            ax.plot(
                [pipe.start_pt[0], pipe.end_pt[0]],
                [pipe.start_pt[1], pipe.end_pt[1]],
                [pipe.start_pt[2], pipe.end_pt[2]],
                color=c,
                alpha=0.5,
                linewidth=2,
            )
            mid_pt = (pipe.start_pt + pipe.end_pt) / 2
            dir_vec = pipe.direction * 300
            ax.quiver(
                mid_pt[0],
                mid_pt[1],
                mid_pt[2],
                dir_vec[0],
                dir_vec[1],
                dir_vec[2],
                color=c,
                length=1.0,
                normalize=True,
            )
        for ramp in self.ramps:
            if ramp.is_dynamic:
                c, ls, a = ("#800080", "--", 0.5)
            elif ramp.ramp_type == "intra_layer":
                c, ls, a = ("#E91E63", "-", 0.4)
            else:
                c, ls, a = ("gray", "--", 0.3)
            ax.plot(
                [ramp.pos_a[0], ramp.pos_b[0]],
                [ramp.pos_a[1], ramp.pos_b[1]],
                [ramp.pos_a[2], ramp.pos_b[2]],
                color=c,
                linestyle=ls,
                alpha=a,
                linewidth=1,
            )
        ax.set_xlim(self.x_min, self.x_max)
        ax.set_ylim(self.y_min, self.y_max)
        ax.set_zlim(self.z_min, self.z_max)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Height Z (m)")
        ax.set_title("UAV Airspace Pipeline Network with Ramps")
        from matplotlib.lines import Line2D

        legend_elements = [
            Line2D([0], [0], color="#4CAF50", lw=2, label="Layer 1 (100m, 0°)"),
            Line2D([0], [0], color="#00BCD4", lw=2, label="Layer 2 (200m, 30°)"),
            Line2D([0], [0], color="#2196F3", lw=2, label="Layer 3 (300m, 60°)"),
            Line2D([0], [0], color="#9C27B0", lw=2, label="Layer 4 (400m, 90°)"),
            Line2D([0], [0], color="#607D8B", lw=2, label="Layer 5 (500m, 120°)"),
            Line2D([0], [0], color="#FF9800", lw=2, label="Layer 6 (600m, 150°)"),
            Line2D(
                [0],
                [0],
                color="gray",
                linestyle="--",
                lw=1,
                label="Inter-layer Ramp (Static)",
            ),
            Line2D(
                [0],
                [0],
                color="#E91E63",
                linestyle="-",
                lw=1,
                label="Intra-layer Ramp (Static)",
            ),
            Line2D(
                [0], [0], color="#800080", linestyle="--", lw=1, label="Dynamic Ramp"
            ),
        ]
        ax.legend(handles=legend_elements, loc="upper left")
        plt.show()

    def _build_routing_graph(self):
        self.graph = {}
        self.node_info = {}
        pipe_nodes = {p.pipe_id: [] for p in self.pipelines}
        for p in self.pipelines:
            s_name, e_name = (f"P{p.pipe_id}_START", f"P{p.pipe_id}_END")
            pipe_nodes[p.pipe_id].append((s_name, 0.0, "START"))
            self.node_info[s_name] = self._node_info_for_pipe(p, 0.0, "START")
            pipe_nodes[p.pipe_id].append((e_name, p.length, "END"))
            self.node_info[e_name] = self._node_info_for_pipe(p, p.length, "END")
        for r in self.ramps:
            na_name, nb_name = (
                f"P{r.pipe_a_id}_R{r.ramp_id}",
                f"P{r.pipe_b_id}_R{r.ramp_id}",
            )
            pipe_nodes[r.pipe_a_id].append((na_name, r.dist_from_start_a, "RAMP"))
            self.node_info[na_name] = self._node_info_for_pipe(
                self.pipelines[r.pipe_a_id], r.dist_from_start_a, "RAMP"
            )
            pipe_nodes[r.pipe_b_id].append((nb_name, r.dist_from_start_b, "RAMP"))
            self.node_info[nb_name] = self._node_info_for_pipe(
                self.pipelines[r.pipe_b_id], r.dist_from_start_b, "RAMP"
            )
        for p in self.pipelines:
            nodes = sorted(pipe_nodes[p.pipe_id], key=lambda x: x[1])
            pipe_nodes[p.pipe_id] = nodes
            for i in range(len(nodes) - 1):
                curr_name, curr_dist, _ = nodes[i]
                next_name, next_dist, _ = nodes[i + 1]
                if curr_name not in self.graph:
                    self.graph[curr_name] = []
                if next_name not in self.graph:
                    self.graph[next_name] = []
                self.graph[curr_name].append(
                    (next_name, (next_dist - curr_dist) / p.max_speed, "FLY", None)
                )
                if (self.topology_profile.name == "metropolis_partition_v1"
                        and abs(next_dist - curr_dist) <= 1.e-7):
                    self.graph[next_name].append((curr_name, 0., "FLY", None))
        for source_pipe_id, target_pipe_id, _external_node_id, edge_type in getattr(
            self, "continuation_edges", ()
        ):
            if edge_type != "CONTINUE":
                continue
            source_node = f"P{int(source_pipe_id)}_END"
            target_node = f"P{int(target_pipe_id)}_START"
            self.graph.setdefault(source_node, [])
            self.graph.setdefault(target_node, [])
            self.graph[source_node].append((target_node, 0.0, "CONTINUE", None))
        for r in self.ramps:
            na_name = f"P{r.pipe_a_id}_R{r.ramp_id}"
            nb_name = f"P{r.pipe_b_id}_R{r.ramp_id}"
            if na_name not in self.graph:
                self.graph[na_name] = []
            if nb_name not in self.graph:
                self.graph[nb_name] = []
            self.graph[na_name].append((nb_name, r.time_penalty, "SWITCH", r.ramp_id))
            if r.is_bidirectional:
                self.graph[nb_name].append(
                    (na_name, r.time_penalty, "SWITCH", r.ramp_id)
                )
        self.pipe_nodes = pipe_nodes

    def path_transition_types(self, path):
        transition_types = []
        for source_node, target_node in zip(path or (), (path or ())[1:]):
            matches = [
                edge_type
                for neighbor, _weight, edge_type, _ramp_id in self.graph.get(
                    source_node, ()
                )
                if neighbor == target_node
            ]
            if matches:
                transition_types.append(matches[0])
        return transition_types

    @staticmethod
    def _node_info_for_pipe(pipe, dist, node_type):
        return {
            "pipe_id": pipe.pipe_id,
            "dist": float(dist),
            "type": node_type,
            "region_id": getattr(pipe, "region_id", 0),
            "region_row": getattr(pipe, "region_row", 0),
            "region_col": getattr(pipe, "region_col", 0),
            "local_pipe_id": getattr(pipe, "local_pipe_id", pipe.pipe_id),
            "local_layer_index": getattr(pipe, "local_layer_index", 0),
        }

    def edge_cost(self, source_node, target_node, uav_speed=None):
        for neighbor, weight, edge_type, ramp_id in self.graph.get(source_node, ()):
            if neighbor != target_node:
                continue
            if edge_type == "SWITCH" and ramp_id is not None:
                return switch_edge_cost(self, ramp_id, uav_speed)
            return float(weight)
        raise KeyError("不存在路由边: {} -> {}".format(source_node, target_node))

    def _get_edges_for_node(
        self,
        curr_node,
        start_pipe,
        start_pos,
        dest_pipe,
        dest_pos,
        dest_3d,
        uav_speed=None,
    ):
        edges = []
        if curr_node == "V_START":
            if start_pipe == dest_pipe and start_pos <= dest_pos:
                fly_time = (
                    (dest_pos - start_pos)
                    * self.pipe_collisions_multiplier.get(start_pipe, 1.0)
                    / self.pipelines[start_pipe].max_speed
                )
                edges.append(("V_END", fly_time, "FLY", start_pipe, dest_3d))
            for n_name, n_dist, _node_type in self.pipe_nodes.get(start_pipe, []):
                if n_dist >= start_pos:
                    if start_pipe == dest_pipe and n_dist > dest_pos:
                        continue
                    fly_time = (
                        (n_dist - start_pos)
                        * self.pipe_collisions_multiplier.get(start_pipe, 1.0)
                        / self.pipelines[start_pipe].max_speed
                    )
                    edges.append(
                        (
                            n_name,
                            fly_time,
                            "FLY",
                            start_pipe,
                            self.pipelines[start_pipe].start_pt
                            + self.pipelines[start_pipe].direction * n_dist,
                        )
                    )
        else:
            curr_info = self.node_info[curr_node]
            curr_pipe = curr_info["pipe_id"]
            if curr_pipe == dest_pipe and curr_info["dist"] <= dest_pos:
                fly_time = (
                    (dest_pos - curr_info["dist"])
                    * self.pipe_collisions_multiplier.get(dest_pipe, 1.0)
                    / self.pipelines[dest_pipe].max_speed
                )
                edges.append(("V_END", fly_time, "FLY", dest_pipe, dest_3d))
            for neighbor, weight, edge_type, ramp_id in self.graph.get(curr_node, []):
                if (
                    edge_type == "SWITCH"
                    and ramp_id is not None
                    and (not self.ramps[ramp_id].is_active)
                ):
                    continue
                if neighbor not in ["V_END", "V_START"]:
                    n_info = self.node_info[neighbor]
                    if n_info["pipe_id"] == dest_pipe and n_info["dist"] > dest_pos:
                        continue
                    target_pipe = n_info["pipe_id"]
                    n_coords = (
                        self.pipelines[target_pipe].start_pt
                        + self.pipelines[target_pipe].direction * n_info["dist"]
                    )
                else:
                    n_coords, target_pipe = (dest_3d, dest_pipe)
                if edge_type == "SWITCH" and ramp_id is not None:
                    actual_weight = switch_edge_cost(self, ramp_id, uav_speed)
                else:
                    actual_weight = weight * self.pipe_collisions_multiplier.get(
                        curr_pipe, 1.0
                    )
                edges.append(
                    (neighbor, actual_weight, edge_type, target_pipe, n_coords)
                )
        return edges

    def find_shortest_path(
        self, start_pipe_id, start_pos, dest_pipe_id, dest_pos, uav_speed=None
    ):
        start_node = "V_START"
        end_node = "V_END"
        dest_pipe_obj = self.pipelines[dest_pipe_id]
        dest_3d = dest_pipe_obj.start_pt + dest_pipe_obj.direction * dest_pos
        pq = [(0.0, start_node, [start_node])]
        visited = set()
        while pq:
            cost, curr_node, path = heapq.heappop(pq)
            if curr_node in visited:
                continue
            visited.add(curr_node)
            if curr_node == end_node:
                return path
            edges = self._get_edges_for_node(
                curr_node,
                start_pipe_id,
                start_pos,
                dest_pipe_id,
                dest_pos,
                dest_3d,
                uav_speed=uav_speed,
            )
            for neighbor, actual_weight, edge_type, target_pipe, n_coords in edges:
                if neighbor not in visited:
                    heapq.heappush(
                        pq, (cost + actual_weight, neighbor, path + [neighbor])
                    )
        return []

    @staticmethod
    def _extract_ramp_id_from_node(node_name):
        if "_R" in node_name:
            try:
                return int(node_name.split("_R")[1])
            except (ValueError, IndexError):
                return None
        return None

    def find_directed_ramp_id(self, source_pipe_id, target_pipe_id, target_node=None):
        source_pipe_id = int(source_pipe_id)
        target_pipe_id = int(target_pipe_id)
        ramp_id = self._extract_ramp_id_from_node(target_node or "")
        if target_node is not None and ramp_id is None:
            return None
        if ramp_id is not None:
            ramp = self.ramp_by_id.get(int(ramp_id))
            if ramp is not None:
                forward = (
                    int(ramp.source_pipe_id) == source_pipe_id
                    and int(ramp.target_pipe_id) == target_pipe_id
                )
                reverse = (
                    bool(ramp.is_bidirectional)
                    and int(ramp.target_pipe_id) == source_pipe_id
                    and (int(ramp.source_pipe_id) == target_pipe_id)
                )
                if forward or reverse:
                    return int(ramp.ramp_id)
            return None
        candidates = self.directed_ramps_by_pipe_pair.get(
            (source_pipe_id, target_pipe_id), ()
        )
        return int(candidates[0]) if len(candidates) == 1 else None

    def get_readonly_snapshot(self):
        """返回可被 ray.put() / pickle 序列化的只读快照数据"""
        return {
            "pipelines": self.pipelines,
            "ramps": self.ramps,
            "graph": self.graph,
            "node_info": dict(self.node_info),
            "pipe_nodes": dict(self.pipe_nodes),
            "layer_configs": self.layer_configs,
            "x_min": self.x_min,
            "x_max": self.x_max,
            "y_min": self.y_min,
            "y_max": self.y_max,
            "z_min": self.z_min,
            "z_max": self.z_max,
            "airspace_mode": self.airspace_mode,
            "xy_span": self.xy_span,
            "xy_diagonal": self.xy_diagonal,
            "z_span": self.z_span,
            "base_station_pos": self.base_station_pos.tolist(),
            "regions": list(self.regions),
            "region_rows": self.region_rows,
            "region_cols": self.region_cols,
            "region_side_length": self.region_side_length,
            "num_lanes_per_pipe": self.num_lanes_per_pipe,
            "topology_profile": self.topology_profile.name,
            "topology_source": getattr(self, "topology_source", "generated"),
            "topology_version": int(getattr(self, "topology_version", 0)),
            "external_id_maps": getattr(self, "external_id_maps", None),
            "continuation_edges": list(getattr(self, "continuation_edges", ())),
        }
