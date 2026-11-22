"""
gui_config.py — PyQt5 可视化交互配置界面
用户在此界面中设定无人机数量、运力分配、中间节点、终点、算法等参数，
确认后生成 config 字典传给 main.py 驱动仿真。
"""
import math
import numpy as np
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QSplitter,
    QScrollArea,
    QGroupBox,
    QSpinBox,
    QDoubleSpinBox,
    QSlider,
    QLabel,
    QCheckBox,
    QComboBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QMessageBox,
    QGridLayout,
    QSizePolicy,
    QFormLayout,
    QLineEdit,
)
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from uav_sim.algorithms.registry import ALGORITHM_REGISTRY
from uav_sim.airspace.config import get_airspace_config
from uav_sim.distributed.engine import DEFAULT_UAVS_PER_ACTOR
from uav_sim.guidance import GuidanceConfig, PARAMETERS

LAYER_COLORS = {
    100: "#4CAF50",
    200: "#00BCD4",
    300: "#2196F3",
    400: "#9C27B0",
    500: "#607D8B",
    600: "#FF9800",
}
DYNAMIC_LEVEL_OPTIONS = [
    ("无", "none"),
    ("低", "low"),
    ("中", "medium"),
    ("高", "high"),
    ("超高", "ultra"),
]
TOPOLOGY_PROFILE_OPTIONS = [
    ("方案 B 稀疏单向拓扑（静态 5400 匝道 / 15120 节点，默认）", "balanced_sparse_v1"),
    ("旧版稠密逻辑连接（单向实体 v2）", "legacy_dense"),
    ("大都会原始管道 / 36 管理区拓扑", "metropolis_partition_v1"),
]
ROUTING_MODE_OPTIONS = [
    ("模式一：服务器完整地图集中寻路", "centralized_full"),
    ("模式二：服务器按区域加载寻路", "centralized_regional"),
    ("模式三：机载区域地图寻路（Ray）", "onboard_regional"),
]


class AirspaceCanvas(FigureCanvasQTAgg):
    def __init__(self, env, parent=None):
        self.fig = Figure(figsize=(7, 6))
        super().__init__(self.fig)
        self.env = env
        self.dest_pipe_ids = set()
        self.pickup_markers = []
        self._last_preview_key = None
        self.setMinimumSize(500, 400)

    def refresh(self, dest_pipe_ids, pickup_markers, destination_markers=None):
        destination_markers = destination_markers or []
        key = (id(self.env), tuple(sorted(dest_pipe_ids)),
               tuple((mk["pipe_id"], mk["pos_1d"]) for mk in pickup_markers),
               tuple((mk["pipe_id"], mk["pos_1d"], mk.get("lane_id")) for mk in destination_markers))
        if key == self._last_preview_key:
            return
        self._last_preview_key = key
        self.dest_pipe_ids = set(dest_pipe_ids)
        self.pickup_markers = pickup_markers
        self.fig.clear()
        ax = self.fig.add_subplot(111, projection="3d")
        for height, color in LAYER_COLORS.items():
            segments = [(pipe.start_pt, pipe.end_pt) for pipe in self.env.pipelines
                        if pipe.height == height]
            if segments:
                ax.add_collection3d(Line3DCollection(segments, colors=color,
                                                    alpha=0.4, linewidths=1.1))
        # A bounded centerline sample keeps the original preview responsive.
        ramps = self.env.ramps
        stride = max(1, math.ceil(len(ramps) / 600))
        if ramps:
            ax.add_collection3d(Line3DCollection(
                [(r.pos_a, r.pos_b) for r in ramps[::stride]], colors="gray",
                alpha=0.18, linewidths=0.6))
        if getattr(self.env, "airspace_mode", "") in ("metropolis", "regional_6x6"):
            grid = []
            for fraction in np.linspace(0, 1, 7):
                x = self.env.x_min + fraction * (self.env.x_max - self.env.x_min)
                y = self.env.y_min + fraction * (self.env.y_max - self.env.y_min)
                grid.extend([((x, self.env.y_min, 0), (x, self.env.y_max, 0)),
                             ((self.env.x_min, y, 0), (self.env.x_max, y, 0))])
            ax.add_collection3d(Line3DCollection(grid, colors="#37474F",
                                                alpha=0.35, linewidths=0.7))
        pipe_lookup = {pipe.pipe_id: pipe for pipe in self.env.pipelines}
        for pid in self.dest_pipe_ids:
            pipe = pipe_lookup[pid]
            ep = pipe.end_pt
            ax.scatter(ep[0], ep[1], ep[2], c="red", marker="X", s=120, zorder=5)
        for mk in self.pickup_markers:
            pipe = pipe_lookup[mk["pipe_id"]]
            pos_3d = pipe.start_pt + pipe.direction * mk["pos_1d"]
            ax.scatter(
                pos_3d[0], pos_3d[1], pos_3d[2], c="purple", marker="*", s=200, zorder=5
            )
        for mk in destination_markers:
            pipe = pipe_lookup[mk["pipe_id"]]
            position = pipe.start_pt + pipe.direction * mk["pos_1d"]
            lanes = getattr(pipe, "lanes", ())
            lane_id = mk.get("lane_id")
            if lane_id is not None and 0 <= lane_id < len(lanes):
                fraction = mk["pos_1d"] / float(pipe.length)
                lane = lanes[lane_id]
                position = lane.start_pt + (lane.end_pt - lane.start_pt) * fraction
            ax.scatter(*position, c="red", marker="X", s=70, zorder=5)
        ax.set_xlim(self.env.x_min, self.env.x_max)
        ax.set_ylim(self.env.y_min, self.env.y_max)
        ax.set_zlim(self.env.z_min, self.env.z_max + 100)
        ax.set_xlabel("X (m)", fontsize=9)
        ax.set_ylabel("Y (m)", fontsize=9)
        ax.set_zlabel("Z (m)", fontsize=9)
        ax.set_title("Airspace Preview", fontsize=11)
        ax.view_init(elev=25, azim=-45)
        self.fig.tight_layout()
        self.draw()


class PickupRow(QWidget):
    def __init__(self, index, env, on_change_cb, parent=None):
        super().__init__(parent)
        self.env = env
        self.on_change_cb = on_change_cb
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.addWidget(QLabel(f"节点{index + 1}:"))
        self.pipe_combo = QComboBox()
        for p in env.pipelines:
            h = p.height
            angle = [lc["angle"] for lc in env.layer_configs if lc["height"] == h][0]
            region_id = getattr(p, "region_id", 0)
            row = getattr(p, "region_row", 0)
            col = getattr(p, "region_col", 0)
            local_id = getattr(p, "local_pipe_id", p.pipe_id)
            self.pipe_combo.addItem(
                f"区域{region_id}({row},{col}) | 管道{p.pipe_id}/本区{local_id} ({h}m, {angle}°)",
                p.pipe_id,
            )
        self.pipe_combo.currentIndexChanged.connect(self._on_pipe_changed)
        layout.addWidget(self.pipe_combo)
        layout.addWidget(QLabel("位置:"))
        self.pos_spin = QDoubleSpinBox()
        self.pos_spin.setDecimals(1)
        self.pos_spin.setSingleStep(100.0)
        self._update_pos_range()
        self.pos_spin.valueChanged.connect(lambda: self.on_change_cb())
        layout.addWidget(self.pos_spin)

    def _on_pipe_changed(self):
        self._update_pos_range()
        self.on_change_cb()

    def _update_pos_range(self):
        pid = self.pipe_combo.currentData()
        if pid is not None:
            pipe_len = self.env.pipelines[pid].length
            self.pos_spin.setRange(0.0, pipe_len)
            self.pos_spin.setValue(min(pipe_len / 2.0, self.pos_spin.value()))

    def get_location(self):
        return {
            "pipe_id": self.pipe_combo.currentData(),
            "pos_1d": self.pos_spin.value(),
        }


class ConfigGUI(QMainWindow):
    def __init__(self, env):
        super().__init__()
        self._is_init = True
        self.env = env
        self._db_preview_ready = getattr(env, "topology_source", "generated") == "postgres"
        self.config = None
        self.pickup_rows = []
        self.preview_includes_dynamic = bool(getattr(env,'include_dynamic',False))
        self.setWindowTitle("UAV 空域仿真 — 参数配置面板")
        self.setMinimumSize(1200, 800)
        self._build_ui()
        self._is_init = False
        self._apply_runtime_compatibility()
        self._on_params_changed()
        self._refresh_preview()

    def run(self):
        self.show()
        app = QApplication.instance()
        app.exec_()
        return self.config

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        splitter = QSplitter(Qt.Horizontal)
        self.settings_splitter = splitter
        main_layout.addWidget(splitter)
        scroll = QScrollArea()
        self.settings_scroll = scroll
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(420)
        scroll.setMaximumWidth(820)
        scroll.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        panel = QWidget()
        self.panel_layout = QVBoxLayout(panel)
        scroll.setWidget(panel)
        splitter.addWidget(scroll)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        self.canvas = AirspaceCanvas(self.env)
        right_layout.addWidget(self.canvas, 1)
        self.info_label = QLabel("总运力: 0 | 批次数: 0")
        self.info_label.setStyleSheet(
            "font-size: 13px; font-weight: bold; padding: 5px;"
        )
        self.info_label.setWordWrap(True)
        self.info_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        right_layout.addWidget(self.info_label)
        splitter.addWidget(right_widget)
        splitter.setChildrenCollapsible(False)
        splitter.setSizes([560, 1000])
        self._build_basic_group()
        self._build_runtime_group()
        self._build_guidance_group()
        self._build_capacity_group()
        self._build_pickup_group()
        self._build_dest_group()
        self._build_algo_group()
        self._build_advanced_group()
        self._build_parallel_group()
        self._build_buttons()
        self.panel_layout.addStretch()

    def _build_basic_group(self):
        grp = QGroupBox("基础参数")
        layout = QFormLayout()
        self.airspace_source_combo = QComboBox()
        self.airspace_source_combo.addItem("程序生成（默认）", "generated")
        self.airspace_source_combo.addItem("PostgreSQL 数据库", "postgres")
        initial_source = getattr(self.env, "topology_source", "generated")
        source_index = self.airspace_source_combo.findData(initial_source)
        if source_index >= 0:
            self.airspace_source_combo.setCurrentIndex(source_index)
        self.airspace_source_combo.currentIndexChanged.connect(
            self._on_airspace_source_changed
        )
        layout.addRow("空域数据源:", self.airspace_source_combo)
        self.database_connection_label = QLabel("数据库连接信息来自环境变量，不在 GUI 中保存密码")
        self.database_connection_label.setWordWrap(True)
        layout.addRow("数据库说明:", self.database_connection_label)
        self.airspace_mode_combo = QComboBox()
        self.airspace_mode_combo.addItem("标准模式 5km x 5km", "standard")
        self.airspace_mode_combo.addItem("大都会模式 30km x 30km", "metropolis")
        self.airspace_mode_combo.addItem("36区域矩阵模式 30km x 30km", "regional_6x6")
        initial_mode = getattr(self.env, "airspace_mode", "standard")
        initial_idx = self.airspace_mode_combo.findData(initial_mode)
        if initial_idx >= 0:
            self.airspace_mode_combo.setCurrentIndex(initial_idx)
        self.airspace_mode_combo.currentIndexChanged.connect(
            self._on_airspace_mode_changed
        )
        layout.addRow("空域模式:", self.airspace_mode_combo)
        self.topology_profile_combo = QComboBox()
        for label, value in TOPOLOGY_PROFILE_OPTIONS:
            self.topology_profile_combo.addItem(label, value)
        initial_profile = getattr(
            getattr(self.env, "topology_profile", None), "name", "balanced_sparse_v1"
        )
        if initial_profile == "legacy_dense_directed_v2":
            initial_profile = "legacy_dense"
        profile_index = self.topology_profile_combo.findData(initial_profile)
        if profile_index >= 0:
            self.topology_profile_combo.setCurrentIndex(profile_index)
        self.topology_profile_combo.setEnabled(initial_mode == "regional_6x6")
        self.topology_profile_combo.currentIndexChanged.connect(
            self._on_density_changed
        )
        layout.addRow("匝道拓扑密度:", self.topology_profile_combo)
        self.num_uavs_spin = QSpinBox()
        cfg = get_airspace_config(self._current_airspace_mode())
        self.num_uavs_spin.setRange(1, cfg.max_num_uavs)
        self.num_uavs_spin.setValue(10000 if self._current_airspace_mode() == "metropolis"
                                    else cfg.default_num_uavs)
        self.num_uavs_spin.valueChanged.connect(self._on_params_changed)
        self.population_input_label = QLabel("目标在场无人机数:")
        layout.addRow(self.population_input_label, self.num_uavs_spin)
        self.max_turns_spin = QSpinBox()
        self.max_turns_spin.setRange(1, 1000000000)
        self.max_turns_spin.setValue(cfg.default_max_turns)
        self.max_turns_spin.setSingleStep(100)
        self.max_turns_spin.setReadOnly(True)
        self.max_turns_spin.setButtonSymbols(QSpinBox.NoButtons)
        self.max_turns_spin.setToolTip("由物理时长 / 仿真步长向上取整，兼容原回合参数")
        layout.addRow("最大步数（自动换算）:", self.max_turns_spin)
        self.batch_interval_spin = QSpinBox()
        self.batch_interval_spin.setRange(1, 1000000000)
        self.batch_interval_spin.setValue(cfg.default_batch_interval)
        self.batch_interval_spin.setReadOnly(True)
        self.batch_interval_spin.setButtonSymbols(QSpinBox.NoButtons)
        layout.addRow("补入间隔步数（自动）:", self.batch_interval_spin)
        scale = getattr(self.env, "scale_config", None) or cfg
        self.num_pipes_per_layer_spin = QSpinBox()
        self.num_pipes_per_layer_spin.setRange(1, 1000)
        self.num_pipes_per_layer_spin.setValue(getattr(scale, "num_pipes_per_layer", cfg.num_pipes_per_layer))
        self.num_pipes_per_layer_spin.valueChanged.connect(self._on_params_changed)
        layout.addRow("每层原始管道数:", self.num_pipes_per_layer_spin)
        self.num_lanes_per_pipe_spin = QSpinBox()
        self.num_lanes_per_pipe_spin.setRange(1, 1000)
        self.num_lanes_per_pipe_spin.setValue(getattr(scale, "num_lanes_per_pipe", cfg.num_lanes_per_pipe))
        self.num_lanes_per_pipe_spin.valueChanged.connect(self._on_params_changed)
        layout.addRow("每管道车道数:", self.num_lanes_per_pipe_spin)
        geometry_hint = QLabel("36 区为原始管道的管理分区。修改数量后可刷新预览，开始仿真时生效。")
        geometry_hint.setWordWrap(True)
        layout.addRow(geometry_hint)
        grp.setLayout(layout)
        self.panel_layout.addWidget(grp)

    def _build_runtime_group(self):
        grp = QGroupBox("路由、物理时间与周期补入")
        layout = QFormLayout(grp)
        self.routing_mode_combo = QComboBox()
        for label, value in ROUTING_MODE_OPTIONS:
            self.routing_mode_combo.addItem(label, value)
        self.routing_mode_combo.currentIndexChanged.connect(self._on_routing_mode_changed)
        layout.addRow("路由控制模式:", self.routing_mode_combo)
        self.sim_dt_seconds_spin = QDoubleSpinBox()
        self.sim_dt_seconds_spin.setRange(0.1, 1.0)
        self.sim_dt_seconds_spin.setDecimals(1)
        self.sim_dt_seconds_spin.setSingleStep(0.1)
        self.sim_dt_seconds_spin.setValue(1.0)
        self.sim_dt_seconds_spin.setSuffix(" 秒 / 步")
        self.sim_dt_seconds_spin.setToolTip("支持 1 秒或 0.1 秒；改变步长不会改变补入周期的物理秒数")
        layout.addRow("仿真步长 dt:", self.sim_dt_seconds_spin)
        self.max_sim_time_seconds_spin = QDoubleSpinBox()
        self.max_sim_time_seconds_spin.setRange(1.0, 10000000.0)
        self.max_sim_time_seconds_spin.setDecimals(1)
        self.max_sim_time_seconds_spin.setValue(18000.0)
        self.max_sim_time_seconds_spin.setSuffix(" 秒")
        layout.addRow("仿真物理时长:", self.max_sim_time_seconds_spin)
        self.drain_timeout_seconds_spin = QDoubleSpinBox()
        self.drain_timeout_seconds_spin.setRange(0.0, 10000000.0)
        self.drain_timeout_seconds_spin.setDecimals(1)
        self.drain_timeout_seconds_spin.setValue(18000.0)
        self.drain_timeout_seconds_spin.setSuffix(" 秒")
        self.drain_timeout_seconds_spin.setToolTip("停止补入后，等待空域内无人机完成飞行的最长时间")
        layout.addRow("结束清空超时:", self.drain_timeout_seconds_spin)
        self.replenishment_enabled_check = QCheckBox("按固定物理周期补足目标在场数量")
        self.replenishment_enabled_check.setChecked(True)
        layout.addRow(self.replenishment_enabled_check)
        self.replenishment_period_seconds_spin = QDoubleSpinBox()
        self.replenishment_period_seconds_spin.setRange(0.1, 1000000.0)
        self.replenishment_period_seconds_spin.setDecimals(1)
        self.replenishment_period_seconds_spin.setValue(100.0)
        self.replenishment_period_seconds_spin.setSuffix(" 秒")
        layout.addRow("补入周期（物理秒）:", self.replenishment_period_seconds_spin)
        self.max_batch_size_spin = QSpinBox()
        self.max_batch_size_spin.setRange(1, 1000)
        self.max_batch_size_spin.setValue(1000)
        self.max_batch_size_spin.setSuffix(" 架")
        self.max_batch_size_spin.valueChanged.connect(self._on_params_changed)
        layout.addRow("初始 / 每批补入上限:", self.max_batch_size_spin)
        self.derived_steps_label = QLabel()
        self.derived_steps_label.setWordWrap(True)
        layout.addRow(self.derived_steps_label)
        self.legacy_runtime_hint = QLabel("旧空域 / 数据库沿用第一版：1 秒步长、固定总数分批起飞、按管道末端选终点；新区路由与周期补足用于增强大都会。")
        self.legacy_runtime_hint.setWordWrap(True)
        self.legacy_runtime_hint.setVisible(False)
        layout.addRow(self.legacy_runtime_hint)
        for spin in (self.sim_dt_seconds_spin, self.max_sim_time_seconds_spin,
                     self.replenishment_period_seconds_spin):
            spin.valueChanged.connect(self._update_derived_steps)
        self._update_derived_steps()
        self.panel_layout.addWidget(grp)

    def _update_derived_steps(self):
        dt = self.sim_dt_seconds_spin.value()
        duration = self.max_sim_time_seconds_spin.value()
        period = self.replenishment_period_seconds_spin.value()
        self.max_turns_spin.setValue(math.ceil(duration / dt - 1e-9))
        self.batch_interval_spin.setValue(math.ceil(period / dt - 1e-9))
        self.derived_steps_label.setText(
            f"换算：{self.max_turns_spin.value()} 步；补入每 {period:g} 秒"
            f"（{self.batch_interval_spin.value()} 步）")

    def _build_guidance_group(self):
        self.guidance_group = QGroupBox("区域流量导引")
        layout = QFormLayout(self.guidance_group)
        self.guidance_enabled_check = QCheckBox("触发拥堵导引")
        self.guidance_evaluate_check = QCheckBox("记录密度指标（关闭导引时也可用）")
        layout.addRow(self.guidance_enabled_check)
        layout.addRow(self.guidance_evaluate_check)
        self.guidance_inputs = {}
        defaults = GuidanceConfig()
        for key, (label, low, high, decimals) in PARAMETERS.items():
            widget = QDoubleSpinBox() if decimals else QSpinBox()
            if decimals:
                widget.setDecimals(decimals)
            widget.setRange(low, high)
            widget.setValue(getattr(defaults, key))
            self.guidance_inputs[key] = widget
            layout.addRow(label, widget)
        self.guidance_enabled_check.toggled.connect(self._update_guidance_inputs)
        self.guidance_evaluate_check.toggled.connect(self._update_guidance_inputs)
        self._update_guidance_inputs()
        self.panel_layout.addWidget(self.guidance_group)

    def _update_guidance_inputs(self):
        enabled = self.guidance_enabled_check.isChecked()
        evaluate = self.guidance_evaluate_check.isChecked()
        for key, widget in self.guidance_inputs.items():
            widget.setEnabled(enabled or (evaluate and key in (
                "sample_interval_seconds", "density_limit", "capacity_scale", "gamma", "deviation_rate")))

    def _guidance_settings(self):
        if not self._enhanced_runtime_selected():
            return {}
        settings = dict(enabled=self.guidance_enabled_check.isChecked(),
                        evaluate=self.guidance_evaluate_check.isChecked())
        if not any(settings.values()):
            return settings
        for key, widget in self.guidance_inputs.items():
            if widget.isEnabled() and not widget.hasAcceptableInput():
                raise ValueError("请输入有效的导引参数 {}".format(PARAMETERS[key][0]))
            if widget.isEnabled():
                settings[key] = widget.value()
        GuidanceConfig.from_mapping(settings)
        return settings

    def _on_routing_mode_changed(self):
        if (self._enhanced_runtime_selected() and hasattr(self, "use_ray_check")
                and self.routing_mode_combo.currentData() == "onboard_regional"):
            self.use_ray_check.setChecked(True)

    def _enhanced_runtime_selected(self):
        return self._current_airspace_source() == "generated" and self._current_airspace_mode() == "metropolis"

    def _apply_runtime_compatibility(self):
        """Expose only settings actually interpreted by the selected runner."""
        enhanced = self._enhanced_runtime_selected()
        previous = getattr(self, "_runtime_is_enhanced", None)
        self._runtime_is_enhanced = enhanced
        generated = self._current_airspace_source() == "generated"
        self.airspace_mode_combo.setEnabled(generated)
        self.topology_profile_combo.setEnabled(generated and self._current_airspace_mode() == "regional_6x6")
        self.inter_dynamic_combo.setEnabled(generated)
        self.intra_dynamic_combo.setEnabled(generated)
        self.dynamic_preview_btn.setText("刷新空域与匝道预览" if generated else "加载 / 刷新数据库空域")
        if generated:
            self.database_connection_label.setText("数据库连接信息来自环境变量，不在 GUI 中保存密码")
        elif self._db_preview_ready:
            self.database_connection_label.setText(
                "已加载数据库真实拓扑：{} 根管道。连接信息来自环境变量。".format(len(self.env.pipelines)))
        else:
            self.database_connection_label.setText("数据库连接信息来自环境变量。请先点击“加载 / 刷新数据库空域”，核对实际管道和终点。")
        self.legacy_runtime_hint.setVisible(not enhanced)
        self.guidance_group.setVisible(enhanced)
        self.population_input_label.setText("目标在场无人机数:" if enhanced else "无人机总数:")
        for widget in (self.routing_mode_combo, self.sim_dt_seconds_spin,
                       self.num_pipes_per_layer_spin, self.num_lanes_per_pipe_spin,
                       self.replenishment_enabled_check, self.max_batch_size_spin,
                       self.drain_timeout_seconds_spin, self.destination_mode_combo):
            widget.setEnabled(enhanced)
        for name in ("ramp_same_layer_degree", "ramp_adjacent_layer_degree", "ramp_express_layer_degree"):
            getattr(self, name + "_spin").setEnabled(enhanced)
        if not enhanced:
            self.sim_dt_seconds_spin.setValue(1.)
            self.routing_mode_combo.setCurrentIndex(0)
            self.replenishment_enabled_check.setChecked(False)
            self.destination_mode_combo.setCurrentIndex(self.destination_mode_combo.findData("manual"))
            if previous is not False:
                self.replenishment_period_seconds_spin.setValue(
                    get_airspace_config(self._current_airspace_mode()).default_batch_interval)
            self.replenishment_period_seconds_spin.setDecimals(0)
            self.replenishment_period_seconds_spin.setMinimum(1.)
        else:
            self.replenishment_period_seconds_spin.setDecimals(1)
            self.replenishment_period_seconds_spin.setMinimum(.1)
            if previous is False:
                self.destination_mode_combo.setCurrentIndex(self.destination_mode_combo.findData("random"))
                self.replenishment_period_seconds_spin.setValue(100.)
                self.replenishment_enabled_check.setChecked(True)
        for row in range(self.dest_table.rowCount()):
            lane, distance = self.dest_table.cellWidget(row, 8), self.dest_table.cellWidget(row, 9)
            if lane is None or distance is None:
                continue
            if not enhanced:
                lane.setValue(0)
                distance.setValue(distance.maximum())
            lane.setEnabled(enhanced)
            distance.setEnabled(enhanced)
        manual = self.destination_mode_combo.currentData() == "manual"
        self.dest_table.setVisible(manual)
        self.destination_count_spin.setEnabled(enhanced and not manual)
        self.destination_policy_combo.setEnabled(enhanced)
        self.random_seed_spin.setEnabled(enhanced)
        if not enhanced:
            self.destination_hint_label.setText("旧流程仅使用勾选的管道末端，不指定到达车道；新位置 / 车道输入只用于增强大都会。")
        self._update_derived_steps()

    def _current_airspace_source(self):
        if hasattr(self, "airspace_source_combo"):
            return self.airspace_source_combo.currentData() or "generated"
        return getattr(self.env, "topology_source", "generated")

    def _on_airspace_source_changed(self):
        if getattr(self, "_is_init", True):
            return
        generated = self._current_airspace_source() == "generated"
        if generated:
            self._db_preview_ready = False
        else:
            self._db_preview_ready = self._db_preview_ready and getattr(self.env, "topology_source", "generated") == "postgres"
        self.airspace_mode_combo.setEnabled(generated)
        self.num_pipes_per_layer_spin.setEnabled(generated)
        self.num_lanes_per_pipe_spin.setEnabled(generated)
        for name in ("ramp_same_layer_degree", "ramp_adjacent_layer_degree", "ramp_express_layer_degree"):
            getattr(self, name + "_spin").setEnabled(generated)
        self.topology_profile_combo.setEnabled(
            generated and self._current_airspace_mode() == "regional_6x6"
        )
        if generated:
            self._rebuild_preview_env(include_dynamic=self.preview_includes_dynamic)
            self._rebuild_pickup_rows()
            self._populate_dest_table()
        self._apply_runtime_compatibility()
        self._on_params_changed()

    def _build_capacity_group(self):
        grp = QGroupBox("运力分配 (各等级占比 %)")
        layout = QGridLayout()
        self.cap_sliders = {}
        self.cap_labels = {}
        for i in range(6):
            label = QLabel(f"Cap={i}:")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 100)
            default_val = {0: 0, 1: 20, 2: 20, 3: 20, 4: 20, 5: 20}[i]
            slider.setValue(default_val)
            val_label = QLabel(f"{default_val}%")
            val_label.setMinimumWidth(40)
            slider.valueChanged.connect(self._on_params_changed)
            layout.addWidget(label, i, 0)
            layout.addWidget(slider, i, 1)
            layout.addWidget(val_label, i, 2)
            self.cap_sliders[i] = slider
            self.cap_labels[i] = val_label
        self.total_cap_label = QLabel("总运力: 0")
        self.total_cap_label.setStyleSheet("font-weight: bold; color: #1976D2;")
        layout.addWidget(self.total_cap_label, 6, 0, 1, 3)
        grp.setLayout(layout)
        self.panel_layout.addWidget(grp)

    def _build_pickup_group(self):
        grp = QGroupBox("中间节点 (Pickup Points)")
        layout = QVBoxLayout()
        row = QHBoxLayout()
        row.addWidget(QLabel("数量:"))
        self.num_pickup_spin = QSpinBox()
        self.num_pickup_spin.setRange(0, 10)
        self.num_pickup_spin.setValue(0)
        self.num_pickup_spin.valueChanged.connect(self._on_pickup_count_changed)
        row.addWidget(self.num_pickup_spin)
        row.addStretch()
        layout.addLayout(row)
        self.pickup_container = QWidget()
        self.pickup_container_layout = QVBoxLayout(self.pickup_container)
        self.pickup_container_layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.pickup_container)
        grp.setLayout(layout)
        self.panel_layout.addWidget(grp)
        self._rebuild_pickup_rows()

    def _on_pickup_count_changed(self):
        self._rebuild_pickup_rows()
        self._on_params_changed()

    def _rebuild_pickup_rows(self, reset_locations=False):
        previous_locations = [] if reset_locations else [row.get_location() for row in self.pickup_rows]
        for row in self.pickup_rows:
            row.setParent(None)
            row.deleteLater()
        self.pickup_rows = []
        default_pipe_ids = self._default_pickup_pipe_ids()
        n = self.num_pickup_spin.value()
        for i in range(n):
            row = PickupRow(i, self.env, self._on_params_changed)
            self.pickup_container_layout.addWidget(row)
            self.pickup_rows.append(row)
            previous = previous_locations[i] if i < len(previous_locations) else None
            valid_ids = {pipe.pipe_id for pipe in self.env.pipelines}
            if previous and previous["pipe_id"] in valid_ids:
                target_pid = previous["pipe_id"]
                target_pos = previous["pos_1d"]
            elif i < len(default_pipe_ids):
                target_pid = default_pipe_ids[i]
                target_pos = self.env.pipelines[target_pid].length * 0.4
            else:
                continue
            if target_pid in valid_ids:
                for combo_idx in range(row.pipe_combo.count()):
                    if row.pipe_combo.itemData(combo_idx) == target_pid:
                        row.pipe_combo.setCurrentIndex(combo_idx)
                        break
                pipe_len = self.env.pipelines[target_pid].length
                row.pos_spin.setValue(min(pipe_len, target_pos))

    def _build_dest_group(self):
        grp = QGroupBox("终点设置")
        layout = QVBoxLayout()
        form = QFormLayout()
        self.destination_mode_combo = QComboBox()
        self.destination_mode_combo.addItem("随机最高层管道内部位置（默认）", "random")
        self.destination_mode_combo.addItem("手动选择位置和车道（默认管道末端）", "manual")
        self.destination_mode_combo.currentIndexChanged.connect(self._on_destination_settings_changed)
        form.addRow("终点来源:", self.destination_mode_combo)
        self.destination_count_spin = QSpinBox()
        self.destination_count_spin.setRange(1, 1000)
        self.destination_count_spin.setValue(10)
        self.destination_count_spin.valueChanged.connect(self._on_destination_settings_changed)
        form.addRow("随机终点数:", self.destination_count_spin)
        self.destination_policy_combo = QComboBox()
        self.destination_policy_combo.addItem("整次运行固定一组终点（默认）", "fixed")
        self.destination_policy_combo.addItem("每批重新抽取终点", "per_batch")
        self.destination_policy_combo.currentIndexChanged.connect(self._on_destination_settings_changed)
        form.addRow("终点复用策略:", self.destination_policy_combo)
        self.random_seed_spin = QSpinBox()
        self.random_seed_spin.setRange(0, 2147483647)
        self.random_seed_spin.setValue(42)
        self.random_seed_spin.valueChanged.connect(self._on_destination_settings_changed)
        form.addRow("随机种子:", self.random_seed_spin)
        layout.addLayout(form)
        self.destination_hint_label = QLabel("随机终点在开始时创建为精确内部节点；红色 × 为位置示意，整次运行共享固定终点。")
        self.destination_hint_label.setWordWrap(True)
        layout.addWidget(self.destination_hint_label)
        self.dest_table = QTableWidget()
        self.dest_table.setColumnCount(10)
        self.dest_table.setHorizontalHeaderLabels(
            ["选择", "区域", "行", "列", "管道ID", "本区ID", "层高", "长度", "车道", "位置(m)"]
        )
        self.dest_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self._populate_dest_table()
        self.dest_table.setMaximumHeight(250)
        self.dest_table.setVisible(False)
        layout.addWidget(self.dest_table)
        self.dest_count_label = QLabel("已选终点: 5")
        self.dest_count_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.dest_count_label)
        grp.setLayout(layout)
        self.panel_layout.addWidget(grp)

    def _on_destination_settings_changed(self):
        if getattr(self, "_is_init", True):
            return
        manual = self.destination_mode_combo.currentData() == "manual"
        self.dest_table.setVisible(manual)
        self.destination_count_spin.setEnabled(not manual)
        self.destination_hint_label.setText(
            "手动终点按表中车道和纵向位置执行；默认第 0 车道、管道末端。" if manual else
            "随机终点在开始时创建为精确内部节点；红色 × 为位置示意。" +
            ("整次运行共享固定终点。" if self.destination_policy_combo.currentData() == "fixed"
             else "每批重新抽取一组终点。"))
        self._on_params_changed()

    def _random_destination_preview(self):
        top_height = max((pipe.height for pipe in self.env.pipelines), default=600)
        pipes = [p for p in self.env.pipelines if p.height == top_height]
        if not pipes:
            return []
        rng = np.random.default_rng(self.random_seed_spin.value())
        count = self.destination_count_spin.value()
        indexes = rng.choice(len(pipes), size=count, replace=count > len(pipes))
        return [{"pipe_id": pipes[int(index)].pipe_id,
                 "pos_1d": float(pipes[int(index)].length * rng.uniform(0.1, 0.9))}
                for index in indexes]

    def _build_algo_group(self):
        grp = QGroupBox("算法选择")
        layout = QVBoxLayout()
        self.algo_checks = {}
        for key, (name_cn, name_en, _, _) in ALGORITHM_REGISTRY.items():
            cb = QCheckBox(f"[{key}] {name_cn}")
            cb.setChecked(key == "1")
            layout.addWidget(cb)
            self.algo_checks[key] = cb
        grp.setLayout(layout)
        self.panel_layout.addWidget(grp)

    def _build_advanced_group(self):
        grp = QGroupBox("高级参数 (V6.0)")
        layout = QFormLayout()
        self.trajectory_input = QLineEdit()
        self.trajectory_input.setPlaceholderText("10, 55, 102 或 random 5")
        layout.addRow("航迹追踪 UAV IDs:", self.trajectory_input)
        self.multi_uav_trajectory_check = QCheckBox("同时在同一张图中叠加绘制所选 UAV 航迹")
        self.multi_uav_trajectory_check.setChecked(False)
        layout.addRow(self.multi_uav_trajectory_check)
        self.trace_ids_input = QLineEdit()
        self.trace_ids_input.setPlaceholderText("例如: 17, 37；留空则不导出逐回合日志")
        layout.addRow("逐回合日志 UAV IDs:", self.trace_ids_input)
        self.trace_path_input = QLineEdit()
        self.trace_path_input.setText("uav_turn_trace.txt")
        layout.addRow("逐回合日志 TXT:", self.trace_path_input)
        self.enable_kl_check = QCheckBox("计算 KL 散度（关闭可提升速度）")
        self.enable_kl_check.setChecked(False)
        layout.addRow(self.enable_kl_check)
        self.enable_channel_gain_check = QCheckBox("计算信道/波束增益（关闭可显著提升速度）")
        self.enable_channel_gain_check.setChecked(False)
        layout.addRow(self.enable_channel_gain_check)
        self.frame_export_check = QCheckBox("导出每回合全部 UAV 的 JSON（数据量较大）")
        self.frame_export_check.setChecked(False)
        layout.addRow(self.frame_export_check)
        self.run_output_root_input = QLineEdit()
        self.run_output_root_input.setText("artifacts/runs")
        layout.addRow("计时与逐帧输出目录:", self.run_output_root_input)
        self.inter_dynamic_combo = QComboBox()
        self.intra_dynamic_combo = QComboBox()
        for label, value in DYNAMIC_LEVEL_OPTIONS:
            self.inter_dynamic_combo.addItem(label, value)
            self.intra_dynamic_combo.addItem(label, value)
        self.inter_dynamic_combo.setCurrentIndex(1)
        self.intra_dynamic_combo.setCurrentIndex(1)
        self.inter_dynamic_combo.currentIndexChanged.connect(self._on_density_changed)
        self.intra_dynamic_combo.currentIndexChanged.connect(self._on_density_changed)
        layout.addRow("跨层动态匝道密度:", self.inter_dynamic_combo)
        layout.addRow("同层动态匝道密度:", self.intra_dynamic_combo)
        self.dynamic_density_hint = QLabel("大都会模式：密度会改变实际道路数量；动态匝道在运行中初始开启并保持，不自动开关。")
        self.dynamic_density_hint.setWordWrap(True)
        self.dynamic_density_hint.setVisible(self._current_airspace_mode() == "metropolis")
        layout.addRow(self.dynamic_density_hint)
        for name, label, default in (
            ("ramp_same_layer_degree", "静态同层匝道度数:", 2),
            ("ramp_adjacent_layer_degree", "相邻层匝道度数:", 4),
            ("ramp_express_layer_degree", "跨层快捷匝道度数:", 1),
        ):
            spin = QSpinBox()
            spin.setRange(0, 8)
            spin.setValue(getattr(self.env, name, default))
            spin.setToolTip("每个原始管道的连接目标数量，开始仿真时生效")
            setattr(self, name + "_spin", spin)
            layout.addRow(label, spin)
        self.dynamic_preview_btn = QPushButton("刷新空域与匝道预览")
        self.dynamic_preview_btn.clicked.connect(self._refresh_dynamic_preview)
        layout.addRow("动态匝道预览:", self.dynamic_preview_btn)
        grp.setLayout(layout)
        self.panel_layout.addWidget(grp)

    def _build_parallel_group(self):
        grp = QGroupBox("V6.0 分布式仿真配置")
        layout = QFormLayout()
        self.use_ray_check = QCheckBox("启用 Ray 分布式引擎 (每架 UAV 独立实体)")
        self.use_ray_check.setChecked(False)
        self.use_ray_check.stateChanged.connect(self._on_ray_toggle)
        layout.addRow(self.use_ray_check)
        self.distributed_control_combo = QComboBox()
        self.distributed_control_combo.addItem("集中路由低内存模式", "centralized_route")
        self.distributed_control_combo.addItem(
            "服务器权威控制模式（寻路+避障集中）", "server_authoritative"
        )
        self.distributed_control_combo.addItem("旧版本地规划模式", "legacy_local")
        self.distributed_control_combo.setEnabled(False)
        layout.addRow("分布式控制:", self.distributed_control_combo)
        self.server_avoidance_combo = QComboBox()
        self.server_avoidance_combo.addItem("规则避障", "rule")

        self.server_avoidance_combo.setEnabled(False)
        layout.addRow("服务器避障:", self.server_avoidance_combo)
        self.safety_substep_spin = QDoubleSpinBox()
        self.safety_substep_spin.setRange(0.05, 1.0)
        self.safety_substep_spin.setSingleStep(0.05)
        self.safety_substep_spin.setDecimals(2)
        self.safety_substep_spin.setValue(0.2)
        self.safety_substep_spin.setSuffix(" 秒")
        self.safety_substep_spin.setEnabled(False)
        layout.addRow("安全预测子步长:", self.safety_substep_spin)
        parallel_row = QHBoxLayout()
        self.parallel_slider = QSlider(Qt.Horizontal)
        self.parallel_slider.setRange(1, 10000)
        self.parallel_slider.setValue(DEFAULT_UAVS_PER_ACTOR)
        self.parallel_spin = QSpinBox()
        self.parallel_spin.setRange(1, 10000)
        self.parallel_spin.setValue(DEFAULT_UAVS_PER_ACTOR)
        self.parallel_spin.setSuffix(" 架/进程")
        self.parallel_slider.valueChanged.connect(self.parallel_spin.setValue)
        self.parallel_spin.valueChanged.connect(self.parallel_slider.setValue)
        self.parallel_spin.valueChanged.connect(self._on_parallel_changed)
        parallel_row.addWidget(self.parallel_slider)
        parallel_row.addWidget(self.parallel_spin)
        layout.addRow("并行级别:", parallel_row)
        self.parallel_slider.setEnabled(False)
        self.parallel_spin.setEnabled(False)
        self.process_count_label = QLabel("预计进程数: —")
        self.process_count_label.setStyleSheet("font-weight: bold; color: #E65100;")
        layout.addRow(self.process_count_label)
        self.ray_hint_label = QLabel("提示: 并行级别=1 时每架无人机占一个独立进程，\n并行级别越大则多架共用进程，性能开销越低。")
        self.ray_hint_label.setStyleSheet("color: #757575; font-size: 11px;")
        self.ray_hint_label.setWordWrap(True)
        layout.addRow(self.ray_hint_label)
        grp.setLayout(layout)
        self.panel_layout.addWidget(grp)

    def _on_parallel_changed(self):
        if getattr(self, "_is_init", True):
            return
        num_uavs = self.num_uavs_spin.value()
        level = self.parallel_spin.value()
        import math

        num_processes = math.ceil(num_uavs / level)
        self.process_count_label.setText(f"预计进程数: {num_processes}")

    def _on_ray_toggle(self, state):
        enabled = state == Qt.Checked
        self.parallel_slider.setEnabled(enabled)
        self.parallel_spin.setEnabled(enabled)
        self.distributed_control_combo.setEnabled(enabled)
        self.server_avoidance_combo.setEnabled(enabled)
        self.safety_substep_spin.setEnabled(enabled)
        if not enabled:
            pass
        if enabled:
            self._on_parallel_changed()
        else:
            self.process_count_label.setText("预计进程数: — (串行模式)")

    def _current_airspace_mode(self):
        if hasattr(self, "airspace_mode_combo"):
            return self.airspace_mode_combo.currentData() or "standard"
        return getattr(self.env, "airspace_mode", "standard")

    def _current_topology_profile(self):
        if self._current_airspace_mode() == "metropolis":
            return "metropolis_partition_v1"
        if self._current_airspace_mode() != "regional_6x6":
            return "legacy_dense"
        if hasattr(self, "topology_profile_combo"):
            return self.topology_profile_combo.currentData() or "balanced_sparse_v1"
        return getattr(
            getattr(self.env, "topology_profile", None), "name", "balanced_sparse_v1"
        )

    def _on_airspace_mode_changed(self):
        if getattr(self, "_is_init", True):
            return
        if self._current_airspace_source() != "generated":
            return
        cfg = get_airspace_config(self._current_airspace_mode())
        self.num_uavs_spin.setRange(1, cfg.max_num_uavs)
        self.num_uavs_spin.setValue(10000 if self._current_airspace_mode() == "metropolis"
                                    else cfg.default_num_uavs)
        self.max_sim_time_seconds_spin.setValue(cfg.default_max_turns)
        self.num_pipes_per_layer_spin.setValue(cfg.num_pipes_per_layer)
        self.num_lanes_per_pipe_spin.setValue(cfg.num_lanes_per_pipe)
        is_regional = self._current_airspace_mode() == "regional_6x6"
        self.topology_profile_combo.setEnabled(is_regional)
        default_profile = ("balanced_sparse_v1" if is_regional else
                           "metropolis_partition_v1" if self._current_airspace_mode() == "metropolis"
                           else "legacy_dense")
        profile_index = self.topology_profile_combo.findData(default_profile)
        if profile_index >= 0:
            self.topology_profile_combo.setCurrentIndex(profile_index)
        self._rebuild_preview_env(include_dynamic=self.preview_includes_dynamic)
        self._rebuild_pickup_rows()
        self._populate_dest_table()
        self._apply_runtime_compatibility()
        self._on_params_changed()

    def _build_buttons(self):
        btn_layout = QHBoxLayout()
        start_btn = QPushButton("▶  开始仿真")
        start_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-size: 16px; font-weight: bold; padding: 10px; border-radius: 5px; }QPushButton:hover { background-color: #388E3C; }"
        )
        start_btn.clicked.connect(self._validate_and_start)
        btn_layout.addWidget(start_btn)
        cancel_btn = QPushButton("✕  取消退出")
        cancel_btn.setStyleSheet(
            "QPushButton { background-color: #f44336; color: white; font-size: 16px; font-weight: bold; padding: 10px; border-radius: 5px; }QPushButton:hover { background-color: #D32F2F; }"
        )
        cancel_btn.clicked.connect(self._cancel)
        btn_layout.addWidget(cancel_btn)
        self.panel_layout.addLayout(btn_layout)

    def _get_dest_pipe_ids(self):
        ids = []
        available_ids = {pipe.pipe_id for pipe in self.env.pipelines}
        for i in range(self.dest_table.rowCount()):
            cb_widget = self.dest_table.cellWidget(i, 0)
            cb = cb_widget.findChild(QCheckBox) if cb_widget is not None else None
            item = self.dest_table.item(i, 4)
            if cb and cb.isChecked() and item is not None:
                pipe_id = int(item.text())
                if pipe_id in available_ids:
                    ids.append(pipe_id)
        return ids

    def _get_manual_destinations(self, validate=False):
        """Return the selected physical points without changing their positions."""
        selected = set(self._get_dest_pipe_ids())
        pipes = {pipe.pipe_id: pipe for pipe in self.env.pipelines}
        result = []
        for row in range(self.dest_table.rowCount()):
            item = self.dest_table.item(row, 4)
            if item is None:
                continue
            pid = int(item.text())
            if pid not in selected:
                continue
            lane = self.dest_table.cellWidget(row, 8)
            distance = self.dest_table.cellWidget(row, 9)
            if lane is None or distance is None:
                continue
            if validate and (not lane.hasAcceptableInput() or not distance.hasAcceptableInput()):
                raise ValueError("手动终点的车道和位置必须为有效数值。")
            # QDoubleSpinBox rounds its displayed maximum. Preserve the exact
            # original endpoint when its default maximum remains selected.
            position = float(pipes[pid].length) if distance.value() == distance.maximum() else distance.value()
            result.append(dict(node_id="GOAL_MANUAL_{}".format(pid), pipe_id=pid,
                               lane_id=lane.value(), distance_m=position))
        return result

    def _default_dest_pipe_ids(self):
        top_pipes = [p.pipe_id for p in self.env.pipelines if p.height == 600]
        if not top_pipes:
            height = max((pipe.height for pipe in self.env.pipelines), default=None)
            top_pipes = [p.pipe_id for p in self.env.pipelines if p.height == height]
        if len(top_pipes) <= 5:
            return set(top_pipes)
        idxs = np.linspace(0, len(top_pipes) - 1, 5).round().astype(int)
        return {top_pipes[int(i)] for i in idxs}

    def _default_pickup_pipe_ids(self):
        defaults = []
        preferred_regions = [0, 7, 14, 28, 35]
        for idx, height in enumerate([100, 200, 300, 400, 500]):
            region_id = preferred_regions[idx] if idx < len(preferred_regions) else None
            region_pipes = [
                p.pipe_id
                for p in self.env.pipelines
                if p.height == height and getattr(p, "region_id", None) == region_id
            ]
            pipes = region_pipes or [
                p.pipe_id for p in self.env.pipelines if p.height == height
            ]
            if pipes:
                defaults.append(pipes[len(pipes) // 2])
        return defaults

    def _populate_dest_table(self, selected_ids=None, selected_specs=()):
        if not hasattr(self, "dest_table"):
            return
        default_dest_ids = self._default_dest_pipe_ids() if selected_ids is None else set(selected_ids)
        previous = {spec["pipe_id"]: spec for spec in selected_specs}
        self.dest_table.setRowCount(len(self.env.pipelines))
        for i, pipe in enumerate(self.env.pipelines):
            h = pipe.height
            angle = [lc["angle"] for lc in self.env.layer_configs if lc["height"] == h][
                0
            ]
            cb = QCheckBox()
            cb.setChecked(pipe.pipe_id in default_dest_ids)
            cb.stateChanged.connect(self._on_params_changed)
            cb_widget = QWidget()
            cb_layout = QHBoxLayout(cb_widget)
            cb_layout.addWidget(cb)
            cb_layout.setAlignment(Qt.AlignCenter)
            cb_layout.setContentsMargins(0, 0, 0, 0)
            self.dest_table.setCellWidget(i, 0, cb_widget)
            self.dest_table.setItem(
                i, 1, QTableWidgetItem(str(getattr(pipe, "region_id", 0)))
            )
            self.dest_table.setItem(
                i, 2, QTableWidgetItem(str(getattr(pipe, "region_row", 0)))
            )
            self.dest_table.setItem(
                i, 3, QTableWidgetItem(str(getattr(pipe, "region_col", 0)))
            )
            self.dest_table.setItem(i, 4, QTableWidgetItem(str(pipe.pipe_id)))
            self.dest_table.setItem(
                i,
                5,
                QTableWidgetItem(str(getattr(pipe, "local_pipe_id", pipe.pipe_id))),
            )
            self.dest_table.setItem(i, 6, QTableWidgetItem(f"{h}m/{angle}°"))
            self.dest_table.setItem(i, 7, QTableWidgetItem(f"{pipe.length:.0f}m"))
            spec = previous.get(pipe.pipe_id, {})
            lane_spin = QSpinBox()
            lane_spin.setRange(0, max(0, int(getattr(pipe, "num_lanes", 1)) - 1))
            lane_spin.setValue(spec.get("lane_id", 0))
            lane_spin.valueChanged.connect(self._on_params_changed)
            self.dest_table.setCellWidget(i, 8, lane_spin)
            distance_spin = QDoubleSpinBox()
            distance_spin.setDecimals(6)
            distance_spin.setRange(0., float(pipe.length))
            distance_spin.setSingleStep(100.)
            distance_spin.setValue(spec.get("distance_m", float(pipe.length)))
            distance_spin.valueChanged.connect(self._on_params_changed)
            self.dest_table.setCellWidget(i, 9, distance_spin)

    def _get_capacity_weights(self):
        weights = {}
        for i in range(6):
            weights[i] = self.cap_sliders[i].value()
        return weights

    def _compute_total_capacity(self):
        num_uavs = self.num_uavs_spin.value()
        weights = self._get_capacity_weights()
        total_w = sum(weights.values())
        if total_w == 0:
            return 0
        total_cap = 0
        for cap_level, w in weights.items():
            count = round(num_uavs * w / total_w)
            total_cap += cap_level * count
        return total_cap

    def _get_dynamic_levels(self):
        return {
            "inter_dynamic_level": self.inter_dynamic_combo.currentData(),
            "intra_dynamic_level": self.intra_dynamic_combo.currentData(),
        }

    def _format_ramp_stats(self):
        stats = getattr(self.env, "ramp_stats", {}) or {}
        if not stats:
            return "匝道: —"
        return f"静态: {stats.get('intra_static', 0) + stats.get('inter_static', 0)} | 动态: {stats.get('inter_dynamic', 0) + stats.get('intra_dynamic', 0)} | 跨区: {stats.get('cross_region', 0)}"

    def _spawn_slot_capacity(self):
        return sum(
            (
                int(getattr(pipe, "num_lanes", 0))
                for pipe in self.env.pipelines
                if pipe.height == 100
            )
        )

    def _on_density_changed(self):
        if getattr(self, "_is_init", True):
            return
        self._on_params_changed()

    def _rebuild_preview_env(self, include_dynamic):
        if self._current_airspace_source() != "generated":
            return
        from uav_sim.airspace import Airspace
        import inspect

        levels = self._get_dynamic_levels()
        cfg = get_airspace_config(self._current_airspace_mode())
        defaults = {
            "num_pipes_per_layer": cfg.num_pipes_per_layer,
            "num_lanes_per_pipe": cfg.num_lanes_per_pipe,
            "ramp_same_layer_degree": 2,
            "ramp_adjacent_layer_degree": 4,
            "ramp_express_layer_degree": 1,
        }
        supported = inspect.signature(Airspace).parameters
        geometry_kwargs = {}
        for name, default in defaults.items():
            control = getattr(self, name + "_spin", None)
            if name in supported:
                geometry_kwargs[name] = control.value() if control is not None else default
        seed_control = getattr(self, "random_seed_spin", None)
        self.env = Airspace(
            mode=self._current_airspace_mode(),
            topology_profile=self._current_topology_profile(),
            inter_dynamic_level=levels["inter_dynamic_level"],
            intra_dynamic_level=levels["intra_dynamic_level"],
            include_dynamic=include_dynamic,
            random_seed=seed_control.value() if seed_control is not None else 42,
            **geometry_kwargs,
        )
        self.canvas.env = self.env

    def _refresh_dynamic_preview(self):
        if self._current_airspace_source() == "postgres":
            self._load_database_preview()
            return
        selected_ids = self._get_dest_pipe_ids()
        selected_specs = self._get_manual_destinations()
        self.preview_includes_dynamic = True
        self._rebuild_preview_env(include_dynamic=True)
        was_init = self._is_init
        self._is_init = True
        try:
            self._rebuild_pickup_rows()
            self._populate_dest_table(selected_ids=selected_ids, selected_specs=selected_specs)
        finally:
            self._is_init = was_init
        self._refresh_preview()
        self._on_params_changed()

    @staticmethod
    def _database_config():
        return {
            "host_env": "UAV_SIM_DB_HOST", "port": 5432,
            "database_env": "UAV_SIM_DB_NAME", "user_env": "UAV_SIM_DB_USER",
            "password_env": "UAV_SIM_DB_PASSWORD", "schema": "public",
            "strict_validation": True, "derive_lane_width_from_pipe_width": True,
            "ramp_poll_interval_turns": 1, "fallback_to_generated": False,
        }

    def _load_database_preview(self):
        """Read the requested database only after an explicit refresh click."""
        try:
            from uav_sim.app import _build_airspace_from_config
            preview = _build_airspace_from_config(dict(airspace_source="postgres",
                airspace_mode=self._current_airspace_mode(), database=self._database_config()))
            if getattr(preview, "topology_source", None) != "postgres" or not preview.pipelines:
                raise ValueError("Database preview did not contain a real nonempty database topology")
        except Exception as error:
            self._db_preview_ready = False
            self._apply_runtime_compatibility()
            QMessageBox.warning(self, "数据库地图加载失败",
                "未能加载数据库空域，请检查连接环境变量后点击刷新重试。不会用程序生成地图替代数据库。错误类型：{}".format(type(error).__name__))
            return False
        was_init = self._is_init
        self._is_init = True
        try:
            self.env = preview
            self.canvas.env = preview
            self._db_preview_ready = True
            self._rebuild_pickup_rows(reset_locations=True)
            self._populate_dest_table()
        finally:
            self._is_init = was_init
        self._apply_runtime_compatibility()
        self._on_params_changed()
        return True

    def _on_params_changed(self):
        if getattr(self, "_is_init", True):
            return
        self.dynamic_density_hint.setVisible(self._current_airspace_source() == "generated"
                                             and self._current_airspace_mode() == "metropolis")
        for i in range(6):
            self.cap_labels[i].setText(f"{self.cap_sliders[i].value()}%")
        total_cap = self._compute_total_capacity()
        num_uavs = self.num_uavs_spin.value()
        spawn_capacity = self._spawn_slot_capacity()
        enhanced = self._enhanced_runtime_selected()
        batch_limit = self.max_batch_size_spin.value() if enhanced else spawn_capacity
        batch_capacity = min(spawn_capacity, batch_limit)
        num_batches = math.ceil(num_uavs / max(1, batch_capacity))
        self.total_cap_label.setText(f"总运力: {total_cap}")
        dest_ids = self._get_dest_pipe_ids()
        manual = self.destination_mode_combo.currentData() == "manual"
        dest_count = len(dest_ids) if manual else self.destination_count_spin.value()
        self.dest_count_label.setText(
            f"已选终点: {dest_count}" if manual else f"随机最高层内部终点: {dest_count}")
        self.info_label.setText(
            f"总运力: {total_cap} | {'目标在场' if enhanced else '总数'}: {num_uavs} 架 | 初始批次数: {num_batches}\n"
            f"起飞槽位: {spawn_capacity} | 每批上限: {batch_limit} | "
            f"终点: {dest_count} 个 | 中间节点: {self.num_pickup_spin.value()} 个 | {self._format_ramp_stats()}"
        )
        if hasattr(self, "use_ray_check") and self.use_ray_check.isChecked():
            self._on_parallel_changed()
        self._refresh_preview()

    def _refresh_preview(self):
        manual = self.destination_mode_combo.currentData() == "manual"
        pickup_markers = [row.get_location() for row in self.pickup_rows]
        markers = ([dict(pipe_id=spec["pipe_id"], pos_1d=spec["distance_m"], lane_id=spec["lane_id"])
                    for spec in self._get_manual_destinations()]
                   if manual else self._random_destination_preview())
        if manual and not self._enhanced_runtime_selected():
            markers = [dict(pipe_id=point['pipe_id'], pos_1d=point['pos_1d']) for point in markers]
        self.canvas.refresh([], pickup_markers, markers)

    def _validate_and_start(self):
        self._apply_runtime_compatibility()
        if self._current_airspace_source() == "postgres" and not self._db_preview_ready:
            QMessageBox.warning(self, "请先加载数据库地图", "请先点击“加载 / 刷新数据库空域”，再核对实际终点和中间节点。")
            return
        for spin, label in ((self.num_pipes_per_layer_spin, "每层管道数"),
                            (self.num_lanes_per_pipe_spin, "每管道车道数"),
                            (self.destination_count_spin, "随机终点数"),
                            (self.max_batch_size_spin, "每批数量")):
            if not spin.hasAcceptableInput() or spin.value() <= 0:
                QMessageBox.warning(self, "校验失败", f"{label}必须为有效正整数。")
                return
        for name in ("ramp_same_layer_degree", "ramp_adjacent_layer_degree", "ramp_express_layer_degree"):
            spin = getattr(self, name + "_spin")
            if not spin.hasAcceptableInput():
                QMessageBox.warning(self, "校验失败", "匝道度数必须为 0 到 8 的整数。")
                return
        if not self.random_seed_spin.hasAcceptableInput():
            QMessageBox.warning(self, "校验失败", "随机种子必须为有效的非负整数。")
            return
        if (not self.max_sim_time_seconds_spin.hasAcceptableInput()
                or not self.drain_timeout_seconds_spin.hasAcceptableInput()):
            QMessageBox.warning(self, "校验失败", "请输入有效的仿真物理时长和结束清空超时。")
            return
        dt = self.sim_dt_seconds_spin.value()
        if not self.sim_dt_seconds_spin.hasAcceptableInput() or not any(
            math.isclose(dt, allowed) for allowed in (0.1, 1.0)
        ):
            QMessageBox.warning(self, "校验失败", "仿真步长 dt 请选择 1 秒或 0.1 秒。")
            return
        period = self.replenishment_period_seconds_spin.value()
        if (not self.replenishment_period_seconds_spin.hasAcceptableInput()
                or period <= 0 or not math.isclose(period / dt, round(period / dt), abs_tol=1e-7)):
            QMessageBox.warning(self, "校验失败", "补入周期必须为正物理秒数，且为 dt 的整数倍。")
            return
        manual = self.destination_mode_combo.currentData() == "manual"
        scale = getattr(self.env, "scale_config", None)
        geometry_changed = self._enhanced_runtime_selected() and any(
            getattr(scale, name, getattr(self, name + "_spin").value()) != getattr(self, name + "_spin").value()
            for name in ("num_pipes_per_layer", "num_lanes_per_pipe"))
        if geometry_changed and (manual or self.num_pickup_spin.value() > 0):
            QMessageBox.warning(self, "校验失败", "管道或车道数量已修改，请先点击“刷新空域与匝道预览”，再核对手动终点和中间节点位置。")
            return
        dest_ids = self._get_dest_pipe_ids() if manual else []
        if manual and not dest_ids:
            QMessageBox.warning(self, "校验失败", "请至少选择 1 个终点管道。")
            return
        try:
            manual_destinations = self._get_manual_destinations(validate=True) if manual else None
        except ValueError as error:
            QMessageBox.warning(self, "校验失败", str(error))
            return
        if not manual and not self.env.pipelines:
            QMessageBox.warning(self, "校验失败", "当前空域没有可创建随机终点的管道。")
            return
        self._update_derived_steps()
        self._on_routing_mode_changed()
        selected_algos = [k for k, cb in self.algo_checks.items() if cb.isChecked()]
        if not selected_algos:
            QMessageBox.warning(self, "校验失败", "请至少选择 1 种算法。")
            return
        total_cap = self._compute_total_capacity()
        num_pickup = self.num_pickup_spin.value()
        if total_cap == 0 and num_pickup > 0:
            reply = QMessageBox.information(
                self, "运力为零", "所有无人机的运力总和为 0，已自动将中间节点数设为 0（直飞模式）。", QMessageBox.Ok
            )
            self.num_pickup_spin.setValue(0)
            num_pickup = 0
        pickup_locations = []
        for row in self.pickup_rows:
            loc = row.get_location()
            if loc["pipe_id"] is None:
                QMessageBox.warning(self, "校验失败", "中间节点的管道 ID 无效，请检查配置。")
                return
            pickup_locations.append(loc)
        try:
            guidance = self._guidance_settings()
        except ValueError as error:
            QMessageBox.warning(self, "导引参数校验失败", str(error))
            return
        self.config = {
            "num_uavs": self.num_uavs_spin.value(),
            "capacity_weights": self._get_capacity_weights(),
            "num_pickup_points": num_pickup,
            "pickup_locations": pickup_locations,
            "dest_pipe_ids": dest_ids,
            "manual_destinations": manual_destinations,
            "destination_mode": self.destination_mode_combo.currentData(),
            "destination_count": self.destination_count_spin.value(),
            "destination_policy": self.destination_policy_combo.currentData(),
            "random_seed": self.random_seed_spin.value(),
            "routing_mode": self.routing_mode_combo.currentData(),
            "guidance": guidance,
            "sim_dt_seconds": dt,
            "max_sim_time_seconds": self.max_sim_time_seconds_spin.value(),
            "drain_timeout_seconds": self.drain_timeout_seconds_spin.value(),
            "replenishment_enabled": self.replenishment_enabled_check.isChecked(),
            "replenishment_period_seconds": period,
            "max_batch_size": self.max_batch_size_spin.value(),
            "num_pipes_per_layer": self.num_pipes_per_layer_spin.value(),
            "num_lanes_per_pipe": self.num_lanes_per_pipe_spin.value(),
            "ramp_same_layer_degree": self.ramp_same_layer_degree_spin.value(),
            "ramp_adjacent_layer_degree": self.ramp_adjacent_layer_degree_spin.value(),
            "ramp_express_layer_degree": self.ramp_express_layer_degree_spin.value(),
            "selected_algorithms": selected_algos,
            "airspace_source": self._current_airspace_source(),
            "airspace_mode": self._current_airspace_mode(),
            "topology_profile": self._current_topology_profile(),
            "max_turns": self.max_turns_spin.value(),
            "batch_interval": self.batch_interval_spin.value(),
            "trajectory_ids_raw": self.trajectory_input.text().strip(),
            "plot_multi_uav_same_figure": self.multi_uav_trajectory_check.isChecked(),
            "trace_ids_raw": self.trace_ids_input.text().strip(),
            "trace_log_path": self.trace_path_input.text().strip(),
            "enable_kl_divergence": self.enable_kl_check.isChecked(),
            "enable_channel_gain": self.enable_channel_gain_check.isChecked(),
            "frame_export_enabled": self.frame_export_check.isChecked(),
            "run_output_root": self.run_output_root_input.text().strip()
            or "artifacts/runs",
            "inter_dynamic_level": self.inter_dynamic_combo.currentData(),
            "intra_dynamic_level": self.intra_dynamic_combo.currentData(),
            "database": self._database_config(),
            "use_ray": self.use_ray_check.isChecked(),
            "parallelism_level": self.parallel_spin.value(),
            "distributed_control_mode": self.distributed_control_combo.currentData()
            if self.use_ray_check.isChecked()
            else "legacy_local",
            "server_avoidance_policy": self.server_avoidance_combo.currentData(),
            "safety_substep": self.safety_substep_spin.value(),
        }
        self.close()

    def _cancel(self):
        self.config = None
        self.close()
