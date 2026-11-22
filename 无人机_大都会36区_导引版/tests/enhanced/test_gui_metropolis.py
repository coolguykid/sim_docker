"""Incremental metropolis controls on the original Qt configuration window."""
import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PyQt5.QtWidgets import QApplication, QCheckBox, QMainWindow, QMessageBox

from uav_sim.ui.gui_config import AirspaceCanvas, ConfigGUI


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def no_modal_dialogs(monkeypatch):
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: QMessageBox.Ok)
    monkeypatch.setattr(QMessageBox, "information", lambda *args: QMessageBox.Ok)


@pytest.fixture
def gui(app):
    pipes = []
    for height in range(100, 601, 100):
        for local in range(12):
            start = np.array([100.0, 100.0 + local * 100, float(height)])
            end = np.array([29900.0, 100.0 + local * 100, float(height)])
            pipes.append(SimpleNamespace(
                pipe_id=len(pipes), height=height, start_pt=start, end_pt=end,
                direction=np.array([1.0, 0.0, 0.0]), length=29800.0,
                num_lanes=20, region_id=local % 36, region_row=0,
                region_col=local % 6, local_pipe_id=local,
            ))
    env = SimpleNamespace(
        airspace_mode="metropolis", topology_source="generated", pipelines=pipes,
        ramps=[], layer_configs=[{"height": h, "angle": i * 30}
            for i, h in enumerate(range(100, 601, 100))],
        scale_config=SimpleNamespace(num_pipes_per_layer=50, num_lanes_per_pipe=20),
        x_min=0, x_max=30000, y_min=0, y_max=30000, z_min=0, z_max=600,
        ramp_stats={}, topology_profile=SimpleNamespace(name="metropolis_partition_v1"),
    )
    window = ConfigGUI(env)
    yield window
    window.close()
    app.processEvents()


def test_original_window_preserved_with_metropolis_defaults(gui):
    assert isinstance(gui, QMainWindow)
    assert isinstance(gui.canvas, AirspaceCanvas)
    assert gui.settings_scroll.minimumWidth() >= 420
    assert gui.airspace_mode_combo.currentData() == "metropolis"
    assert gui.topology_profile_combo.currentData() == "metropolis_partition_v1"
    assert gui.num_uavs_spin.value() == 10000
    assert gui.num_pickup_spin.value() == 0
    assert gui.num_pipes_per_layer_spin.value() == 50
    assert gui.num_lanes_per_pipe_spin.value() == 20
    assert [key for key, check in gui.algo_checks.items() if check.isChecked()] == ["1"]
    assert gui.destination_mode_combo.currentData() == "random"
    assert gui.destination_count_spin.value() == 10
    assert gui.destination_policy_combo.currentData() == "fixed"
    assert gui.random_seed_spin.value() == 42


def test_preview_refresh_keeps_the_loaded_dynamic_roads_enabled(app):
    from uav_sim.airspace.environment import Airspace
    env=Airspace(mode='metropolis',num_pipes_per_layer=2,include_dynamic=True)
    window=ConfigGUI(env)
    try:
        assert env.ramp_stats['inter_dynamic'] > 0
        assert window.preview_includes_dynamic
        window._rebuild_preview_env(include_dynamic=window.preview_includes_dynamic)
        assert window.env.ramp_stats['inter_dynamic'] > 0
    finally:
        window.close()


def test_default_config_does_not_require_manual_endpoints(gui):
    for row in range(gui.dest_table.rowCount()):
        gui.dest_table.cellWidget(row, 0).findChild(QCheckBox).setChecked(False)
    gui._validate_and_start()
    assert gui.config is not None
    assert gui.config["destination_mode"] == "random"
    assert gui.config["destination_count"] == 10
    assert gui.config["destination_policy"] == "fixed"
    assert gui.config["dest_pipe_ids"] == []
    assert gui.config["routing_mode"] == "centralized_full"
    assert gui.config["topology_profile"] == "metropolis_partition_v1"
    assert gui.config["replenishment_enabled"] is True
    assert gui.config["max_batch_size"] == 1000
    assert gui.config["replenishment_period_seconds"] == 100.0
    assert gui.config["max_sim_time_seconds"] == 18000.0
    assert gui.config["drain_timeout_seconds"] == 18000.0


def test_physical_period_and_duration_are_stable_when_dt_changes(gui):
    gui.sim_dt_seconds_spin.setValue(0.1)
    assert gui.replenishment_period_seconds_spin.value() == 100.0
    assert gui.max_sim_time_seconds_spin.value() == 18000.0
    assert gui.max_turns_spin.value() == 180000
    assert gui.batch_interval_spin.value() == 1000
    assert "180000" in gui.derived_steps_label.text()
    gui._validate_and_start()
    assert gui.config["sim_dt_seconds"] == 0.1
    assert gui.config["max_turns"] == 180000
    assert gui.config["batch_interval"] == 1000


def test_all_routing_modes_and_onboard_auto_enable_ray(gui):
    modes = [gui.routing_mode_combo.itemData(i)
             for i in range(gui.routing_mode_combo.count())]
    assert modes == ["centralized_full", "centralized_regional", "onboard_regional"]
    gui.routing_mode_combo.setCurrentIndex(2)
    assert gui.use_ray_check.isChecked()
    gui.use_ray_check.setChecked(False)
    gui._validate_and_start()
    assert gui.config["routing_mode"] == "onboard_regional"
    assert gui.config["use_ray"] is True


def test_guidance_controls_feed_the_same_configuration_as_cli(gui):
    assert not gui.guidance_enabled_check.isChecked()
    gui.guidance_enabled_check.setChecked(True)
    gui.guidance_evaluate_check.setChecked(True)
    gui.guidance_inputs["gamma"].setValue(.2)
    gui.guidance_inputs["sample_interval_seconds"].setValue(7)
    gui.guidance_inputs["edge_penalty_seconds"].setValue(80.)
    gui._validate_and_start()
    guidance = gui.config["guidance"]
    assert guidance["enabled"] and guidance["evaluate"]
    assert guidance["gamma"] == .2
    assert guidance["sample_interval_seconds"] == 7
    assert guidance["edge_penalty_seconds"] == 80.


def test_manual_destination_validation_and_per_batch_saved(gui, monkeypatch):
    gui.destination_mode_combo.setCurrentIndex(gui.destination_mode_combo.findData("manual"))
    for row in range(gui.dest_table.rowCount()):
        gui.dest_table.cellWidget(row, 0).findChild(QCheckBox).setChecked(False)
    warnings = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: warnings.append(args[2]))
    gui._validate_and_start()
    assert gui.config is None
    assert warnings and "终点" in warnings[-1]
    gui.dest_table.cellWidget(60, 0).findChild(QCheckBox).setChecked(True)
    gui.destination_policy_combo.setCurrentIndex(gui.destination_policy_combo.findData("per_batch"))
    gui._validate_and_start()
    assert gui.config["destination_mode"] == "manual"
    assert gui.config["dest_pipe_ids"] == [60]
    assert gui.config["destination_policy"] == "per_batch"


def test_manual_destinations_keep_exact_endpoint_default_and_edited_lane_position(gui):
    gui.destination_mode_combo.setCurrentIndex(1)
    for row in range(gui.dest_table.rowCount()):
        gui.dest_table.cellWidget(row, 0).findChild(QCheckBox).setChecked(row == 60)
    assert gui.dest_table.columnCount() == 10
    gui._validate_and_start()
    assert gui.config["manual_destinations"] == [dict(
        node_id="GOAL_MANUAL_60", pipe_id=60, lane_id=0,
        distance_m=gui.env.pipelines[60].length)]
    gui.dest_table.cellWidget(60, 8).setValue(7)
    gui.dest_table.cellWidget(60, 9).setValue(1234.5)
    gui._validate_and_start()
    assert gui.config["manual_destinations"] == [dict(
        node_id="GOAL_MANUAL_60", pipe_id=60, lane_id=7, distance_m=1234.5)]


def test_manual_goal_lane_position_survive_explicit_preview_refresh(gui):
    gui.destination_mode_combo.setCurrentIndex(1)
    for row in range(gui.dest_table.rowCount()):
        gui.dest_table.cellWidget(row, 0).findChild(QCheckBox).setChecked(row == 5)
    assert gui.dest_table.columnCount() == 10
    gui.dest_table.cellWidget(5, 8).setValue(3)
    gui.dest_table.cellWidget(5, 9).setValue(1234.5)
    gui.num_pipes_per_layer_spin.setValue(8)
    gui.num_lanes_per_pipe_spin.setValue(4)
    gui._refresh_dynamic_preview()
    gui._validate_and_start()
    assert gui.config["manual_destinations"] == [dict(
        node_id="GOAL_MANUAL_5", pipe_id=5, lane_id=3, distance_m=1234.5)]


def test_manual_goal_invalid_lane_text_is_rejected(gui, monkeypatch):
    gui.destination_mode_combo.setCurrentIndex(1)
    for row in range(gui.dest_table.rowCount()):
        gui.dest_table.cellWidget(row, 0).findChild(QCheckBox).setChecked(row == 60)
    assert gui.dest_table.columnCount() == 10
    gui.dest_table.cellWidget(60, 8).lineEdit().setText("3.5")
    warnings = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: warnings.append(args[2]))
    gui._validate_and_start()
    assert gui.config is None
    assert warnings


@pytest.mark.parametrize("manual,pickups", [(True, 0), (False, 1)])
def test_selected_physical_points_require_refresh_after_geometry_changes(gui, monkeypatch, manual, pickups):
    if manual:
        gui.destination_mode_combo.setCurrentIndex(1)
    gui.num_pickup_spin.setValue(pickups)
    gui.num_pipes_per_layer_spin.setValue(8)
    warnings = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: warnings.append(args[2]))
    gui._validate_and_start()
    assert gui.config is None
    assert warnings and "刷新" in warnings[-1]


def test_numeric_settings_do_not_rebuild_topology_and_old_controls_saved(gui, monkeypatch):
    def unexpected_rebuild(*args, **kwargs):
        pytest.fail("scalar settings must not regenerate the entire airspace")
    monkeypatch.setattr(gui, "_rebuild_preview_env", unexpected_rebuild)
    gui.num_uavs_spin.setValue(12000)
    gui.num_pipes_per_layer_spin.setValue(60)
    gui.num_lanes_per_pipe_spin.setValue(24)
    gui.max_batch_size_spin.setValue(800)
    gui.random_seed_spin.setValue(17)
    gui.trajectory_input.setText("random 5")
    gui.trace_ids_input.setText("17, 37")
    gui.frame_export_check.setChecked(True)
    gui.enable_kl_check.setChecked(True)
    gui.enable_channel_gain_check.setChecked(True)
    gui._validate_and_start()
    assert gui.config["num_pipes_per_layer"] == 60
    assert gui.config["num_lanes_per_pipe"] == 24
    assert gui.config["random_seed"] == 17
    assert gui.config["max_batch_size"] == 800
    assert gui.config["trajectory_ids_raw"] == "random 5"
    assert gui.config["trace_ids_raw"] == "17, 37"
    assert gui.config["frame_export_enabled"] is True
    assert gui.config["enable_kl_divergence"] is True
    assert gui.config["enable_channel_gain"] is True
    assert gui.config["database"]["password_env"] == "UAV_SIM_DB_PASSWORD"


def test_random_preview_destinations_are_top_layer_interior_points(gui):
    markers = gui._random_destination_preview()
    assert len(markers) == 10
    assert markers == gui._random_destination_preview()
    for marker in markers:
        pipe = gui.env.pipelines[marker["pipe_id"]]
        assert pipe.height == 600
        assert 0.0 < marker["pos_1d"] < pipe.length


def test_invalid_dt_and_integer_fields_are_rejected(gui, monkeypatch):
    warnings = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: warnings.append(args[2]))
    gui.sim_dt_seconds_spin.setValue(0.2)
    gui._validate_and_start()
    assert gui.config is None
    assert warnings and "0.1" in warnings[-1]


def test_ramp_degrees_and_physical_controls_are_saved(gui):
    assert gui.ramp_same_layer_degree_spin.value() == 2
    assert gui.ramp_adjacent_layer_degree_spin.value() == 4
    assert gui.ramp_express_layer_degree_spin.value() == 1
    gui.ramp_same_layer_degree_spin.setValue(3)
    gui.ramp_adjacent_layer_degree_spin.setValue(5)
    gui.ramp_express_layer_degree_spin.setValue(0)
    gui.replenishment_enabled_check.setChecked(False)
    gui.drain_timeout_seconds_spin.setValue(3600)
    gui._validate_and_start()
    assert gui.config["ramp_same_layer_degree"] == 3
    assert gui.config["ramp_adjacent_layer_degree"] == 5
    assert gui.config["ramp_express_layer_degree"] == 0
    assert gui.config["replenishment_enabled"] is False
    assert gui.config["drain_timeout_seconds"] == 3600.0


def test_fractional_integer_setting_is_rejected(gui, monkeypatch):
    warnings = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: warnings.append(args[2]))
    gui.num_pipes_per_layer_spin.lineEdit().setText("3.5")
    gui._validate_and_start()
    assert gui.config is None
    assert warnings and "正整数" in warnings[-1]


def test_period_must_align_with_physical_step(gui, monkeypatch):
    warnings = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: warnings.append(args[2]))
    gui.replenishment_period_seconds_spin.setValue(100.5)
    gui._validate_and_start()
    assert gui.config is None
    assert warnings and "整数倍" in warnings[-1]


def test_explicit_preview_uses_edited_geometry_and_ramp_degrees(gui):
    gui.num_pipes_per_layer_spin.setValue(8)
    gui.num_lanes_per_pipe_spin.setValue(4)
    gui.ramp_same_layer_degree_spin.setValue(3)
    gui.ramp_adjacent_layer_degree_spin.setValue(5)
    gui.ramp_express_layer_degree_spin.setValue(0)
    gui._rebuild_preview_env(include_dynamic=False)
    assert len(gui.env.pipelines) == 48
    assert all(pipe.num_lanes == 4 for pipe in gui.env.pipelines)
    assert gui.env.ramp_same_layer_degree == 3
    assert gui.env.ramp_adjacent_layer_degree == 5
    assert gui.env.ramp_express_layer_degree == 0


def test_ramp_degree_controls_match_supported_range(gui):
    for name in ("ramp_same_layer_degree", "ramp_adjacent_layer_degree", "ramp_express_layer_degree"):
        spin = getattr(gui, name + "_spin")
        assert spin.minimum() == 0
        assert spin.maximum() == 8


def test_metropolis_dynamic_density_hint_explains_fixed_open_state(gui):
    assert hasattr(gui, "dynamic_density_hint")
    assert not gui.dynamic_density_hint.isHidden()
    assert "初始开启" in gui.dynamic_density_hint.text()
    assert "保持" in gui.dynamic_density_hint.text()
    assert "道路数量" in gui.dynamic_density_hint.text()
    gui.airspace_mode_combo.setCurrentIndex(gui.airspace_mode_combo.findData("standard"))
    assert gui.dynamic_density_hint.isHidden()


def test_explicit_preview_refresh_preserves_valid_manual_locations(gui):
    gui.destination_mode_combo.setCurrentIndex(1)
    for row in range(gui.dest_table.rowCount()):
        gui.dest_table.cellWidget(row, 0).findChild(QCheckBox).setChecked(row == 5)
    gui.num_pickup_spin.setValue(1)
    gui.pickup_rows[0].pipe_combo.setCurrentIndex(2)
    gui.pickup_rows[0].pos_spin.setValue(1234.0)
    gui.num_pipes_per_layer_spin.setValue(8)
    gui.num_lanes_per_pipe_spin.setValue(4)
    gui._refresh_dynamic_preview()
    assert len(gui.env.pipelines) == 48
    assert gui.dest_table.rowCount() == 48
    assert gui._get_dest_pipe_ids() == [5]
    assert gui.pickup_rows[0].get_location() == {"pipe_id": 2, "pos_1d": 1234.0}
    gui._validate_and_start()
    assert gui.config["pickup_locations"] == [{"pipe_id": 2, "pos_1d": 1234.0}]


@pytest.mark.parametrize("control_name, text", [
    ("ramp_same_layer_degree_spin", "3.5"),
    ("max_sim_time_seconds_spin", ""),
    ("drain_timeout_seconds_spin", "-1"),
])
def test_invalid_ramp_and_duration_text_is_not_silently_accepted(gui, monkeypatch, control_name, text):
    warnings = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: warnings.append(args[2]))
    getattr(gui, control_name).lineEdit().setText(text)
    gui._validate_and_start()
    assert gui.config is None
    assert warnings


@pytest.mark.parametrize("mode", ["standard", "regional_6x6"])
def test_legacy_mode_uses_real_pipe_endpoints_and_one_second_flow(gui, monkeypatch, mode):
    monkeypatch.setattr(gui, "_rebuild_preview_env", lambda **kwargs: None)
    gui.sim_dt_seconds_spin.setValue(.1)
    gui.routing_mode_combo.setCurrentIndex(2)
    gui.dest_table.cellWidget(60, 8).setValue(3)
    gui.dest_table.cellWidget(60, 9).setValue(1234.)
    gui.airspace_mode_combo.setCurrentIndex(gui.airspace_mode_combo.findData(mode))
    assert gui.destination_mode_combo.currentData() == "manual"
    assert not gui.destination_mode_combo.isEnabled()
    assert gui.sim_dt_seconds_spin.value() == 1.
    assert not gui.sim_dt_seconds_spin.isEnabled()
    assert not gui.routing_mode_combo.isEnabled()
    assert not gui.num_pipes_per_layer_spin.isEnabled()
    assert not gui.dest_table.cellWidget(60, 8).isEnabled()
    assert gui.dest_table.cellWidget(60, 8).value() == 0
    assert gui.dest_table.cellWidget(60, 9).value() == gui.dest_table.cellWidget(60, 9).maximum()
    assert gui.use_ray_check.isEnabled()
    gui._validate_and_start()
    assert gui.config is not None
    assert gui.config['dest_pipe_ids']
    assert gui.config['sim_dt_seconds'] == 1.


def test_database_source_uses_legacy_manual_controls_instead_of_new_runtime_options(gui, monkeypatch):
    gui.sim_dt_seconds_spin.setValue(.1)
    gui.airspace_source_combo.setCurrentIndex(gui.airspace_source_combo.findData('postgres'))
    assert gui.destination_mode_combo.currentData() == 'manual'
    assert gui.sim_dt_seconds_spin.value() == 1.
    assert not gui.routing_mode_combo.isEnabled()
    assert not gui.destination_mode_combo.isEnabled()
    assert not gui.inter_dynamic_combo.isEnabled() and not gui.intra_dynamic_combo.isEnabled()
    db = database_environment(monkeypatch)
    monkeypatch.setattr('uav_sim.app._build_airspace_from_config', lambda config: db)
    gui._refresh_dynamic_preview()
    gui._validate_and_start()
    assert gui.config is not None and gui.config['dest_pipe_ids']
    assert gui.config['airspace_source'] == 'postgres'
    assert gui.config['sim_dt_seconds'] == 1.


def test_returning_to_enhanced_metropolis_reenables_its_new_controls(gui, monkeypatch):
    monkeypatch.setattr(gui, '_rebuild_preview_env', lambda **kwargs: None)
    gui.airspace_mode_combo.setCurrentIndex(gui.airspace_mode_combo.findData('standard'))
    gui.airspace_mode_combo.setCurrentIndex(gui.airspace_mode_combo.findData('metropolis'))
    assert gui.routing_mode_combo.isEnabled()
    assert gui.sim_dt_seconds_spin.isEnabled()
    assert gui.destination_mode_combo.isEnabled()
    assert gui.destination_mode_combo.currentData() == 'random'


def test_legacy_endpoint_preview_does_not_claim_a_specific_arrival_lane(gui, monkeypatch):
    monkeypatch.setattr(gui, '_rebuild_preview_env', lambda **kwargs: None)
    gui.airspace_mode_combo.setCurrentIndex(gui.airspace_mode_combo.findData('standard'))
    calls = []
    monkeypatch.setattr(gui.canvas, 'refresh', lambda *args: calls.append(args))
    gui._refresh_preview()
    assert calls[-1][2]
    assert all('lane_id' not in point for point in calls[-1][2])


def database_environment(monkeypatch):
    from tests.test_app_database_airspace_source import FakeRepository
    from uav_sim import app as app_module
    monkeypatch.setattr(app_module, 'PostgresAirspaceRepository', FakeRepository)
    return app_module._build_airspace_from_config(dict(airspace_source='postgres'))


def test_database_source_selection_does_not_connect_and_start_requires_refresh(gui, monkeypatch):
    calls, warnings = [], []
    monkeypatch.setattr('uav_sim.app._build_airspace_from_config', lambda config: calls.append(config))
    monkeypatch.setattr(QMessageBox, 'warning', lambda *args: warnings.append(args[2]))
    gui.airspace_source_combo.setCurrentIndex(gui.airspace_source_combo.findData('postgres'))
    assert calls == []
    gui._validate_and_start()
    assert gui.config is None and calls == []
    assert warnings and '刷新' in warnings[-1]


def test_explicit_database_refresh_replaces_generated_geometry_and_old_selections(gui, monkeypatch):
    db = database_environment(monkeypatch)
    calls = []
    monkeypatch.setattr('uav_sim.app._build_airspace_from_config', lambda config: calls.append(config) or db)
    gui.num_pickup_spin.setValue(1)
    gui.pickup_rows[0].pos_spin.setValue(1234.)
    for row in range(gui.dest_table.rowCount()):
        gui.dest_table.cellWidget(row, 0).findChild(QCheckBox).setChecked(row == 60)
    gui.airspace_source_combo.setCurrentIndex(gui.airspace_source_combo.findData('postgres'))
    gui._refresh_dynamic_preview()
    assert calls and calls[-1]['database']['fallback_to_generated'] is False
    assert gui.env is db and gui.canvas.env is db
    assert gui.dest_table.rowCount() == len(db.pipelines)
    assert gui._get_dest_pipe_ids()
    assert set(gui._get_dest_pipe_ids()) <= {pipe.pipe_id for pipe in db.pipelines}
    assert 60 not in gui._get_dest_pipe_ids()
    assert gui.pickup_rows[0].get_location()['pos_1d'] != 1234.
    gui._validate_and_start()
    assert gui.config is not None and gui.config['airspace_source'] == 'postgres'


def test_database_refresh_failure_keeps_retry_state_without_credentials_or_fallback(gui, monkeypatch):
    db = database_environment(monkeypatch)
    old_env, warnings = gui.env, []
    def fail(config):
        raise RuntimeError('password=should_not_be_exposed')
    monkeypatch.setattr('uav_sim.app._build_airspace_from_config', fail)
    monkeypatch.setattr(QMessageBox, 'warning', lambda *args: warnings.append(args[2]))
    gui.airspace_source_combo.setCurrentIndex(gui.airspace_source_combo.findData('postgres'))
    gui._refresh_dynamic_preview()
    assert gui.env is old_env
    assert gui.airspace_source_combo.currentData() == 'postgres'
    assert warnings and 'should_not_be_exposed' not in warnings[-1]
    gui._validate_and_start()
    assert gui.config is None
    monkeypatch.setattr('uav_sim.app._build_airspace_from_config', lambda config: db)
    gui._refresh_dynamic_preview()
    gui._validate_and_start()
    assert gui.config is not None
