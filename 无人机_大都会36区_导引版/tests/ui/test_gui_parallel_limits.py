import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication

from uav_sim.airspace import Airspace
from uav_sim.ui.gui_config import ConfigGUI
from uav_sim.distributed.engine import DistributedEngine


def test_parallelism_controls_allow_10000_uavs_per_actor():
    app = QApplication.instance() or QApplication([])
    env = Airspace(mode="standard", include_dynamic=False)
    gui = ConfigGUI(env)

    try:
        assert gui.parallel_spin.maximum() == 10000
        assert gui.parallel_slider.maximum() == 10000
    finally:
        gui.close()
        app.processEvents()


def test_parallelism_defaults_to_300_uavs_per_actor():
    app = QApplication.instance() or QApplication([])
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    gui = ConfigGUI(env)

    try:
        assert gui.parallel_spin.value() == 300
        assert gui.parallel_slider.value() == 300
        assert DistributedEngine().parallelism_level == 300
    finally:
        gui.close()
        app.processEvents()


def test_settings_panel_can_expand_to_show_parallel_controls():
    app = QApplication.instance() or QApplication([])
    env = Airspace(mode="standard", include_dynamic=False)
    gui = ConfigGUI(env)

    try:
        assert gui.settings_scroll.minimumWidth() >= 420
        assert gui.settings_scroll.maximumWidth() >= 720
    finally:
        gui.close()
        app.processEvents()


def test_algorithm_selection_defaults_to_dijkstra_only():
    app = QApplication.instance() or QApplication([])
    env = Airspace(mode="standard", include_dynamic=False)
    gui = ConfigGUI(env)

    try:
        selected = [key for key, cb in gui.algo_checks.items() if cb.isChecked()]
        assert selected == ["1"]
    finally:
        gui.close()
        app.processEvents()


def test_multi_uav_overlay_option_defaults_off_and_is_saved():
    app = QApplication.instance() or QApplication([])
    env = Airspace(mode="standard", include_dynamic=False)
    gui = ConfigGUI(env)

    try:
        assert gui.multi_uav_trajectory_check.isChecked() is False

        gui.multi_uav_trajectory_check.setChecked(True)
        gui._validate_and_start()

        assert gui.config["plot_multi_uav_same_figure"] is True
    finally:
        gui.close()
        app.processEvents()


def test_optional_metric_checkboxes_default_off_and_are_saved():
    app = QApplication.instance() or QApplication([])
    env = Airspace(mode="standard", include_dynamic=False)
    gui = ConfigGUI(env)

    try:
        assert gui.enable_kl_check.isChecked() is False
        assert gui.enable_channel_gain_check.isChecked() is False
        assert gui.frame_export_check.isChecked() is False

        gui.enable_kl_check.setChecked(True)
        gui.enable_channel_gain_check.setChecked(True)
        gui.frame_export_check.setChecked(True)
        gui.run_output_root_input.setText("custom/runs")
        gui._validate_and_start()

        assert gui.config["enable_kl_divergence"] is True
        assert gui.config["enable_channel_gain"] is True
        assert gui.config["frame_export_enabled"] is True
        assert gui.config["run_output_root"] == "custom/runs"
    finally:
        gui.close()
        app.processEvents()


def test_topology_profile_defaults_sparse_and_legacy_dense_is_saved():
    app = QApplication.instance() or QApplication([])
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    gui = ConfigGUI(env)

    try:
        assert gui.topology_profile_combo.currentData() == "balanced_sparse_v1"
        dense_index = gui.topology_profile_combo.findData("legacy_dense")
        assert dense_index >= 0
        gui.topology_profile_combo.setCurrentIndex(dense_index)
        gui._validate_and_start()

        assert gui.config["topology_profile"] == "legacy_dense"
    finally:
        gui.close()
        app.processEvents()


def test_postgres_airspace_source_disables_generated_topology_controls(monkeypatch):
    app = QApplication.instance() or QApplication([])
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    gui = ConfigGUI(env)

    try:
        assert gui.airspace_source_combo.currentData() == "generated"
        postgres_index = gui.airspace_source_combo.findData("postgres")
        assert postgres_index >= 0
        gui.airspace_source_combo.setCurrentIndex(postgres_index)

        assert gui.airspace_mode_combo.isEnabled() is False
        assert gui.topology_profile_combo.isEnabled() is False
        assert "环境变量" in gui.database_connection_label.text()

        from tests.test_app_database_airspace_source import FakeRepository
        from uav_sim import app as app_module
        monkeypatch.setattr(app_module, 'PostgresAirspaceRepository', FakeRepository)
        database_env = app_module._build_airspace_from_config(dict(airspace_source='postgres'))
        monkeypatch.setattr(app_module, '_build_airspace_from_config', lambda config: database_env)
        gui._refresh_dynamic_preview()
        gui._validate_and_start()
        assert gui.config["airspace_source"] == "postgres"
    finally:
        gui.close()
        app.processEvents()
