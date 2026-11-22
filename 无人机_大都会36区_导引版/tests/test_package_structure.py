import importlib
import importlib.util
from pathlib import Path


def test_new_package_layout_imports_and_main_entrypoint_exists():
    importable_modules = [
        "uav_sim.app",
        "uav_sim.cli",
        "uav_sim.airspace.environment",
        "uav_sim.entities.uav",
        "uav_sim.algorithms.registry",
        "uav_sim.utils.trace_logger",
    ]
    dependency_bound_modules = [
        "uav_sim.distributed.orchestrator",
        "uav_sim.simulation.runner",
        "uav_sim.ui.gui_config",
    ]
    for module in importable_modules:
        importlib.import_module(module)
    for module in dependency_bound_modules:
        assert importlib.util.find_spec(module) is not None
    assert Path("main.py").is_file()
