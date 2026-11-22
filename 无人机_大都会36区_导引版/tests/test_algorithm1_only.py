from pathlib import Path
import ast


def test_only_algorithm_one_is_registered():
    from uav_sim.algorithms.registry import ALGORITHM_REGISTRY, ALGO_DISTRIBUTED_META

    assert set(ALGORITHM_REGISTRY) == {"1"}
    assert set(ALGO_DISTRIBUTED_META) == {"1"}


def test_production_has_no_removed_algorithm_imports():
    root = Path(__file__).resolve().parents[1] / "src" / "uav_sim"
    removed = (
        "torch",
        "uav_sim.models",
        "uav_sim.training",
        "uav_sim.algorithms.mcts",
        "uav_sim.algorithms.real_mcts",
        "uav_sim.distributed.inference_server",
    )
    imports = []
    for path in root.rglob("*.py"):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8-sig"))):
            if isinstance(node, ast.Import):
                imports.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imports.append(node.module or "")
    assert not [name for name in imports if name.startswith(removed)]


def test_removed_implementation_directories_are_absent():
    root = Path(__file__).resolve().parents[1]
    assert not (root / "src/uav_sim/models").exists()
    assert not (root / "src/uav_sim/training").exists()
    assert not list(root.rglob("*.pth"))


def test_dijkstra_closes_generated_dynamic_ramps():
    from uav_sim.airspace import Airspace
    from uav_sim.algorithms.registry import run_dijkstra_rule
    from uav_sim.entities.uav import UAV

    env = Airspace(inter_dynamic_level="low", intra_dynamic_level="low", random_seed=42)
    assert any(r.is_dynamic for r in env.ramps)
    result = run_dijkstra_rule(
        env, [UAV(0, 0, 0, 10, env, capacity=0)], [], [], max_turns=2
    )
    assert len(result) == 8
    assert all(not r.is_active for r in env.ramps if r.is_dynamic)


def test_non_dijkstra_route_modes_are_rejected():
    import pytest
    from uav_sim.airspace import Airspace
    from uav_sim.distributed.route_planning_server import RoutePlanningCore

    env = Airspace(include_dynamic=False)
    with pytest.raises(ValueError, match="DIJKSTRA"):
        RoutePlanningCore(env.get_readonly_snapshot(), routing_mode="unsupported")


def test_actor_cpu_budget_uses_available_cpus():
    from uav_sim.app import _estimate_uav_actor_num_cpus

    assert _estimate_uav_actor_num_cpus(100, 10, total_cpus=16) == 1.0


def test_main_completes_serial_simulation_and_reports(monkeypatch, tmp_path):
    import matplotlib.pyplot as plt
    from uav_sim import app
    from uav_sim.ui import gui_config

    class Config:
        def __init__(self, env):
            pass

        def run(self):
            return {
                "selected_algorithms": ["1"],
                "airspace_mode": "standard",
                "num_uavs": 3,
                "capacity_weights": {0: 1},
                "num_pickup_points": 0,
                "pickup_locations": [],
                "dest_pipe_ids": [10],
                "max_turns": 2,
                "run_output_root": str(tmp_path),
            }

    monkeypatch.setattr(gui_config, "ConfigGUI", Config)
    monkeypatch.setattr(plt, "show", lambda: None)
    try:
        app.main()
    finally:
        plt.close("all")
    assert list(tmp_path.rglob("turn_times.txt"))
