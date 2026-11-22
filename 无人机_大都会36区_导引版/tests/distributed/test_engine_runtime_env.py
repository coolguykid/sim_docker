from pathlib import Path

from uav_sim.distributed.engine import _build_runtime_env


def test_ray_runtime_env_includes_project_src_on_pythonpath():
    runtime_env = _build_runtime_env()
    pythonpath = runtime_env["env_vars"]["PYTHONPATH"]
    project_src = str(Path(__file__).resolve().parents[2] / "src")

    assert project_src in pythonpath.split(";")
