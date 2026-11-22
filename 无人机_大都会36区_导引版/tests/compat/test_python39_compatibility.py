import ast
import dataclasses
import importlib.util
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = PROJECT_ROOT / "src"


def test_all_production_sources_parse_with_python39_grammar():
    failures = []
    for path in SRC_ROOT.rglob("*.py"):
        try:
            ast.parse(
                path.read_text(encoding="utf-8-sig"),
                filename=str(path),
                feature_version=(3, 9),
            )
        except SyntaxError as exc:
            failures.append(f"{path.relative_to(PROJECT_ROOT)}: {exc}")
    assert failures == []


def test_actor_snapshot_module_imports_when_dataclass_slots_is_unavailable(
        monkeypatch):
    module_path = (
        SRC_ROOT / "uav_sim" / "distributed" / "actor_airspace_snapshot.py"
    )
    real_dataclass = dataclasses.dataclass

    def python39_dataclass(*args, **kwargs):
        if "slots" in kwargs:
            raise TypeError("dataclass() got an unexpected keyword argument 'slots'")
        return real_dataclass(*args, **kwargs)

    monkeypatch.setattr(dataclasses, "dataclass", python39_dataclass)
    spec = importlib.util.spec_from_file_location(
        "uav_sim_python39_actor_snapshot_probe",
        module_path,
    )
    module = importlib.util.module_from_spec(spec)

    spec.loader.exec_module(module)

    assert "__dict__" not in module.ActorPipeline.__slots__
    assert "__dict__" not in module.ActorRamp.__slots__
