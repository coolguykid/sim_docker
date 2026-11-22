import json
import pytest
from uav_sim.airspace import Airspace
from uav_sim.entities.uav import UAV
from uav_sim.simulation.runner import run_simulation
from uav_sim.visualization.frame_exporter import FrameJsonExporter


def test_sequential_run_writes_turn_timing_and_complete_frame_json(tmp_path):
    env = Airspace(mode="standard", include_dynamic=False)
    uav = UAV(0, 0, 0, 10, env, capacity=0)
    output_dir = tmp_path / "algorithm_1"
    result = run_simulation(
        env,
        [uav],
        [],
        mode="DIJKSTRA",
        max_turns=1,
        track_uav_ids=[],
        run_output_dir=output_dir,
        frame_export_enabled=True,
        run_id="test-run",
        algorithm_id="1",
    )
    timing_path = output_dir / "timing" / "turn_times.txt"
    frame_path = output_dir / "frames" / "000000-000999" / "frame_000000.json"
    manifest_path = output_dir / "manifest.json"
    assert timing_path.exists()
    assert len(timing_path.read_text(encoding="utf-8").splitlines()) == 2
    assert json.loads(frame_path.read_text(encoding="utf-8"))["uav_count"] == 1
    assert json.loads(manifest_path.read_text(encoding="utf-8"))["complete"] is True
    assert result[7]["timing"]["completed_turns"] == 1


def test_sequential_frame_export_failure_marks_manifest_incomplete(
    tmp_path, monkeypatch
):
    env = Airspace(mode="standard", include_dynamic=False)
    uav = UAV(0, 0, 0, 10, env, capacity=0)
    output_dir = tmp_path / "algorithm_1"

    def fail_write(*_args, **_kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(FrameJsonExporter, "write_uav_objects", fail_write)
    with pytest.raises(OSError, match="disk full"):
        run_simulation(
            env,
            [uav],
            [],
            mode="DIJKSTRA",
            max_turns=1,
            track_uav_ids=[],
            run_output_dir=output_dir,
            frame_export_enabled=True,
            run_id="test-run",
            algorithm_id="1",
        )
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["complete"] is False
    assert manifest["error"] == "disk full"
