import json
import numpy as np
import pytest
import shutil

from uav_sim.airspace import Airspace
from uav_sim.visualization.frame_exporter import FrameJsonExporter


def _state(pipe_id, lane_id, pos_1d, state="FLYING"):
    return {
        "pipe_id": pipe_id,
        "region_id": 0,
        "lane_id": lane_id,
        "pos_1d": pos_1d,
        "speed": 20.0,
        "state": state,
        "phase": "TO_DEST",
        "last_acceleration": 0.0,
        "last_action": 1,
        "current_ramp_id": None,
    }


def test_frame_exporter_writes_every_uav_atomically(tmp_path):
    env = Airspace(mode="standard", include_dynamic=False)
    exporter = FrameJsonExporter(
        tmp_path,
        env,
        run_id="run-1",
        algorithm_id="1",
        routing_mode="DIJKSTRA",
        control_mode="server_authoritative",
    )

    exporter.write_frame(0, {
        0: _state(0, 1, 10.0),
        1: _state(1, 2, 20.0, state="WAITING"),
    })
    exporter.close()

    frame_path = tmp_path / "frames" / "000000-000999" / "frame_000000.json"
    frame = json.loads(frame_path.read_text(encoding="utf-8"))
    manifest = json.loads((tmp_path / "manifest.json").read_text(encoding="utf-8"))

    assert frame["uav_count"] == 2
    assert {row["uav_id"] for row in frame["uavs"]} == {0, 1}
    assert frame["uavs"][0]["position_m"]["x"] != 0.0
    assert manifest["last_complete_turn"] == 0
    assert manifest["complete"] is True
    assert not list(tmp_path.rglob("*.tmp"))


def test_frame_exporter_shards_every_thousand_turns(tmp_path):
    env = Airspace(mode="standard", include_dynamic=False)
    exporter = FrameJsonExporter(tmp_path, env, run_id="run-2", algorithm_id="1")

    exporter.write_frame(1000, {0: _state(0, 0, 0.0)})
    exporter.close()

    expected = tmp_path / "frames" / "001000-001999" / "frame_001000.json"
    assert expected.exists()


def test_frame_exporter_marks_incomplete_run(tmp_path):
    env = Airspace(mode="standard", include_dynamic=False)
    exporter = FrameJsonExporter(tmp_path, env, run_id="run-3", algorithm_id="1")

    exporter.write_frame(0, {0: _state(0, 0, 0.0)})
    exporter.close(complete=False, error="stopped")

    manifest = json.loads((tmp_path / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["complete"] is False
    assert manifest["error"] == "stopped"


def test_frame_exporter_normalizes_numpy_scalar_status_fields(tmp_path):
    env = Airspace(mode="standard", include_dynamic=False)
    exporter = FrameJsonExporter(tmp_path, env, run_id="run-4", algorithm_id="1")
    state = _state(0, 0, 0.0)
    state["last_action"] = np.int64(2)
    state["current_ramp_id"] = np.int64(3)

    exporter.write_frame(0, {np.int64(7): state})
    exporter.close()

    frame_path = tmp_path / "frames" / "000000-000999" / "frame_000000.json"
    row = json.loads(frame_path.read_text(encoding="utf-8"))["uavs"][0]
    assert row["uav_id"] == 7
    assert row["action_id"] == 2
    assert row["current_ramp_id"] == 3


def test_frame_exporter_rejects_insufficient_disk_before_writing_scene(
        tmp_path, monkeypatch):
    env = Airspace(mode="standard", include_dynamic=False)
    monkeypatch.setattr(
        shutil,
        "disk_usage",
        lambda _path: type("DiskUsage", (), {"free": 1024})(),
    )

    with pytest.raises(OSError, match="磁盘空间不足"):
        FrameJsonExporter(
            tmp_path,
            env,
            run_id="run-preflight",
            algorithm_id="1",
            expected_uav_count=10000,
            expected_frame_count=18000,
        )

    assert not (tmp_path / "scene.json").exists()
    assert not (tmp_path / "manifest.json").exists()


def test_frame_exporter_positions_uavs_on_their_actual_lane_offsets(tmp_path):
    env = Airspace(mode="standard", include_dynamic=False)
    exporter = FrameJsonExporter(tmp_path, env, run_id="run-lanes", algorithm_id="1")
    exporter.write_frame(0, {
        0: _state(0, 0, 100.0),
        1: _state(0, 1, 100.0),
    })
    exporter.close()

    frame = json.loads(
        (tmp_path / "frames" / "000000-000999" / "frame_000000.json")
        .read_text(encoding="utf-8")
    )
    first, second = frame["uavs"]
    delta_xy = np.hypot(
        first["position_m"]["x"] - second["position_m"]["x"],
        first["position_m"]["y"] - second["position_m"]["y"],
    )

    assert delta_xy == pytest.approx(env.pipelines[0].lane_width)


def test_scene_and_manifest_record_topology_profile_and_graph_size(tmp_path):
    env = Airspace(mode="regional_6x6", include_dynamic=False)
    exporter = FrameJsonExporter(
        tmp_path,
        env,
        run_id="run-topology",
        algorithm_id="1",
    )
    exporter.close()

    scene = json.loads((tmp_path / "scene.json").read_text(encoding="utf-8"))
    manifest = json.loads(
        (tmp_path / "manifest.json").read_text(encoding="utf-8")
    )

    for payload in (scene, manifest):
        assert payload["topology_profile"] == "balanced_sparse_v1"
        assert payload["pipeline_count"] == 2160
        assert payload["ramp_count"] == 5400
        assert payload["routing_node_count"] == 15120


def test_v2_scene_and_frame_export_directed_ramp_metadata(tmp_path):
    env = Airspace(
        mode="standard",
        include_dynamic=False,
        topology_profile="balanced_sparse_v1",
    )
    ramp = next(item for item in env.ramps if item.ramp_type == "inter_layer")
    exporter = FrameJsonExporter(
        tmp_path, env, run_id="run-v2", algorithm_id="1")
    state = _state(ramp.source_pipe_id, 0, ramp.dist_from_start_a, state="SWITCHING")
    state.update({
        "current_ramp_id": ramp.ramp_id,
        "ramp_length_m": ramp.length,
        "ramp_remaining_seconds": 4.0,
    })
    exporter.write_frame(0, {1: state})
    exporter.close()

    scene = json.loads((tmp_path / "scene.json").read_text(encoding="utf-8"))
    manifest = json.loads((tmp_path / "manifest.json").read_text(encoding="utf-8"))
    frame = json.loads(
        (tmp_path / "frames" / "000000-000999" / "frame_000000.json")
        .read_text(encoding="utf-8")
    )
    scene_ramp = next(item for item in scene["ramps"] if item["ramp_id"] == ramp.ramp_id)
    uav = frame["uavs"][0]

    assert scene["schema_version"] == "uav_sim.scene.v2"
    assert manifest["schema_version"] == "uav_sim.manifest.v2"
    assert manifest["frame_schema_version"] == "uav_sim.frame.v2"
    assert frame["schema_version"] == "uav_sim.frame.v2"
    assert scene_ramp["source_pipe_id"] == ramp.source_pipe_id
    assert scene_ramp["target_pipe_id"] == ramp.target_pipe_id
    assert scene_ramp["length"] == ramp.length
    assert scene_ramp["is_bidirectional"] is False
    assert "external_ramp_id" in scene_ramp
    assert uav["external_pipe_id"] == env.pipelines[ramp.source_pipe_id].external_pipe_id
    assert uav["external_ramp_id"] == ramp.external_ramp_id
    assert uav["ramp_length_m"] == ramp.length
    assert uav["ramp_remaining_seconds"] == 4.0
