from pathlib import Path
from types import SimpleNamespace

import pytest

from uav_sim.utils.simulation_timing import SimulationTimingRecorder
from uav_sim.distributed import orchestrator as orchestrator_module
from uav_sim.distributed.orchestrator import SimulationOrchestrator


def test_timing_summary_uses_completed_turn_average(tmp_path):
    path = tmp_path / "turn_times.txt"
    recorder = SimulationTimingRecorder(path, sim_dt=1.0)

    recorder.record_turn(
        turn=0,
        compute_seconds=0.4,
        export_seconds=0.1,
        total_seconds=0.5,
        active_uavs=10,
        arrived_uavs=0,
    )
    recorder.record_turn(
        turn=1,
        compute_seconds=0.5,
        export_seconds=0.2,
        total_seconds=0.7,
        active_uavs=8,
        arrived_uavs=2,
    )
    summary = recorder.close(
        initial_route_planning_seconds=2.0,
        algorithm_total_seconds=3.5,
    )

    assert summary.completed_turns == 2
    assert summary.average_turn_seconds == pytest.approx(0.6)
    assert summary.simulation_loop_seconds == pytest.approx(1.2)
    assert summary.initial_route_planning_seconds == pytest.approx(2.0)
    assert summary.algorithm_total_seconds == pytest.approx(3.5)
    assert summary.real_time_factor == pytest.approx(2.0 / 1.2)
    lines = path.read_text(encoding="utf-8").splitlines()
    assert lines[0].startswith("turn\tsim_time_s\tcompute_s")
    assert lines[1].startswith("0\t0.000\t0.400000\t0.100000\t0.500000")
    assert lines[2].startswith("1\t1.000\t0.500000\t0.200000\t0.700000")


def test_timing_recorder_handles_zero_completed_turns(tmp_path):
    recorder = SimulationTimingRecorder(tmp_path / "empty.txt")

    summary = recorder.close(algorithm_total_seconds=0.25)

    assert summary.completed_turns == 0
    assert summary.average_turn_seconds == 0.0
    assert summary.real_time_factor == 0.0


def test_distributed_algorithm_total_includes_pre_turn_route_planning(monkeypatch):
    captured = {}

    class Recorder:
        def close(self, **kwargs):
            captured.update(kwargs)
            return SimpleNamespace(
                average_turn_seconds=1.0,
                simulation_loop_seconds=3.0,
                algorithm_total_seconds=5.0,
                real_time_factor=1.0,
                to_dict=lambda: {"algorithm_total_seconds": 5.0},
            )

    orchestrator = SimulationOrchestrator.__new__(SimulationOrchestrator)
    orchestrator._frame_exporter = None
    orchestrator._timing_recorder = Recorder()
    orchestrator._algorithm_started_at = 100.0
    orchestrator.initial_route_planning_seconds = 2.0
    monkeypatch.setattr(orchestrator_module.time, "perf_counter", lambda: 103.0)

    summary = orchestrator._close_run_outputs()

    assert captured["algorithm_total_seconds"] == pytest.approx(5.0)
    assert summary["algorithm_total_seconds"] == 5.0
