"""Integration checks for continuous certificates behind fixed output slots."""
import pytest

from tests.enhanced.test_uav_partition_motion import fixture
from uav_sim.airspace.environment import Pipeline
from uav_sim.simulation.metropolis_runtime import MetropolisSession


def make_session(dt=1.0, mode="centralized_regional", **options):
    env, graph, _ = fixture()
    env.pipelines[1] = Pipeline(1, (10.5, 0, 100), (40, 0, 100), 100, 20, num_lanes=1)
    graph.metadata.update(
        pipe_endpoints={0: {"start": "a", "end": "b"}, 1: {"start": "e", "end": "f"}},
        pipes={
            0: dict(height=100, length_m=10, lane_count=1, speed_limit_mps=20),
            1: dict(height=100, length_m=29.5, lane_count=1, speed_limit_mps=20),
        },
        goal_specs={"f": dict(pipe_id=1, lane_id=0, distance_m=19.5)},
    )
    cfg = dict(
        execution_engine="certified_prefix",
        sim_dt_seconds=dt,
        routing_mode=mode,
        num_uavs=2,
        max_batch_size=2,
        mission_limit=2,
        replenishment_period_seconds=100,
        max_sim_time_seconds=10,
        drain_timeout_seconds=40,
        destination_count=1,
        destination_mode="manual",
        manual_destinations=[dict(node_id="f", pipe_id=1, lane_id=0, distance_m=19.5)],
    )
    cfg.update(options)
    return MetropolisSession(env, cfg, graph)


def test_admission_exception_is_recorded_as_failed_before_output_is_closed(tmp_path, monkeypatch):
    import json
    with make_session(execution_engine='certified_journey', run_output_dir=str(tmp_path)) as session:
        session._ensure_certified_traffic()
        def reject(*args, **kwargs):
            raise RuntimeError('admission failed during initialization')
        monkeypatch.setattr(session.control.prefix_reservations, 'reserve', reject)
        with pytest.raises(RuntimeError, match='admission failed'):
            session.run()
        assert session.failed == 'admission failed during initialization'
        assert session.summary()['safety_violations'] is None
    saved = json.loads((tmp_path/'summary.json').read_text(encoding='utf-8'))
    assert not saved['success'] and saved['failed']


def test_context_exception_never_leaves_a_successful_manifest(tmp_path):
    import json
    with pytest.raises(RuntimeError, match='caller failed'):
        with make_session(run_output_dir=str(tmp_path)):
            raise RuntimeError('caller failed')
    saved = json.loads((tmp_path/'summary.json').read_text(encoding='utf-8'))
    assert saved['failed'] == 'caller failed'


def test_metrics_failure_cannot_skip_backend_cleanup_or_mask_original_error(tmp_path, monkeypatch):
    import json
    session = make_session(run_output_dir=str(tmp_path))
    closed = []
    session.failed = 'original planning error'
    def unavailable():
        raise RuntimeError('worker metrics unavailable')
    monkeypatch.setattr(session.backend, 'metrics', unavailable)
    monkeypatch.setattr(session.backend, 'close', lambda: closed.append(True))
    session.close()
    saved = json.loads((tmp_path/'summary.json').read_text(encoding='utf-8'))
    assert closed == [True]
    assert saved['failed'] == 'original planning error'
    assert saved['routing']['metrics_error'] == 'worker metrics unavailable'


def test_full_export_capacity_is_checked_before_ray_backend_start(tmp_path, monkeypatch):
    from collections import namedtuple
    import uav_sim.simulation.metropolis_runtime as runtime
    Usage = namedtuple('Usage', 'total used free')
    monkeypatch.setattr('uav_sim.visualization.frame_exporter.shutil.disk_usage', lambda path: Usage(100,99,1))
    started = []
    monkeypatch.setattr(runtime, 'RegionalRoutingBackend', lambda *a, **kw: started.append(True))
    with pytest.raises(OSError, match='磁盘空间不足'):
        make_session(run_output_dir=str(tmp_path), frame_export_enabled=True)
    assert started == []


def test_unknown_execution_engine_is_rejected_instead_of_silently_mixing_models():
    with pytest.raises(ValueError, match='execution_engine'):
        make_session(execution_engine='typo')


def test_legacy_per_step_engine_cannot_be_mixed_with_live_certified_profiles():
    with make_session(execution_engine='per_step') as session:
        with pytest.raises(ValueError, match='per_step'):
            session.advance_one_step()


def test_trace_only_selection_keeps_the_actual_final_arrival_row(tmp_path):
    trace = tmp_path/'trace.txt'
    with make_session(execution_engine='certified_journey', trace_uav_ids=[0],
                      trace_log_path=str(trace), track_uav_ids=[]) as session:
        assert session.run()['success']
    assert '\tARRIVED\t' in trace.read_text(encoding='utf-8')


@pytest.mark.parametrize("mode", ["centralized_full", "centralized_regional"])
def test_certificates_reach_real_goals_and_reuse_interval_proofs(mode):
    with make_session(0.1, mode) as session:
        summary = session.run()
        assert summary["execution_engine"] == "certified_prefix"
        assert summary["success"]
        assert summary["generated"] == summary["departed"] == summary["arrived"] == 2
        # One bottom-layer fixture pipe starts in the final region already.
        assert summary["certified_prefixes"] >= 4
        assert summary["audited_steps"] > summary["certified_prefixes"]
        assert summary["continuous_phase_pairs_checked"] > 0
        assert all(r["actual_goal_node_id"] == "f" for r in session.completed)
        assert summary["active"] == summary["pending"] == summary["unfinished"] == 0


def test_fixed_grid_never_quantizes_physical_arrival_or_planning_events():
    results = []
    for dt in (1.0, 0.1):
        with make_session(dt) as session:
            result = session.run()
            assert result["execution_engine"] == "certified_prefix"
            results.append((result, session.completed))
    for a, b in zip(results[0][1], results[1][1]):
        assert a["uav_id"] == b["uav_id"]
        assert a["arrival_time_seconds"] == pytest.approx(
            b["arrival_time_seconds"], abs=1e-9
        )
        assert a["travel_seconds"] == pytest.approx(b["travel_seconds"], abs=1e-9)
    assert results[0][0]["audited_steps"] * 10 >= results[1][0]["audited_steps"] - 10


def test_progress_callback_reports_grid_time_without_requesting_route_metrics():
    with make_session(0.1) as session:
        times = []
        original = session.backend.metrics
        calls = []

        def metrics():
            calls.append(True)
            return original()

        session.backend.metrics = metrics
        summary = session.run(lambda s: times.append(s["sim_time_seconds"]))
        assert summary["execution_engine"] == "certified_prefix"
        assert times and times[0] == 0.1
        assert len(calls) < 5
        assert all(round(t * 10) == pytest.approx(t * 10) for t in times)


@pytest.mark.parametrize("dt", [1.0, 0.1])
def test_subsecond_replenishment_uses_physical_event_time_not_output_grid(dt):
    with make_session(
        dt,
        num_uavs=4,
        max_batch_size=1,
        mission_limit=4,
        replenishment_period_seconds=0.3,
        max_sim_time_seconds=2,
    ) as session:
        result = session.run()
        assert result["success"]
        assert [b["sim_time_seconds"] for b in session.batches] == pytest.approx(
            [0.0, 0.3, 0.6, 0.9]
        )
        assert all(b["generated"] <= 1 for b in session.batches)
        assert result["generated"] == result["arrived"] == 4
        assert result["peak_active"] <= 4


def test_permanently_blocked_ground_request_is_not_removed_or_reported_arrived():
    with make_session() as session:
        session.control.prefix_reservations.reserve(
            999, (), 0.0, start_position=(0.0, 0.0, 100.0), end_hold=True, cold=False
        )
        result = session.run()
        assert not result["success"]
        assert result["generated"] == 2 and result["arrived"] == 1
        assert result["pending"] == result["unfinished"] == 1
        assert result["failed"] and result["hard_collisions"] is None
        assert 0 in session.pending_uavs


def test_far_future_endpoint_hold_allows_a_safe_stationary_queue_inside_current_region():
    from dataclasses import replace

    with make_session(
        1.0, num_uavs=1, max_batch_size=1, mission_limit=1, drain_timeout_seconds=300
    ) as session:
        graph = session.graph
        # Scale the three-region straight fixture, leaving a long first local
        # corridor that is physically useful even while its exit is occupied.
        graph.nodes.update(
            {
                nid: replace(node, position=(node.position[0] * 100, 0.0, 100.0))
                for nid, node in graph.nodes.items()
            }
        )
        graph.edges.update(
            {
                eid: replace(
                    edge,
                    points=tuple((p[0] * 100, 0.0, 100.0) for p in edge.points),
                    length_m=edge.length_m * 100,
                )
                for eid, edge in graph.edges.items()
            }
        )
        session.control.prefix_reservations.reserve(
            999, (), 0.0, start_position=(1000.0, 0.0, 100.0), end_hold=True, cold=False
        )
        result = session.run()
        assert not result["success"]
        assert result["departed"] == result["active"] == 1
        state = session.active[0]._road_state
        assert 10.0 < state.position[0] < 992.0
        assert state.speed == 0.0 and state.node_id != state.goal.node_id
        assert result["partial_queue_prefixes"] > 0


def test_engine_can_advance_one_exact_output_interval_and_return_live_positions():
    with make_session(0.1, num_uavs=1, mission_limit=1, max_batch_size=1) as session:
        first = session.advance_one_step()
        assert first["sim_time_seconds"] == 0.1
        frame = session.current_frame()
        assert frame["sim_time_seconds"] == 0.1 and frame["sim_dt_seconds"] == 0.1
        assert frame["uavs"][0]["pos_3d"] == pytest.approx((0.05, 0.0, 100.0))
        for _ in range(9):
            session.advance_one_step()
        assert session.current_frame()["uavs"][0]["pos_3d"] == pytest.approx(
            (5.0, 0.0, 100.0)
        )
        result = session.run()
        assert result["success"]
