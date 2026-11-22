import inspect
import pytest
from uav_sim.airspace import Airspace
from uav_sim.distributed.orchestrator import SimulationOrchestrator
from uav_sim.entities.uav import UAV
from uav_sim.simulation import runner as runner_module


def test_optional_metrics_default_to_disabled():
    orchestrator_signature = inspect.signature(SimulationOrchestrator.__init__)
    sequential_signature = inspect.signature(runner_module.run_simulation)
    assert orchestrator_signature.parameters["enable_kl_divergence"].default is False
    assert orchestrator_signature.parameters["enable_channel_gain"].default is False
    assert sequential_signature.parameters["enable_kl_divergence"].default is False
    assert sequential_signature.parameters["enable_channel_gain"].default is False


def test_disabled_distributed_metrics_do_not_call_calculation_methods():
    orchestrator = SimulationOrchestrator.__new__(SimulationOrchestrator)
    orchestrator.enable_kl_divergence = False
    orchestrator.enable_channel_gain = False
    orchestrator._compute_kl_from_uav_updates = lambda *_: pytest.fail(
        "KL must not run"
    )
    orchestrator._compute_beamforming_gain = lambda *_: pytest.fail("gain must not run")
    kl_history = []
    gain_history = []
    orchestrator._record_optional_metrics({}, {}, kl_history, gain_history)
    assert kl_history == []
    assert gain_history == []


def test_enabled_distributed_metrics_call_each_calculation_once():
    orchestrator = SimulationOrchestrator.__new__(SimulationOrchestrator)
    orchestrator.enable_kl_divergence = True
    orchestrator.enable_channel_gain = True
    orchestrator._compute_kl_from_uav_updates = lambda updates: 1.25
    orchestrator._compute_beamforming_gain = lambda pipe_map: 2.5
    kl_history = []
    gain_history = []
    orchestrator._record_optional_metrics(
        {0: [{"uav_id": 1}]}, {1: {"state": "FLYING"}}, kl_history, gain_history
    )
    assert kl_history == [1.25]
    assert gain_history == [2.5]


def test_disabled_sequential_metrics_do_not_call_calculations(monkeypatch):
    env = Airspace(mode="standard", include_dynamic=False)
    uav = UAV(0, 0, 0, 10, env, capacity=0)
    monkeypatch.setattr(
        runner_module,
        "compute_kl_divergence",
        lambda *_: pytest.fail("KL must not run"),
    )
    result = runner_module.run_simulation(
        env,
        [uav],
        [],
        mode="DIJKSTRA",
        max_turns=1,
        track_uav_ids=[],
        enable_kl_divergence=False,
        enable_channel_gain=False,
    )
    assert result[2] == []
    assert result[5] == []


def test_enabled_channel_gain_uses_exact_closed_form():
    env = Airspace(mode="standard", include_dynamic=False)
    orchestrator = SimulationOrchestrator.__new__(SimulationOrchestrator)
    orchestrator.env = env
    value = orchestrator._compute_beamforming_gain({})
    assert value == pytest.approx(40.0 * 8 * 8 / len(env.ramps))


def test_enabled_channel_gain_uses_registry_ramp_state_snapshot():
    env = Airspace(mode="standard", include_dynamic=False)
    orchestrator = SimulationOrchestrator.__new__(SimulationOrchestrator)
    orchestrator.env = env
    states = {ramp.ramp_id: False for ramp in env.ramps}
    states[env.ramps[0].ramp_id] = True
    value = orchestrator._compute_beamforming_gain({}, ramp_states=states)
    assert value == pytest.approx(40.0 * 8 * 8)
