import pytest

from tests.fixtures.database_topology_rows import (
    LANE_ROWS,
    LAYER_ROWS,
    PIPE_ROWS,
    RAMP_ROWS,
    REGION_ROWS,
)
from uav_sim import app as app_module


class FakeRepository:
    @classmethod
    def from_config(cls, config):
        instance = cls()
        instance.config = config
        return instance

    def fetch_topology_rows(self):
        return {
            "regions": REGION_ROWS,
            "layers": LAYER_ROWS,
            "pipes": PIPE_ROWS,
            "lanes": LANE_ROWS,
            "ramps": RAMP_ROWS,
            "derived_topology_version": 73,
        }


class FailingRepository(FakeRepository):
    def fetch_topology_rows(self):
        raise RuntimeError("database down")


def test_generated_source_keeps_existing_airspace():
    env = app_module._build_airspace_from_config({
        "airspace_source": "generated",
        "airspace_mode": "standard",
        "inter_dynamic_level": "none",
        "intra_dynamic_level": "none",
    })

    assert env.topology_source == "generated"
    assert env.airspace_mode == "standard"


def test_postgres_source_uses_repository_and_adapter(monkeypatch):
    monkeypatch.setattr(
        app_module,
        "PostgresAirspaceRepository",
        FakeRepository,
    )

    env = app_module._build_airspace_from_config({
        "airspace_source": "postgres",
        "database": {"strict_validation": True},
    })

    assert env.topology_source == "postgres"
    assert env.topology_version == 73
    assert env.external_id_maps.pipe_external_to_internal[101] == 0


def test_postgres_failure_does_not_silently_generate_another_topology(monkeypatch):
    monkeypatch.setattr(
        app_module,
        "PostgresAirspaceRepository",
        FailingRepository,
    )

    with pytest.raises(RuntimeError, match="database down"):
        app_module._build_airspace_from_config({
            "airspace_source": "postgres",
            "database": {"fallback_to_generated": False},
        })


def test_unknown_airspace_source_is_rejected():
    with pytest.raises(ValueError, match="空域数据源"):
        app_module._build_airspace_from_config({"airspace_source": "unknown"})


def test_database_topology_never_enters_new_metropolis_runtime(monkeypatch):
    monkeypatch.setattr(app_module, 'PostgresAirspaceRepository', FakeRepository)
    config = dict(airspace_source='postgres', airspace_mode='metropolis')
    env = app_module._build_airspace_from_config(config)
    assert not app_module._is_enhanced_metropolis(env, config)


@pytest.mark.parametrize('config', [dict(airspace_mode='standard'), dict(airspace_source='postgres')])
def test_new_headless_config_rejects_legacy_flows_before_silently_changing_semantics(config, monkeypatch):
    def must_not_build(_config):
        pytest.fail('Legacy config reached enhanced runtime construction')
    monkeypatch.setattr(app_module, '_build_airspace_from_config', must_not_build)
    with pytest.raises(ValueError, match='generated metropolis'):
        app_module.run_config(config)


def test_main_dispatches_database_fixture_to_original_runner(monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    monkeypatch.setattr(app_module, 'PostgresAirspaceRepository', FakeRepository)
    config = dict(airspace_source='postgres', airspace_mode='metropolis',
                  selected_algorithms=['1'], num_uavs=1, num_pickup_points=0,
                  dest_pipe_ids=[0], capacity_weights={0:100}, use_ray=False, max_turns=1)
    env = app_module._build_airspace_from_config(config)
    monkeypatch.setattr(app_module, 'Airspace', lambda **kwargs: env)
    monkeypatch.setattr(app_module, '_build_airspace_from_config', lambda cfg: env)
    class FakeGUI:
        def __init__(self, _env):
            self.env = env
        def run(self):
            return config
    monkeypatch.setattr('uav_sim.ui.gui_config.ConfigGUI', FakeGUI)
    monkeypatch.setattr(app_module, '_build_airspace_from_config', lambda cfg: pytest.fail('Loaded DB preview must be reused'))
    calls = []
    monkeypatch.setattr(app_module, 'run_config', lambda *args, **kw: pytest.fail('DB entered enhanced runtime'))
    monkeypatch.setattr(app_module, 'generate_uavs', lambda actual, cfg: calls.append(('generate', actual.topology_source)) or [])
    monkeypatch.setattr(app_module, 'create_pickup_points_and_assign', lambda *args: [])
    monkeypatch.setattr(app_module, '_run_sequential', lambda *args, **kw: calls.append(('original', True)))
    monkeypatch.setattr('uav_sim.simulation.plotting.print_comparison_report', lambda *args, **kw: None)
    monkeypatch.setattr('uav_sim.simulation.plotting.plot_3d_trajectories', lambda *args, **kw: None)
    app_module.main()
    assert calls == [('generate', 'postgres'), ('original', True)]


@pytest.mark.parametrize('destination_ids', [[], [999]])
def test_legacy_endpoint_selection_has_actionable_validation(monkeypatch, destination_ids):
    monkeypatch.setattr(app_module, 'PostgresAirspaceRepository', FakeRepository)
    env = app_module._build_airspace_from_config(dict(airspace_source='postgres'))
    config = dict(num_uavs=1, capacity_weights={0:100}, dest_pipe_ids=destination_ids)
    with pytest.raises(ValueError, match='终点'):
        app_module.generate_uavs(env, config)
