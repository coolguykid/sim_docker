import json


def test_cli_config_uses_existing_application_runner(tmp_path, monkeypatch):
    from uav_sim import cli

    path = tmp_path / "run.json"
    path.write_text(
        json.dumps(dict(sim_dt_seconds=0.1, routing_mode="centralized_regional"))
    )
    calls = []
    monkeypatch.setattr("uav_sim.app.run_config", lambda config: calls.append(config))
    cli.main(["--config", str(path)])
    assert calls == [dict(sim_dt_seconds=0.1, routing_mode="centralized_regional")]
