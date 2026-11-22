from types import SimpleNamespace
from uav_sim import app as app_module


def test_sequential_dijkstra_does_not_announce_neural_path_rebuild(monkeypatch, capsys):
    calls = {"planned": 0, "loaded": False, "ran": False}

    class DummyUav:
        def plan(self):
            calls["planned"] += 1

    def fake_algorithm(env, uavs, pickup_points, target_uavs, **kwargs):
        calls["ran"] = True
        return (-1, 0, [], 0, 0.0, [], {}, {})

    monkeypatch.setitem(
        app_module.ALGORITHM_REGISTRY,
        "1",
        ("纯 Dijkstra + 规则避障", "Pure Dijkstra + Rule Avoidance", fake_algorithm, []),
    )
    env = SimpleNamespace(
        routing_mode="DIJKSTRA", scale_config=SimpleNamespace(default_max_turns=10)
    )
    app_module._run_sequential(
        env, [DummyUav(), DummyUav()], [], {"max_turns": 10}, ["1"], [], results_dict={}
    )
    output = capsys.readouterr().out
    assert "神经网络全面重构" not in output
    assert "Dijkstra" in output
    assert calls == {"planned": 0, "loaded": False, "ran": True}
