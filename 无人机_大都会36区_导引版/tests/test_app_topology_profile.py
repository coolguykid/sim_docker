import uav_sim.app as app_module


def test_airspace_rebuild_forwards_selected_topology_profile(monkeypatch):
    captured = {}

    def fake_airspace(**kwargs):
        captured.update(kwargs)
        return object()

    monkeypatch.setattr(app_module, "Airspace", fake_airspace)

    env = app_module._build_airspace_from_config({
        "airspace_mode": "regional_6x6",
        "topology_profile": "legacy_dense",
        "inter_dynamic_level": "low",
        "intra_dynamic_level": "low",
    })

    assert env is not None
    assert captured["topology_profile"] == "legacy_dense"


def test_metropolis_honors_dynamic_geometry_instead_of_silently_disabling_it(monkeypatch):
    captured = {}
    monkeypatch.setattr(app_module, 'Airspace', lambda **kw: captured.update(kw) or object())
    app_module._build_airspace_from_config(dict(airspace_mode='metropolis', include_dynamic=True))
    assert captured['include_dynamic'] is True
