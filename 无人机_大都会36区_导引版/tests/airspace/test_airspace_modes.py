from uav_sim.airspace import Airspace


def test_standard_mode_keeps_existing_scale():
    env = Airspace(mode="standard", include_dynamic=False)

    assert env.x_max == 5000
    assert env.y_max == 5000
    assert len(env.pipelines) == 60
    assert {p.height for p in env.pipelines} == {100, 200, 300, 400, 500, 600}


def test_metropolis_mode_builds_30km_airspace():
    env = Airspace(mode="metropolis", include_dynamic=False)

    assert env.x_max == 30000
    assert env.y_max == 30000
    assert len(env.pipelines) == 300
    assert env.airspace_mode == "metropolis"
    for pipe in env.pipelines:
        for pt in (pipe.start_pt, pipe.end_pt):
            assert 0 <= pt[0] <= 30000
            assert 0 <= pt[1] <= 30000
            assert pt[2] in {100, 200, 300, 400, 500, 600}


def test_snapshot_contains_scale_metadata():
    env = Airspace(mode="metropolis", include_dynamic=False)
    snap = env.get_readonly_snapshot()

    assert snap["airspace_mode"] == "metropolis"
    assert snap["xy_span"] == 30000
    assert snap["xy_diagonal"] > 40000
    assert snap["z_span"] == 600

