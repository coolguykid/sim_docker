from uav_sim.airspace import Airspace
from uav_sim.algorithms.normalization import normalize_distance, normalize_height_diff


def test_distance_normalization_stays_comparable_across_modes():
    standard = Airspace(mode="standard", include_dynamic=False)
    metro = Airspace(mode="metropolis", include_dynamic=False)

    assert 0.0 < normalize_distance(standard, 5000.0) <= 1.0
    assert 0.0 < normalize_distance(metro, 30000.0) <= 1.0


def test_height_normalization_uses_env_z_span():
    env = Airspace(mode="metropolis", include_dynamic=False)

    assert normalize_height_diff(env, 600.0) == 1.0
    assert normalize_height_diff(env, 300.0) == 0.5

