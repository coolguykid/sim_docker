from uav_sim.integrations.postgres_ramp_state import PostgresRampStateProvider


class FakeRampRepository:
    def __init__(self, result):
        self.result = result
        self.calls = []

    def fetch_ramp_states(self, since_source_version, region_id=None):
        self.calls.append((since_source_version, region_id))
        return self.result


def test_provider_maps_external_ramp_ids_and_keeps_version_contract():
    repository = FakeRampRepository({
        "rows": [
            {"ramp_id": 9001, "is_open": 1},
            {"ramp_id": 9007, "is_open": 0},
        ],
        "derived_source_version": 12,
    })
    provider = PostgresRampStateProvider(
        repository,
        external_to_internal={9001: 0, 9007: 1},
    )

    update = provider.fetch_update(
        current_turn=88,
        since_source_version=11,
        region_id=3,
    )

    assert update.states == {0: True, 1: False}
    assert update.source_version == 12
    assert update.effective_from_turn == 88
    assert update.effective_until_turn is None
    assert update.region_id == 3
    assert repository.calls == [(11, 3)]


def test_provider_returns_none_when_database_version_has_not_advanced():
    repository = FakeRampRepository({
        "rows": [{"ramp_id": 9001, "is_open": 1}],
        "derived_source_version": 7,
    })
    provider = PostgresRampStateProvider(
        repository,
        external_to_internal={9001: 0},
    )

    assert provider.fetch_update(20, since_source_version=7) is None
