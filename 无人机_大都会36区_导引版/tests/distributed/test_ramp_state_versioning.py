from uav_sim.airspace.registry import AirspaceRegistryBase
from uav_sim.airspace.routing_state import ExternalRampStateUpdate
from uav_sim.distributed.ramp_state_sync import RampStatePoller


def test_ramp_state_update_increments_version_only_when_state_changes():
    registry = AirspaceRegistryBase(num_pipelines=4)
    registry.init_ramp_states(
        {0: True, 1: False},
        ramp_regions={0: (0,), 1: (1,)},
    )

    first = registry.get_ramp_state_snapshot()
    unchanged = registry.apply_external_ramp_states(
        {0: True}, source_version=10, current_turn=5)
    changed = registry.apply_external_ramp_states(
        {0: False}, source_version=11, current_turn=5)

    assert unchanged["accepted"] is True
    assert unchanged["version"] == first["version"]
    assert unchanged["changed_ramp_count"] == 0
    assert changed["accepted"] is True
    assert changed["version"] == first["version"] + 1
    assert changed["changed_ramp_count"] == 1
    assert registry.get_ramp_states()[0] is False


def test_stale_external_ramp_update_is_rejected():
    registry = AirspaceRegistryBase(num_pipelines=4)
    registry.init_ramp_states({0: True}, ramp_regions={0: (3,)})

    accepted = registry.apply_external_ramp_states(
        {0: False}, source_version=5, current_turn=10)
    stale = registry.apply_external_ramp_states(
        {0: True}, source_version=4, current_turn=10)

    assert accepted["accepted"] is True
    assert stale["accepted"] is False
    assert stale["reason"] == "stale_source_version"
    assert registry.get_ramp_states()[0] is False


def test_region_snapshot_and_update_only_include_region_ramps():
    registry = AirspaceRegistryBase(num_pipelines=4)
    registry.init_ramp_states(
        {0: True, 1: False, 2: True},
        ramp_regions={0: (0,), 1: (1,), 2: (0, 1)},
    )

    snapshot = registry.get_ramp_state_snapshot(region_id=1)
    update = registry.apply_external_ramp_states(
        {0: False, 1: True, 2: False},
        source_version=8,
        region_id=1,
        current_turn=3,
    )

    assert snapshot["states"] == {1: False, 2: True}
    assert update["accepted"] is True
    assert registry.get_ramp_states() == {0: True, 1: True, 2: False}


def test_update_outside_effective_turn_window_is_rejected():
    registry = AirspaceRegistryBase(num_pipelines=4)
    registry.init_ramp_states({0: True}, ramp_regions={0: (0,)})

    too_early = registry.apply_external_ramp_states(
        {0: False},
        source_version=1,
        effective_from_turn=20,
        effective_until_turn=30,
        current_turn=19,
    )
    too_late = registry.apply_external_ramp_states(
        {0: False},
        source_version=2,
        effective_from_turn=20,
        effective_until_turn=30,
        current_turn=31,
    )

    assert too_early["accepted"] is False
    assert too_early["reason"] == "outside_effective_window"
    assert too_late["accepted"] is False
    assert too_late["reason"] == "outside_effective_window"
    assert registry.get_ramp_states()[0] is True


def test_shared_context_exposes_scalar_routing_versions():
    registry = AirspaceRegistryBase(num_pipelines=2)
    registry.init_ramp_states({0: True}, ramp_regions={0: (0,)})
    registry.apply_external_ramp_states(
        {0: False}, source_version=1, current_turn=0)

    context = registry.get_shared_context()

    assert context["ramp_state_version"] == 1
    assert context["topology_version"] == 0
    assert context["traffic_cost_version"] == 0
    assert context["ramp_states"] == {0: False}


def test_ramp_state_poller_requests_only_newer_external_versions():
    class Provider:
        def __init__(self):
            self.calls = []

        def fetch_update(self, current_turn, since_source_version, region_id=None):
            self.calls.append((current_turn, since_source_version, region_id))
            if len(self.calls) == 1:
                return ExternalRampStateUpdate(
                    states={3: False},
                    source_version=5,
                    effective_from_turn=10,
                    effective_until_turn=20,
                    region_id=region_id,
                )
            return None

    provider = Provider()
    poller = RampStatePoller(provider, region_id=7)

    first = poller.poll(10)
    poller.acknowledge(first.source_version)
    second = poller.poll(11)

    assert first.source_version == 5
    assert second is None
    assert provider.calls == [(10, -1, 7), (11, 5, 7)]


def test_ramp_state_poller_does_not_advance_until_registry_accepts_update():
    class Provider:
        def __init__(self):
            self.since_versions = []

        def fetch_update(self, current_turn, since_source_version, region_id=None):
            self.since_versions.append(since_source_version)
            return ExternalRampStateUpdate(
                states={0: False},
                source_version=9,
                effective_from_turn=20,
                effective_until_turn=30,
            )

    provider = Provider()
    poller = RampStatePoller(provider)

    assert poller.poll(19).source_version == 9
    assert poller.poll(20).source_version == 9
    assert provider.since_versions == [-1, -1]


def test_external_ramp_state_window_reverts_after_end_turn():
    registry = AirspaceRegistryBase(num_pipelines=2)
    registry.init_ramp_states({0: True}, ramp_regions={0: (0,)})
    applied = registry.apply_external_ramp_states(
        {0: False},
        source_version=1,
        effective_from_turn=10,
        effective_until_turn=12,
        current_turn=10,
    )

    during = registry.get_shared_context(current_turn=12)
    after = registry.get_shared_context(current_turn=13)

    assert applied["accepted"] is True
    assert during["ramp_states"][0] is False
    assert after["ramp_states"][0] is True
    assert after["ramp_state_version"] == 2
