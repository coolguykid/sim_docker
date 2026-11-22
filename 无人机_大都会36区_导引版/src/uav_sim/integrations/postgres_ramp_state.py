"""Versioned dynamic-ramp state provider backed by PostgreSQL."""

from uav_sim.airspace.routing_state import ExternalRampStateUpdate


class PostgresRampStateProvider:
    def __init__(self, repository, external_to_internal):
        self.repository = repository
        self.external_to_internal = dict(external_to_internal or {})

    def fetch_update(self, current_turn, since_source_version, region_id=None):
        result = self.repository.fetch_ramp_states(
            since_source_version=int(since_source_version),
            region_id=region_id,
        )
        if not result:
            return None
        source_version = int(result.get("derived_source_version", 0))
        if source_version <= int(since_source_version):
            return None
        states = {}
        for row in result.get("rows", ()):
            external_id = int(row["ramp_id"])
            if external_id not in self.external_to_internal:
                continue
            states[int(self.external_to_internal[external_id])] = bool(
                int(row.get("is_open", 1))
            )
        return ExternalRampStateUpdate(
            states=states,
            source_version=source_version,
            effective_from_turn=int(current_turn),
            effective_until_turn=None,
            region_id=region_id,
        )
