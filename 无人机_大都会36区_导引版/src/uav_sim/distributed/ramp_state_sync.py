"""Extension points for future external ramp-state polling.

The simulator owns no network policy here. A future HTTP/WebSocket/database
adapter only needs to implement ``fetch_update`` and return the versioned
contract defined by :mod:`uav_sim.airspace.routing_state`.
"""
from typing import Optional, Protocol

from uav_sim.airspace.routing_state import ExternalRampStateUpdate


class RampStateProvider(Protocol):
    def fetch_update(
            self,
            current_turn: int,
            since_source_version: int,
            region_id: Optional[int] = None) -> Optional[ExternalRampStateUpdate]:
        """Return a newer state update, or ``None`` when nothing changed."""


class NullRampStateProvider:
    """Default provider used until an external system is configured."""

    def fetch_update(self, current_turn, since_source_version, region_id=None):
        return None


class RampStatePoller:
    """Tracks the latest external source version across polling calls."""

    def __init__(self, provider, region_id=None):
        self.provider = provider
        self.region_id = region_id
        self.last_source_version = -1

    def poll(self, current_turn):
        update = self.provider.fetch_update(
            current_turn=int(current_turn),
            since_source_version=int(self.last_source_version),
            region_id=self.region_id,
        )
        if update is None:
            return None
        if int(update.source_version) <= self.last_source_version:
            return None
        return update

    def acknowledge(self, source_version):
        self.last_source_version = max(
            self.last_source_version,
            int(source_version),
        )
