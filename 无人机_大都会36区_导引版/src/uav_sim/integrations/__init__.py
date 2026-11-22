"""External data-source adapters."""

from uav_sim.integrations.postgres_airspace import PostgresAirspaceRepository
from uav_sim.integrations.postgres_ramp_state import PostgresRampStateProvider

__all__ = ["PostgresAirspaceRepository", "PostgresRampStateProvider"]
