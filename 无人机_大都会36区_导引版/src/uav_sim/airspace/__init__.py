"""Airspace environment and scale configuration."""

from uav_sim.airspace.config import AirspaceMode, AirspaceScaleConfig, get_airspace_config
from uav_sim.airspace.environment import Airspace, PickupPoint
from uav_sim.airspace.database_topology_adapter import DatabaseTopologyAdapter
from uav_sim.airspace.database_records import AdaptedTopology, ExternalIdMaps
from uav_sim.airspace.registry import AirspaceRegistry

__all__ = [
    "Airspace",
    "PickupPoint",
    "AirspaceMode",
    "AirspaceScaleConfig",
    "AirspaceRegistry",
    "DatabaseTopologyAdapter",
    "AdaptedTopology",
    "ExternalIdMaps",
    "get_airspace_config",
]
