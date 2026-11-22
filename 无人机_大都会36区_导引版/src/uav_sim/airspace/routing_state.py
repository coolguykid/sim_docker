"""Version contracts for route-affecting mutable state."""
from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass(frozen=True)
class RoutingStateVersion:
    topology_version: int = 0
    ramp_state_version: int = 0
    traffic_cost_version: int = 0


@dataclass(frozen=True)
class ExternalRampStateUpdate:
    states: Dict[int, bool] = field(default_factory=dict)
    source_version: int = 0
    effective_from_turn: int = 0
    effective_until_turn: Optional[int] = None
    region_id: Optional[int] = None

