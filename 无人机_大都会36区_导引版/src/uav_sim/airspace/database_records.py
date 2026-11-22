"""Pure records produced by the database topology adapter."""
from dataclasses import dataclass
from typing import Dict, List

from uav_sim.airspace.environment import Pipeline, Ramp


@dataclass(frozen=True)
class ExternalIdMaps:
    pipe_external_to_internal: Dict[int, int]
    pipe_internal_to_external: Dict[int, int]
    ramp_external_to_internal: Dict[int, int]
    ramp_internal_to_external: Dict[int, int]


@dataclass
class AdaptedTopology:
    regions: List[dict]
    layer_configs: List[dict]
    pipelines: List[Pipeline]
    ramps: List[Ramp]
    continuation_edges: List[tuple]
    id_maps: ExternalIdMaps
    topology_version: int
    diagnostics: List[dict]
