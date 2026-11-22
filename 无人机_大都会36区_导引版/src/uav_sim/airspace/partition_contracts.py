"""Shared contracts for regional views of the original physical airspace."""
from dataclasses import dataclass,field
from typing import Dict,Tuple,Optional

Vec3=Tuple[float,float,float]

@dataclass(frozen=True)
class RoadNode:
    node_id:str
    region_id:int
    position:Vec3
    pipe_id:Optional[int]=None
    distance_m:Optional[float]=None

@dataclass(frozen=True)
class RoadEdge:
    edge_id:str
    source:str
    target:str
    region_id:int
    kind:str
    length_m:float
    speed_limit_mps:float
    points:Tuple[Vec3,...]
    pipe_id:Optional[int]=None
    ramp_id:Optional[int]=None
    lane_count:int=20
    offsets_start:Tuple[Vec3,...]=()
    offsets_end:Tuple[Vec3,...]=()

@dataclass
class PartitionGraph:
    nodes:Dict[str,RoadNode]
    edges:Dict[str,RoadEdge]
    regions:dict
    metadata:dict=field(default_factory=dict)
    version:int=0
    ramp_states:dict=field(default_factory=dict)

@dataclass
class RegionMap:
    region_id:int
    nodes:Dict[str,RoadNode]
    edges:Dict[str,RoadEdge]
    adjacency:dict
    version:int=0
    cost_revision:int=0

@dataclass(frozen=True)
class PortalArc:
    target:str
    cost:float
    kind:str='LOCAL'
    edge_id:Optional[str]=None

@dataclass
class PortalSummary:
    node_regions:dict
    adjacency:dict
    version:int=0
    cost_revision:int=0

@dataclass(frozen=True)
class Destination:
    node_id:str
    region_id:int
    ingress_costs:dict
    version:int=0
    pipe_id:Optional[int]=None
    lane_id:int=0
    distance_m:float=0.
    position:Vec3=(0.,0.,0.)
    cost_revision:int=0

@dataclass(frozen=True)
class RegionPlan:
    status:str
    edge_ids:Tuple[str,...]=()
    region_path:Tuple[int,...]=()
    cost:float=float('inf')
    version:int=0
    reason:str=''
