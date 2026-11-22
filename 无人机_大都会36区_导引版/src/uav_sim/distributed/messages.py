"""
distributed_messages.py — lightweight messages for low-memory distributed runs.

These dataclasses are intentionally small and pickle-friendly. They are used to
move only UAV state, route requests, and route updates between Ray actors
instead of repeatedly broadcasting heavy environment/model objects.
"""
from dataclasses import asdict, dataclass, field
import sys
from typing import Optional


def _message_dataclass(cls):
    if sys.version_info >= (3, 10):
        return dataclass(slots=True)(cls)
    return dataclass(cls)


@_message_dataclass
class CompactUAVState:
    uav_id: int
    pipe_id: int
    lane_id: int
    pos_1d: float
    speed: float
    state: str
    phase: str
    dest_pipe_id: int
    current_ramp_id: Optional[int] = None
    gaussian_intensity: float = 0.0
    spawn_turn: int = 0
    max_accel: float = 10.0
    max_decel: float = 10.0
    last_acceleration: float = 0.0

    def to_dict(self):
        return asdict(self)


@_message_dataclass
class RouteRequest:
    uav_id: int
    current_pipe_id: int
    pos_1d: float
    dest_pipe_id: int
    dest_pos: float
    phase: str
    spawn_turn: int
    current_turn: int
    speed: float = 0.0
    max_accel: float = 10.0
    max_decel: float = 10.0
    mandatory_waypoints: Optional[list[tuple[int, float]]] = None
    final_dest_pipe_id: Optional[int] = None


@_message_dataclass
class RouteUpdate:
    uav_id: int
    raw_path: list[str]
    dest_pos: float
    phase: str
    status: str = "ok"
    node_info_patch: dict = field(default_factory=dict)
    diagnostics: dict = field(default_factory=dict)

    def to_dict(self):
        return asdict(self)


def as_route_request(value):
    if isinstance(value, RouteRequest):
        return value
    return RouteRequest(**value)


def as_route_update(value):
    if isinstance(value, RouteUpdate):
        return value
    return RouteUpdate(**value)


def uav_to_compact_state(uav):
    return CompactUAVState(
        uav_id=uav.uav_id,
        pipe_id=uav.current_pipe_id,
        lane_id=uav.current_lane_id,
        pos_1d=float(uav.pos_1d),
        speed=float(uav.speed),
        max_accel=float(getattr(uav, "max_accel", 10.0)),
        max_decel=float(getattr(uav, "max_decel", 10.0)),
        last_acceleration=float(getattr(uav, "last_acceleration", 0.0)),
        state=uav.state,
        phase=uav.phase,
        dest_pipe_id=uav.dest_pipe_id,
        current_ramp_id=getattr(uav, "current_ramp_id", None),
        gaussian_intensity=float(getattr(uav, "gaussian_intensity", 0.0)),
        spawn_turn=int(getattr(uav, "spawn_turn", 0)),
    )
