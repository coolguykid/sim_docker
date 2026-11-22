"""Compact immutable-ish airspace data for UAV execution actors."""
import numpy as np


class ActorPipeline:
    """Python 3.9-compatible slotted pipeline DTO."""

    __slots__ = (
        "pipe_id", "start_pt", "end_pt", "direction", "length", "height",
        "max_speed", "num_lanes", "lane_width", "region_id", "region_row",
        "region_col", "local_pipe_id", "local_layer_index", "layer_index",
        "d_safe", "external_pipe_id", "external_start_node_id",
        "external_end_node_id",
    )

    def __init__(self, pipe_id, start_pt, end_pt, direction, length, height,
                 max_speed, num_lanes, lane_width, region_id, region_row,
                 region_col, local_pipe_id, local_layer_index, layer_index,
                 d_safe, external_pipe_id, external_start_node_id,
                 external_end_node_id):
        self.pipe_id = pipe_id
        self.start_pt = start_pt
        self.end_pt = end_pt
        self.direction = direction
        self.length = length
        self.height = height
        self.max_speed = max_speed
        self.num_lanes = num_lanes
        self.lane_width = lane_width
        self.region_id = region_id
        self.region_row = region_row
        self.region_col = region_col
        self.local_pipe_id = local_pipe_id
        self.local_layer_index = local_layer_index
        self.layer_index = layer_index
        self.d_safe = d_safe
        self.external_pipe_id = external_pipe_id
        self.external_start_node_id = external_start_node_id
        self.external_end_node_id = external_end_node_id

    @classmethod
    def from_pipeline(cls, pipe):
        return cls(
            pipe_id=int(pipe.pipe_id),
            start_pt=np.asarray(pipe.start_pt, dtype=float),
            end_pt=np.asarray(pipe.end_pt, dtype=float),
            direction=np.asarray(pipe.direction, dtype=float),
            length=float(pipe.length),
            height=int(pipe.height),
            max_speed=float(pipe.max_speed),
            num_lanes=int(pipe.num_lanes),
            lane_width=float(pipe.lane_width),
            region_id=int(getattr(pipe, "region_id", 0)),
            region_row=int(getattr(pipe, "region_row", 0)),
            region_col=int(getattr(pipe, "region_col", 0)),
            local_pipe_id=int(getattr(pipe, "local_pipe_id", pipe.pipe_id)),
            local_layer_index=int(getattr(pipe, "local_layer_index", 0)),
            layer_index=int(getattr(pipe, "layer_index", 0)),
            d_safe=(
                None
                if getattr(pipe, "d_safe", None) is None
                else float(pipe.d_safe)
            ),
            external_pipe_id=getattr(pipe, "external_pipe_id", pipe.pipe_id),
            external_start_node_id=getattr(
                pipe, "external_start_node_id", None),
            external_end_node_id=getattr(
                pipe, "external_end_node_id", None),
        )


class ActorRamp:
    """Python 3.9-compatible mutable slotted ramp DTO."""

    __slots__ = (
        "ramp_id", "source_pipe_id", "target_pipe_id", "region_a_id",
        "region_b_id", "ramp_type", "is_dynamic", "is_active",
        "is_cross_region", "length", "time_penalty", "is_bidirectional",
        "external_ramp_id", "pair_id",
    )

    def __init__(self, ramp_id, source_pipe_id, target_pipe_id, region_a_id,
                 region_b_id, ramp_type, is_dynamic, is_active,
                 is_cross_region, length, time_penalty, is_bidirectional,
                 external_ramp_id, pair_id):
        self.ramp_id = ramp_id
        self.source_pipe_id = source_pipe_id
        self.target_pipe_id = target_pipe_id
        self.region_a_id = region_a_id
        self.region_b_id = region_b_id
        self.ramp_type = ramp_type
        self.is_dynamic = is_dynamic
        self.is_active = is_active
        self.is_cross_region = is_cross_region
        self.length = length
        self.time_penalty = time_penalty
        self.is_bidirectional = is_bidirectional
        self.external_ramp_id = external_ramp_id
        self.pair_id = pair_id

    @property
    def pipe_a_id(self):
        return self.source_pipe_id

    @property
    def pipe_b_id(self):
        return self.target_pipe_id

    def travel_time_seconds(self, speed_mps, minimum_speed_mps=1.0):
        return self.length / max(float(speed_mps), float(minimum_speed_mps))

    @classmethod
    def from_ramp(cls, ramp):
        return cls(
            ramp_id=int(ramp.ramp_id),
            source_pipe_id=int(ramp.source_pipe_id),
            target_pipe_id=int(ramp.target_pipe_id),
            region_a_id=int(getattr(ramp, "region_a_id", 0)),
            region_b_id=int(getattr(ramp, "region_b_id", 0)),
            ramp_type=str(getattr(ramp, "ramp_type", "inter_layer")),
            is_dynamic=bool(getattr(ramp, "is_dynamic", False)),
            is_active=bool(getattr(ramp, "is_active", True)),
            is_cross_region=bool(getattr(ramp, "is_cross_region", False)),
            length=float(ramp.length),
            time_penalty=float(ramp.time_penalty),
            is_bidirectional=bool(getattr(ramp, "is_bidirectional", False)),
            external_ramp_id=getattr(ramp, "external_ramp_id", ramp.ramp_id),
            pair_id=getattr(ramp, "pair_id", None),
        )


def build_actor_airspace_snapshot(env):
    snapshot = dict(env.get_readonly_snapshot())
    snapshot["pipelines"] = [
        ActorPipeline.from_pipeline(pipe) for pipe in env.pipelines
    ]
    snapshot["ramps"] = [ActorRamp.from_ramp(ramp) for ramp in env.ramps]
    snapshot["graph"] = {}
    snapshot["node_info"] = {}
    snapshot["pipe_nodes"] = {}
    snapshot["route_graph_available"] = False
    return snapshot
