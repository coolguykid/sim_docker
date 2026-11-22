"""Normalize database rows into the simulator's dense runtime topology."""
from collections import defaultdict

import numpy as np

from uav_sim.airspace.database_records import AdaptedTopology, ExternalIdMaps
from uav_sim.airspace.environment import Pipeline, Ramp


SOURCE_PIPE_ALIASES = (
    "source_pipe_id",
    "r_in_pipe_id",
    "in_pipe_id",
    "pipe_a_id",
)
TARGET_PIPE_ALIASES = (
    "target_pipe_id",
    "r_out_pipe_id",
    "out_pipe_id",
    "pipe_b_id",
)


def normalize_database_row(row):
    return {
        str(key).strip().lower(): value
        for key, value in dict(row).items()
    }


def _as_point(value, field_name):
    if isinstance(value, str):
        raw = value.strip().strip("{}[]()")
        value = [part.strip() for part in raw.split(",") if part.strip()]
    if value is None or len(value) != 3:
        raise ValueError("{} 必须是三维坐标".format(field_name))
    return np.asarray([float(item) for item in value], dtype=float)


def _as_regions(value, fallback):
    if value is None:
        return (int(fallback),)
    if isinstance(value, (int, float, str)):
        return (int(value),)
    return tuple(int(item) for item in value)


def _first_present(row, aliases):
    for name in aliases:
        if name in row and row[name] is not None:
            return row[name]
    return None


class DatabaseTopologyAdapter:
    def __init__(self, strict=True, length_tolerance=1e-3,
                 coordinate_tolerance=1e-6,
                 derive_lane_width_from_pipe_width=True):
        self.strict = bool(strict)
        self.length_tolerance = float(length_tolerance)
        self.coordinate_tolerance = float(coordinate_tolerance)
        self.derive_lane_width_from_pipe_width = bool(
            derive_lane_width_from_pipe_width
        )

    def adapt(self, region_rows, layer_rows, pipe_rows, lane_rows, ramp_rows,
              topology_version=0):
        diagnostics = []
        regions = self._adapt_regions(region_rows)
        layer_configs, layers_by_external_id = self._adapt_layers(layer_rows)
        pipelines, pipe_external_to_internal = self._adapt_pipelines(
            pipe_rows,
            lane_rows,
            regions,
            layers_by_external_id,
            diagnostics,
        )
        ramps, ramp_external_to_internal = self._adapt_ramps(
            ramp_rows,
            pipelines,
            pipe_external_to_internal,
            diagnostics,
        )
        continuation_edges = self._build_continuation_edges(pipelines)
        id_maps = ExternalIdMaps(
            pipe_external_to_internal=dict(pipe_external_to_internal),
            pipe_internal_to_external={
                internal: external
                for external, internal in pipe_external_to_internal.items()
            },
            ramp_external_to_internal=dict(ramp_external_to_internal),
            ramp_internal_to_external={
                internal: external
                for external, internal in ramp_external_to_internal.items()
            },
        )
        return AdaptedTopology(
            regions=regions,
            layer_configs=layer_configs,
            pipelines=pipelines,
            ramps=ramps,
            continuation_edges=continuation_edges,
            id_maps=id_maps,
            topology_version=int(topology_version),
            diagnostics=diagnostics,
        )

    def adapt_from_repository_result(self, result):
        return self.adapt(
            region_rows=result.get("regions", ()),
            layer_rows=result.get("layers", ()),
            pipe_rows=result.get("pipes", ()),
            lane_rows=result.get("lanes", ()),
            ramp_rows=result.get("ramps", ()),
            topology_version=result.get("derived_topology_version", 0),
        )

    @staticmethod
    def _adapt_regions(rows):
        regions = []
        for index, raw in enumerate(rows or ()):
            row = normalize_database_row(raw)
            region_id = int(row.get("region_id", index))
            regions.append({
                "region_id": region_id,
                "row": int(row.get("row", row.get("region_row", 0))),
                "col": int(row.get("col", row.get("region_col", index))),
                "center_x": float(row.get("center_x", 0.0)),
                "center_y": float(row.get("center_y", 0.0)),
            })
        return regions

    @staticmethod
    def _adapt_layers(rows):
        configs = []
        by_external_id = {}
        for index, raw in enumerate(rows or ()):
            row = normalize_database_row(raw)
            external_id = int(row.get("layer_id", index))
            config = {
                "height": float(row.get("height", row.get("layer_height", 0.0))),
                "angle": float(row.get("angle", 0.0)),
                "speed": float(row.get("speed_limit", row.get("speed", 1.0))),
                "external_layer_id": external_id,
                "layer_index": index,
            }
            configs.append(config)
            by_external_id[external_id] = config
        return configs, by_external_id

    def _adapt_pipelines(self, pipe_rows, lane_rows, regions,
                         layers_by_external_id, diagnostics):
        normalized = [normalize_database_row(row) for row in (pipe_rows or ())]
        external_to_internal = {}
        for internal_id, row in enumerate(normalized):
            external_id = int(row["pipe_id"])
            if external_id in external_to_internal:
                raise ValueError("重复 Pipe_id: {}".format(external_id))
            external_to_internal[external_id] = internal_id

        region_lookup = {int(item["region_id"]): item for item in regions}
        pipelines = []
        for internal_id, row in enumerate(normalized):
            external_id = int(row["pipe_id"])
            start = _as_point(row.get("start_pt"), "Start_pt")
            end = _as_point(row.get("end_pt"), "End_pt")
            computed_length = float(np.linalg.norm(end - start))
            self._validate_length(
                "Pipe", external_id, row.get("length"), computed_length,
                diagnostics)
            external_layer_id = int(row.get("layer_id", 0))
            layer = layers_by_external_id.get(external_layer_id, {})
            region_id = int(row.get("region_id", 0))
            region = region_lookup.get(region_id, {})
            num_lanes = max(1, int(row.get("lane_number", 1)))
            pipe_width = row.get("pipe_width")
            lane_width = 10.0
            if pipe_width is not None and self.derive_lane_width_from_pipe_width:
                lane_width = float(pipe_width) / num_lanes
                diagnostics.append({
                    "type": "derived_lane_width",
                    "external_pipe_id": external_id,
                    "lane_width": lane_width,
                })
            pipeline = Pipeline(
                pipe_id=internal_id,
                start_pt=start,
                end_pt=end,
                height=float(start[2] if len(start) >= 3 else layer.get("height", 0.0)),
                max_speed=float(row.get("speed_limit", layer.get("speed", 1.0))),
                num_lanes=num_lanes,
                lane_width=lane_width,
                region_id=region_id,
                region_row=int(region.get("row", 0)),
                region_col=int(region.get("col", 0)),
                local_pipe_id=internal_id,
                local_layer_index=int(row.get("local_layer_index", internal_id)),
                layer_index=int(layer.get("layer_index", 0)),
                d_safe=row.get("d_safe"),
                max_capacity=row.get("max_capacity"),
                pipe_width=pipe_width,
                external_pipe_id=external_id,
                external_layer_id=external_layer_id,
                external_start_node_id=row.get("start_pt_name"),
                external_end_node_id=row.get("end_pt_name"),
                start_node_regions=_as_regions(
                    row.get("start_pt_region"), region_id),
                end_node_regions=_as_regions(
                    row.get("end_pt_region"), region_id),
            )
            pipelines.append(pipeline)

        lanes_by_pipe = defaultdict(list)
        for raw in lane_rows or ():
            row = normalize_database_row(raw)
            lanes_by_pipe[int(row["pipe_id"])].append(row)
        for external_pipe_id, rows in lanes_by_pipe.items():
            if external_pipe_id not in external_to_internal:
                self._problem(
                    "Lane 引用了不存在的 Pipe_id: {}".format(external_pipe_id),
                    diagnostics,
                    {"type": "missing_lane_pipe", "external_pipe_id": external_pipe_id},
                )
                continue
            specs = []
            for local_lane_id, row in enumerate(rows):
                start = _as_point(row.get("start_pt"), "Lane.Start_pt")
                end = _as_point(row.get("end_pt"), "Lane.End_pt")
                computed_length = float(np.linalg.norm(end - start))
                self._validate_length(
                    "Lane", row.get("lane_id", local_lane_id),
                    row.get("length"), computed_length, diagnostics)
                specs.append({
                    "lane_id": local_lane_id,
                    "external_lane_id": row.get("lane_id", local_lane_id),
                    "start_pt": start,
                    "end_pt": end,
                })
            pipelines[external_to_internal[external_pipe_id]].set_database_lanes(specs)
        return pipelines, external_to_internal

    def _adapt_ramps(self, ramp_rows, pipelines, pipe_external_to_internal,
                     diagnostics):
        ramps = []
        external_to_internal = {}
        for internal_id, raw in enumerate(ramp_rows or ()):
            row = normalize_database_row(raw)
            external_id = int(row["ramp_id"])
            source_external = _first_present(row, SOURCE_PIPE_ALIASES)
            target_external = _first_present(row, TARGET_PIPE_ALIASES)
            if source_external is None:
                raise ValueError(
                    "Ramp {} 缺少入口管道字段（{}）".format(
                        external_id, ", ".join(SOURCE_PIPE_ALIASES))
                )
            if target_external is None:
                raise ValueError(
                    "Ramp {} 缺少出口管道字段（{}）".format(
                        external_id, ", ".join(TARGET_PIPE_ALIASES))
                )
            source_external = int(source_external)
            target_external = int(target_external)
            if source_external not in pipe_external_to_internal:
                raise ValueError("Ramp {} 的入口管道 {} 不存在".format(
                    external_id, source_external))
            if target_external not in pipe_external_to_internal:
                raise ValueError("Ramp {} 的出口管道 {} 不存在".format(
                    external_id, target_external))
            source = pipelines[pipe_external_to_internal[source_external]]
            target = pipelines[pipe_external_to_internal[target_external]]
            entry = _as_point(row.get("r_in"), "R_in")
            exit_pt = _as_point(row.get("r_out"), "R_out")
            computed_length = float(np.linalg.norm(exit_pt - entry))
            self._validate_length(
                "Ramp", external_id, row.get("length"), computed_length,
                diagnostics)
            ramp_type_value = str(row.get("type", "1")).strip().lower()
            if ramp_type_value in ("1", "inter", "inter_layer"):
                ramp_type = "inter_layer"
            elif source.region_id != target.region_id:
                ramp_type = "cross_region_intra_layer"
            else:
                ramp_type = "intra_layer"
            ramp = Ramp(
                ramp_id=internal_id,
                pipe_a=source,
                pipe_b=target,
                pos_a_3d=entry,
                pos_b_3d=exit_pt,
                time_penalty=None,
                is_dynamic=bool(row.get("is_dynamic", False)),
                ramp_type=ramp_type,
                is_cross_region=source.region_id != target.region_id,
                is_bidirectional=False,
                external_ramp_id=external_id,
                pair_id=row.get("pair_id"),
                external_entry_node_id=row.get("r_in_name"),
                external_exit_node_id=row.get("r_out_name"),
                external_region_id=row.get("region_id"),
            )
            ramp.is_active = bool(int(row.get("is_open", 1)))
            ramps.append(ramp)
            external_to_internal[external_id] = internal_id
        return ramps, external_to_internal

    def _build_continuation_edges(self, pipelines):
        starts = defaultdict(list)
        ends = defaultdict(list)
        for pipe in pipelines:
            if pipe.external_start_node_id is not None:
                starts[pipe.external_start_node_id].append(pipe)
            if pipe.external_end_node_id is not None:
                ends[pipe.external_end_node_id].append(pipe)
        edges = []
        for external_node_id, sources in ends.items():
            for source in sources:
                for target in starts.get(external_node_id, ()):
                    if source.pipe_id == target.pipe_id:
                        continue
                    if not np.allclose(
                            source.end_pt, target.start_pt,
                            atol=self.coordinate_tolerance, rtol=0.0):
                        continue
                    same_or_shared_region = (
                        source.region_id == target.region_id
                        or bool(set(source.end_node_regions)
                                & set(target.start_node_regions))
                    )
                    if not same_or_shared_region:
                        continue
                    edges.append((
                        int(source.pipe_id),
                        int(target.pipe_id),
                        external_node_id,
                        "CONTINUE",
                    ))
        return edges

    def _validate_length(self, record_type, external_id, supplied,
                         computed, diagnostics):
        if supplied is None:
            return
        supplied = float(supplied)
        tolerance = max(
            self.length_tolerance,
            self.length_tolerance * max(1.0, abs(computed)),
        )
        if abs(supplied - computed) <= tolerance:
            return
        message = (
            "{} {} 的 Length={} 与坐标计算值={} 不一致".format(
                record_type, external_id, supplied, computed)
        )
        self._problem(message, diagnostics, {
            "type": "length_mismatch",
            "record_type": record_type,
            "external_id": external_id,
            "supplied_length": supplied,
            "computed_length": computed,
        })

    def _problem(self, message, diagnostics, diagnostic):
        if self.strict:
            raise ValueError(message)
        diagnostics.append(diagnostic)
