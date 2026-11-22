"""Atomic scene, manifest and complete per-turn UAV JSON export."""
from pathlib import Path
import json
import os
import shutil


class FrameJsonExporter:
    SCHEMA_VERSION = "uav_sim.frame.v2"
    SCENE_SCHEMA_VERSION = "uav_sim.scene.v2"
    MANIFEST_SCHEMA_VERSION = "uav_sim.manifest.v2"

    def __init__(self, output_dir, env, run_id, algorithm_id,
                 routing_mode="DIJKSTRA", control_mode="unknown",
                 shard_size=1000, sim_dt=1.0,
                 expected_uav_count=None, expected_frame_count=None):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.frames_dir = self.output_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.env = env
        self.run_id = str(run_id)
        self.algorithm_id = str(algorithm_id)
        self.routing_mode = str(routing_mode)
        self.control_mode = str(control_mode)
        self.topology_profile = str(getattr(
            getattr(env, "topology_profile", None),
            "name",
            "unknown",
        ))
        self.pipeline_count = len(getattr(env, "pipelines", []))
        self.ramp_count = len(getattr(env, "ramps", []))
        self.routing_node_count = len(getattr(env, "node_info", {}))
        self.shard_size = max(1, int(shard_size))
        self.sim_dt = float(sim_dt)
        self.last_complete_turn = -1
        self.frame_count = 0
        self._closed = False
        self.estimated_required_bytes = None
        self.free_bytes_at_start = None
        self._preflight_disk_capacity(
            expected_uav_count,
            expected_frame_count,
        )
        self._write_scene()
        self._write_manifest(complete=False, error=None)

    def _preflight_disk_capacity(self, expected_uav_count,
                                 expected_frame_count):
        if expected_uav_count is None or expected_frame_count is None:
            return
        required, free = self.check_disk_capacity(self.output_dir, expected_uav_count, expected_frame_count)
        self.estimated_required_bytes = required
        self.free_bytes_at_start = free

    @staticmethod
    def check_disk_capacity(output_dir, expected_uav_count, expected_frame_count):
        """Check before starting Ray or opening simulation output streams."""
        uav_count = max(0, int(expected_uav_count))
        frame_count = max(0, int(expected_frame_count))
        estimated_frame_bytes = 2048 + 1024 * uav_count
        estimated_scene_and_manifest = 16 * 1024 * 1024
        required = int(
            (estimated_scene_and_manifest + frame_count * estimated_frame_bytes)
            * 1.10
        )
        free = int(shutil.disk_usage(output_dir).free)
        if free < required:
            required_gib = required / (1024 ** 3)
            free_gib = free / (1024 ** 3)
            raise OSError(
                "磁盘空间不足，拒绝启动完整逐帧 JSON 导出："
                f"预计至少需要 {required_gib:.2f} GiB，"
                f"当前可用 {free_gib:.2f} GiB"
            )
        return required, free

    @staticmethod
    def _atomic_json(path, payload):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = Path(str(path) + ".tmp")
        try:
            with temp_path.open("w", encoding="utf-8") as fh:
                json.dump(payload, fh, ensure_ascii=False, separators=(",", ":"))
            os.replace(temp_path, path)
        except Exception:
            temp_path.unlink(missing_ok=True)
            raise

    @staticmethod
    def _optional_int(value):
        if value is None or value == "":
            return None
        return int(value)

    def _write_scene(self):
        partition = getattr(self.env, 'partition_scene', {})
        scene = {
            "schema_version": self.SCENE_SCHEMA_VERSION,
            "run_id": self.run_id,
            "airspace_mode": getattr(self.env, "airspace_mode", "standard"),
            "topology_profile": self.topology_profile,
            "topology_source": getattr(self.env, "topology_source", "generated"),
            "topology_version": int(getattr(self.env, "topology_version", 0)),
            "pipeline_count": self.pipeline_count,
            "ramp_count": self.ramp_count,
            "routing_node_count": self.routing_node_count,
            "unit": "meter",
            "bounds": {
                "x_min": float(self.env.x_min),
                "x_max": float(self.env.x_max),
                "y_min": float(self.env.y_min),
                "y_max": float(self.env.y_max),
                "z_min": float(self.env.z_min),
                "z_max": float(self.env.z_max),
            },
            "regions": [dict(region) for region in partition.get('regions', getattr(self.env, "regions", []))],
            "goals": partition.get('goals', []),
            "pipelines": [
                {
                    "pipe_id": int(pipe.pipe_id),
                    "external_pipe_id": getattr(
                        pipe, "external_pipe_id", pipe.pipe_id),
                    "region_id": int(getattr(pipe, "region_id", 0)),
                    "management_region_ids": partition.get('pipe_regions', {}).get(pipe.pipe_id, [getattr(pipe, 'region_id', 0)]),
                    "start": [float(value) for value in pipe.start_pt],
                    "end": [float(value) for value in pipe.end_pt],
                    "height": int(pipe.height),
                    "max_speed": float(pipe.max_speed),
                    "num_lanes": int(pipe.num_lanes),
                    "local_pipe_id": int(getattr(pipe, "local_pipe_id", pipe.pipe_id)),
                    "layer_index": int(getattr(pipe, "layer_index", 0)),
                    "d_safe": (
                        None
                        if getattr(pipe, "d_safe", None) is None
                        else float(pipe.d_safe)
                    ),
                }
                for pipe in self.env.pipelines
            ],
            "ramps": [
                {
                    "ramp_id": int(ramp.ramp_id),
                    "pipe_a_id": int(ramp.pipe_a_id),
                    "pipe_b_id": int(ramp.pipe_b_id),
                    "source_pipe_id": int(ramp.source_pipe_id),
                    "target_pipe_id": int(ramp.target_pipe_id),
                    "external_ramp_id": getattr(
                        ramp, "external_ramp_id", ramp.ramp_id),
                    "region_a_id": int(getattr(ramp, "region_a_id", 0)),
                    "region_b_id": int(getattr(ramp, "region_b_id", 0)),
                    "management_region_ids": partition.get('ramp_regions', {}).get(ramp.ramp_id, []),
                    "pos_a": [float(value) for value in ramp.pos_a],
                    "pos_b": [float(value) for value in ramp.pos_b],
                    "ramp_type": str(ramp.ramp_type),
                    "is_dynamic": bool(ramp.is_dynamic),
                    "is_cross_region": bool(getattr(ramp, "is_cross_region", False)),
                    "length": float(ramp.length),
                    "is_bidirectional": bool(
                        getattr(ramp, "is_bidirectional", False)),
                    "pair_id": getattr(ramp, "pair_id", None),
                }
                for ramp in self.env.ramps
            ],
        }
        self._atomic_json(self.output_dir / "scene.json", scene)

    def _write_manifest(self, complete, error):
        manifest = {
            "schema_version": self.MANIFEST_SCHEMA_VERSION,
            "frame_schema_version": self.SCHEMA_VERSION,
            "run_id": self.run_id,
            "algorithm_id": self.algorithm_id,
            "routing_mode": self.routing_mode,
            "control_mode": self.control_mode,
            "topology_profile": self.topology_profile,
            "topology_source": getattr(self.env, "topology_source", "generated"),
            "topology_version": int(getattr(self.env, "topology_version", 0)),
            "pipeline_count": self.pipeline_count,
            "ramp_count": self.ramp_count,
            "routing_node_count": self.routing_node_count,
            "sim_dt_seconds": self.sim_dt,
            "frame_count": self.frame_count,
            "last_complete_turn": self.last_complete_turn,
            "complete": bool(complete),
            "error": error,
            "estimated_required_bytes": self.estimated_required_bytes,
            "free_bytes_at_start": self.free_bytes_at_start,
        }
        self._atomic_json(self.output_dir / "manifest.json", manifest)

    def _frame_path(self, turn):
        turn = int(turn)
        shard_start = (turn // self.shard_size) * self.shard_size
        shard_end = shard_start + self.shard_size - 1
        shard = f"{shard_start:06d}-{shard_end:06d}"
        return self.frames_dir / shard / f"frame_{turn:06d}.json"

    def write_frame(self, turn, uav_updates, sim_time_seconds=None):
        if self._closed:
            raise RuntimeError("frame exporter is closed")
        turn = int(turn)
        sim_time_seconds = (
            turn * self.sim_dt if sim_time_seconds is None else float(sim_time_seconds)
        )
        rows = []
        for raw_uid, info in sorted(uav_updates.items(), key=lambda item: int(item[0])):
            uid = int(raw_uid)
            pipe_id = int(info.get("pipe_id", 0))
            lane_id = int(info.get("lane_id", 0))
            pos_1d = float(info.get("pos_1d", 0.0))
            pipe = self.env.pipelines[pipe_id]
            pos_3d = info.get("pos_3d")
            if pos_3d is None:
                pos_3d = pipe.start_pt + pipe.direction * pos_1d
                lane_id = max(0, min(int(pipe.num_lanes) - 1, lane_id))
                lane_offset = (
                    lane_id - (int(pipe.num_lanes) - 1) / 2.0
                ) * float(pipe.lane_width)
                pos_3d = pos_3d.copy()
                pos_3d[0] += -float(pipe.direction[1]) * lane_offset
                pos_3d[1] += float(pipe.direction[0]) * lane_offset
            current_ramp_id = self._optional_int(info.get("current_ramp_id"))
            ramp = None
            if current_ramp_id is not None:
                ramp = getattr(self.env, "ramp_by_id", {}).get(current_ramp_id)
            rows.append({
                "uav_id": uid,
                "state": str(info.get("state", "")),
                "phase": str(info.get("phase", "")),
                "region_id": int(info.get(
                    "region_id",
                    getattr(self.env.pipelines[pipe_id], "region_id", 0),
                )),
                "pipe_id": pipe_id,
                "external_pipe_id": info.get(
                    "external_pipe_id",
                    getattr(pipe, "external_pipe_id", pipe_id),
                ),
                "lane_id": lane_id,
                "pos_1d_m": pos_1d,
                "position_m": {
                    "x": float(pos_3d[0]),
                    "y": float(pos_3d[1]),
                    "z": float(pos_3d[2]),
                },
                "speed_mps": float(info.get("speed", 0.0)),
                "acceleration_mps2": float(info.get("last_acceleration", 0.0)),
                "current_ramp_id": current_ramp_id,
                "external_ramp_id": info.get(
                    "external_ramp_id",
                    None if ramp is None else getattr(
                        ramp, "external_ramp_id", ramp.ramp_id),
                ),
                "ramp_length_m": float(info.get(
                    "ramp_length_m",
                    0.0 if ramp is None else ramp.length,
                )),
                "ramp_remaining_seconds": float(info.get(
                    "ramp_remaining_seconds", 0.0)),
                "action_id": self._optional_int(info.get("last_action")),
            })
            if 'goal_node_id' in info:
                rows[-1].update(goal_node_id=info['goal_node_id'],goal_position_m=list(info['goal_position_m']),
                                arrival_time_seconds=info.get('arrival_time_seconds'))
        frame = {
            "schema_version": self.SCHEMA_VERSION,
            "run_id": self.run_id,
            "algorithm_id": self.algorithm_id,
            "routing_mode": self.routing_mode,
            "control_mode": self.control_mode,
            "turn": turn,
            "sim_time_seconds": sim_time_seconds,
            "uav_count": len(rows),
            "uavs": rows,
        }
        self._atomic_json(self._frame_path(turn), frame)
        self.last_complete_turn = turn
        self.frame_count += 1
        self._write_manifest(complete=False, error=None)

    def write_uav_objects(self, turn, uavs):
        updates = {}
        for uav in uavs:
            pipe = self.env.pipelines[uav.current_pipe_id]
            ramp_id = getattr(uav, "current_ramp_id", None)
            ramp = (
                getattr(self.env, "ramp_by_id", {}).get(int(ramp_id))
                if ramp_id is not None else None
            )
            updates[uav.uav_id] = {
                "pipe_id": uav.current_pipe_id,
                "region_id": getattr(uav, "current_region_id", 0),
                "lane_id": uav.current_lane_id,
                "pos_1d": uav.pos_1d,
                "speed": uav.speed,
                "state": uav.state,
                "phase": uav.phase,
                "last_acceleration": getattr(uav, "last_acceleration", 0.0),
                "last_action": getattr(uav, "last_action", None),
                "current_ramp_id": getattr(uav, "current_ramp_id", None),
                "external_pipe_id": getattr(
                    pipe, "external_pipe_id", pipe.pipe_id),
                "external_ramp_id": (
                    None if ramp is None else getattr(
                        ramp, "external_ramp_id", ramp.ramp_id)
                ),
                "ramp_length_m": 0.0 if ramp is None else float(ramp.length),
                "ramp_remaining_seconds": max(
                    0.0,
                    float(getattr(uav, "switch_timer", 0)) * self.sim_dt,
                ),
            }
            if getattr(uav,'_road_state',None) is not None:
                from .state_adapter import partition_update
                updates[uav.uav_id].update(partition_update(uav, self.env, turn*self.sim_dt))
        self.write_frame(turn, updates)

    def close(self, complete=True, error=None):
        if self._closed:
            return
        self._write_manifest(complete=complete and not error, error=error)
        self._closed = True
