"""
trace_logger.py — per-turn UAV trace export.

The logger writes a tab-separated txt file for selected UAV IDs. It is kept
small and dependency-free so serial and Ray orchestration can both use it.
"""
import os


class TurnTraceLogger:
    HEADER = (
        "algo_id\tmode\tturn\tuav_id\tstate\tpipe_id\tlane_id\tpos_1d\t"
        "speed\tmax_accel\tmax_decel\tlast_acceleration\taction\tx\ty\tz\tcurrent_ramp_id\t"
        "external_pipe_id\texternal_ramp_id\tramp_length_m\tramp_remaining_seconds\tphase\tsim_time_seconds\n"
    )

    def __init__(self, uav_ids, path):
        self.uav_ids = list(uav_ids or [])
        self.path = str(path or "").strip()
        self._fh = None
        if not self.uav_ids or not self.path:
            return
        directory = os.path.dirname(os.path.abspath(self.path))
        if directory:
            os.makedirs(directory, exist_ok=True)
        write_header = not os.path.exists(self.path) or os.path.getsize(self.path) == 0
        self._fh = open(self.path, "a", encoding="utf-8")
        if write_header:
            self._fh.write(self.HEADER)

    @property
    def enabled(self):
        return self._fh is not None

    def log_turn(self, algo_id, mode, turn, uav_updates, env=None):
        if not self.enabled:
            return
        for uid in self.uav_ids:
            info = uav_updates.get(uid)
            if info is None:
                continue
            x, y, z = self._position_3d(info, env)
            self._fh.write(
                f"{algo_id}\t{mode}\t{turn}\t{uid}\t"
                f"{info.get('state', '')}\t"
                f"{info.get('pipe_id', '')}\t"
                f"{info.get('lane_id', '')}\t"
                f"{float(info.get('pos_1d', 0.0)):.3f}\t"
                f"{float(info.get('speed', 0.0)):.3f}\t"
                f"{float(info.get('max_accel', 10.0)):.3f}\t"
                f"{float(info.get('max_decel', 10.0)):.3f}\t"
                f"{float(info.get('last_acceleration', 0.0)):.3f}\t"
                f"{info.get('last_action', '')}\t"
                f"{x:.3f}\t{y:.3f}\t{z:.3f}\t"
                f"{info.get('current_ramp_id', '')}\t"
                f"{info.get('external_pipe_id', '')}\t"
                f"{info.get('external_ramp_id', '')}\t"
                f"{float(info.get('ramp_length_m', 0.0)):.3f}\t"
                f"{float(info.get('ramp_remaining_seconds', 0.0)):.3f}\t"
                f"{info.get('phase', '')}\t{info.get('sim_time_seconds', '')}\n"
            )
        self._fh.flush()

    def log_uav_objects(self, algo_id, mode, turn, uavs, env):
        if not self.enabled:
            return
        updates = {}
        by_uid = {u.uav_id: u for u in uavs}
        for uid in self.uav_ids:
            u = by_uid.get(uid)
            if u is None:
                continue
            pipe = env.pipelines[u.current_pipe_id]
            pos_3d = pipe.start_pt + pipe.direction * u.pos_1d
            ramp_id = getattr(u, "current_ramp_id", None)
            ramp = (
                getattr(env, "ramp_by_id", {}).get(int(ramp_id))
                if ramp_id is not None else None
            )
            updates[uid] = {
                "state": u.state,
                "pipe_id": u.current_pipe_id,
                "lane_id": u.current_lane_id,
                "pos_1d": u.pos_1d,
                "speed": u.speed,
                "max_accel": getattr(u, "max_accel", 10.0),
                "max_decel": getattr(u, "max_decel", 10.0),
                "last_acceleration": getattr(u, "last_acceleration", 0.0),
                "last_action": getattr(u, "last_action", ""),
                "current_ramp_id": getattr(u, "current_ramp_id", ""),
                "external_pipe_id": getattr(
                    pipe, "external_pipe_id", pipe.pipe_id),
                "external_ramp_id": (
                    "" if ramp is None else getattr(
                        ramp, "external_ramp_id", ramp.ramp_id)
                ),
                "ramp_length_m": 0.0 if ramp is None else float(ramp.length),
                "ramp_remaining_seconds": max(
                    0.0, float(getattr(u, "switch_timer", 0))
                ),
                "pos_3d": pos_3d.tolist(),
            }
        self.log_turn(algo_id, mode, turn, updates, env=None)

    @staticmethod
    def _position_3d(info, env):
        pos_3d = info.get("pos_3d")
        if pos_3d is not None:
            return float(pos_3d[0]), float(pos_3d[1]), float(pos_3d[2])
        if env is not None and info.get("pipe_id") is not None:
            pipe = env.pipelines[int(info["pipe_id"])]
            pos = pipe.start_pt + pipe.direction * float(info.get("pos_1d", 0.0))
            return float(pos[0]), float(pos[1]), float(pos[2])
        return 0.0, 0.0, 0.0

    def close(self):
        if self._fh is not None:
            self._fh.close()
            self._fh = None
