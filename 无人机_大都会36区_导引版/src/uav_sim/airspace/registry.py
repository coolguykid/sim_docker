"""
airspace_registry.py — V6.0 中央状态注册表 Actor
管理所有需要跨 UAV 读写的共享可变状态。
只读结构数据（pipelines, ramps, graph）通过 ray.put() 放入 Object Store。
"""
try:
    import ray
except ModuleNotFoundError:
    ray = None


class AirspaceRegistry:
    """中央状态注册表 — 替代 Airspace 中的可变共享字段"""

    def __init__(self, num_pipelines):
        self.num_pipelines = num_pipelines
        self.topology_version = 0
        self.ramp_state_version = 0
        self.traffic_cost_version = 0
        self._external_ramp_source_version = -1
        self._ramp_regions = {}
        self._base_ramp_states = {}
        self._external_ramp_windows = []
        self._ramp_state_current_turn = 0
        self.pipe_traffic_load = {i: 0 for i in range(num_pipelines)}
        self.pipe_collisions_window = {i: 0 for i in range(num_pipelines)}
        self.pipe_collisions_multiplier = {i: 1.0 for i in range(num_pipelines)}
        self.uav_positions = {}
        self.ramp_active_states = {}
        self.pipe_traffic_history = {i: 0 for i in range(num_pipelines)}
        self.ramp_throughput_history = {}

    def register_uav(self, uav_id, info):
        self.uav_positions[uav_id] = info

    def batch_register_uavs(self, updates):
        self.uav_positions.update(updates)

    def batch_update_uav_positions(self, updates):
        for uid, info in updates.items():
            if uid in self.uav_positions:
                self.uav_positions[uid].update(info)
            else:
                self.uav_positions[uid] = info

    def get_all_uav_states(self):
        return dict(self.uav_positions)

    def get_flying_uav_count(self):
        return sum(
            (1 for v in self.uav_positions.values() if v.get("state") == "FLYING")
        )

    def get_pipe_uavs_map(self):
        pipe_map = {}
        for uid, info in self.uav_positions.items():
            if info.get("state") == "FLYING":
                pid = info["pipe_id"]
                pipe_map.setdefault(pid, []).append({"uav_id": uid, **info})
        return pipe_map

    def get_geo_fenced_pipe_uavs_map(self, pipe_windows):
        """Return UAVs only inside requested 1D pipe windows.

        Args:
            pipe_windows: [(pipe_id, center_pos_1d, radius), ...]
        """
        windows_by_pipe = {}
        for pid, center, radius in pipe_windows or []:
            windows_by_pipe.setdefault(pid, []).append((float(center), float(radius)))
        pipe_map = {}
        for uid, info in self.uav_positions.items():
            if info.get("state") != "FLYING":
                continue
            pid = info["pipe_id"]
            windows = windows_by_pipe.get(pid)
            if not windows:
                continue
            pos = float(info.get("pos_1d", 0.0))
            for center, radius in windows:
                if abs(pos - center) <= radius:
                    pipe_map.setdefault(pid, []).append({"uav_id": uid, **info})
                    break
        return pipe_map

    def batch_update_traffic(self, pipe_counts):
        for pid, count in pipe_counts.items():
            self.pipe_traffic_load[pid] = self.pipe_traffic_load.get(pid, 0) + count
            self.pipe_traffic_history[pid] = (
                self.pipe_traffic_history.get(pid, 0) + count
            )

    def get_traffic_load(self):
        return dict(self.pipe_traffic_load)

    def get_traffic_history(self):
        return dict(self.pipe_traffic_history)

    def reset_traffic_load(self):
        self.pipe_traffic_load = {i: 0 for i in range(self.num_pipelines)}

    def reset_traffic_history(self):
        self.pipe_traffic_history = {i: 0 for i in range(self.num_pipelines)}

    def get_collisions_multiplier(self):
        return dict(self.pipe_collisions_multiplier)

    def batch_update_collisions_window(self, pipe_counts):
        for pid, count in pipe_counts.items():
            self.pipe_collisions_window[pid] = (
                self.pipe_collisions_window.get(pid, 0) + count
            )

    def apply_collision_decay(self):
        changed = False
        for p_id in self.pipe_collisions_window:
            x = self.pipe_collisions_window[p_id]
            previous = self.pipe_collisions_multiplier.get(p_id, 1.0)
            updated = max(1.0, 0.7 * previous + 0.3 * 1.1**x)
            self.pipe_collisions_multiplier[p_id] = updated
            changed = changed or updated != previous
            self.pipe_collisions_window[p_id] = 0
        if changed:
            self.traffic_cost_version += 1

    def reset_collisions(self):
        self.pipe_collisions_window = {i: 0 for i in range(self.num_pipelines)}
        self.pipe_collisions_multiplier = {i: 1.0 for i in range(self.num_pipelines)}

    def init_ramp_states(self, ramp_states, ramp_regions=None):
        self._base_ramp_states = {
            int(ramp_id): bool(state) for ramp_id, state in ramp_states.items()
        }
        self.ramp_active_states = dict(self._base_ramp_states)
        self._ramp_regions = {
            int(ramp_id): tuple((int(region_id) for region_id in region_ids))
            for ramp_id, region_ids in (ramp_regions or {}).items()
        }
        self.ramp_state_version = 0
        self._external_ramp_source_version = -1
        self._external_ramp_windows = []
        self._ramp_state_current_turn = 0

    def update_ramp_states(self, ramp_states):
        for ramp_id, state in ramp_states.items():
            ramp_id = int(ramp_id)
            state = bool(state)
            self._base_ramp_states[ramp_id] = state
        self._recompute_effective_ramp_states(self._ramp_state_current_turn)
        return self.get_ramp_state_snapshot()

    def get_ramp_states(self):
        return dict(self.ramp_active_states)

    def apply_external_ramp_states(
        self,
        ramp_states,
        source_version,
        region_id=None,
        effective_from_turn=0,
        effective_until_turn=None,
        current_turn=0,
    ):
        source_version = int(source_version)
        current_turn = int(current_turn)
        effective_from_turn = int(effective_from_turn)
        if current_turn < effective_from_turn or (
            effective_until_turn is not None
            and current_turn > int(effective_until_turn)
        ):
            return {
                "accepted": False,
                "reason": "outside_effective_window",
                "version": self.ramp_state_version,
                "source_version": self._external_ramp_source_version,
            }
        if source_version <= self._external_ramp_source_version:
            return {
                "accepted": False,
                "reason": "stale_source_version",
                "version": self.ramp_state_version,
                "source_version": self._external_ramp_source_version,
            }
        normalized_region = None if region_id is None else int(region_id)
        applicable = {}
        for ramp_id, state in ramp_states.items():
            ramp_id = int(ramp_id)
            if ramp_id not in self.ramp_active_states:
                continue
            if normalized_region is not None:
                ramp_regions = self._ramp_regions.get(ramp_id, ())
                if normalized_region not in ramp_regions:
                    continue
            applicable[ramp_id] = bool(state)
        before_states = dict(self.ramp_active_states)
        self._external_ramp_source_version = source_version
        self._external_ramp_windows.append(
            {
                "source_version": source_version,
                "effective_from_turn": effective_from_turn,
                "effective_until_turn": None
                if effective_until_turn is None
                else int(effective_until_turn),
                "states": applicable,
            }
        )
        before_version = self.ramp_state_version
        self._recompute_effective_ramp_states(current_turn)
        changed_ramp_count = sum(
            (
                1
                for ramp_id in applicable
                if before_states.get(ramp_id) != self.ramp_active_states.get(ramp_id)
            )
        )
        return {
            "accepted": True,
            "reason": "applied"
            if self.ramp_state_version != before_version
            else "unchanged",
            "version": self.ramp_state_version,
            "source_version": source_version,
            "changed_ramp_count": changed_ramp_count,
        }

    def get_ramp_state_snapshot(self, region_id=None):
        if region_id is None:
            states = dict(self.ramp_active_states)
        else:
            region_id = int(region_id)
            states = {
                ramp_id: state
                for ramp_id, state in self.ramp_active_states.items()
                if region_id in self._ramp_regions.get(ramp_id, ())
            }
        return {
            "version": self.ramp_state_version,
            "source_version": self._external_ramp_source_version,
            "states": states,
        }

    def _recompute_effective_ramp_states(self, current_turn):
        current_turn = int(current_turn)
        desired = dict(self._base_ramp_states)
        active_windows = []
        for window in sorted(
            self._external_ramp_windows, key=lambda item: int(item["source_version"])
        ):
            start = int(window["effective_from_turn"])
            end = window["effective_until_turn"]
            if end is not None and current_turn > int(end):
                continue
            active_windows.append(window)
            if current_turn < start:
                continue
            desired.update(window["states"])
        self._external_ramp_windows = active_windows
        self._ramp_state_current_turn = current_turn
        if desired != self.ramp_active_states:
            self.ramp_active_states = desired
            self.ramp_state_version += 1

    def get_shared_context(self, current_turn=None):
        if current_turn is not None:
            self._recompute_effective_ramp_states(current_turn)
        return {
            "collisions_multiplier": dict(self.pipe_collisions_multiplier),
            "traffic_load": dict(self.pipe_traffic_load),
            "ramp_states": dict(self.ramp_active_states),
            "topology_version": self.topology_version,
            "ramp_state_version": self.ramp_state_version,
            "traffic_cost_version": self.traffic_cost_version,
            "external_ramp_source_version": self._external_ramp_source_version,
        }

    def init_ramp_throughput(self, ramp_ids):
        self.ramp_throughput_history = {rid: 0 for rid in ramp_ids}

    def update_ramp_throughput(self, ramp_counts):
        for rid, count in ramp_counts.items():
            self.ramp_throughput_history[rid] = (
                self.ramp_throughput_history.get(rid, 0) + count
            )

    def get_ramp_throughput(self):
        return dict(self.ramp_throughput_history)

    def reset_ramp_throughput(self):
        self.ramp_throughput_history = {rid: 0 for rid in self.ramp_throughput_history}

    def full_reset(self):
        self.pipe_traffic_load = {i: 0 for i in range(self.num_pipelines)}
        self.pipe_traffic_history = {i: 0 for i in range(self.num_pipelines)}
        self.pipe_collisions_window = {i: 0 for i in range(self.num_pipelines)}
        self.pipe_collisions_multiplier = {i: 1.0 for i in range(self.num_pipelines)}
        self.uav_positions = {}
        self.ramp_active_states = {}
        self._base_ramp_states = {}
        self._external_ramp_windows = []
        self._ramp_state_current_turn = 0
        self.ramp_state_version = 0
        self.traffic_cost_version = 0
        self._external_ramp_source_version = -1
        self.ramp_throughput_history = {}


AirspaceRegistryBase = AirspaceRegistry
if ray is not None:
    AirspaceRegistry = ray.remote(AirspaceRegistry)
