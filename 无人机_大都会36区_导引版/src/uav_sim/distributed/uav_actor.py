"""
uav_actor.py — V6.0 UAV 分组 Actor
每个 Actor 管理 N 架 UAV，在独立进程中执行 Sense→Think→Act 循环。
集中低内存模式下，宏观路线由 RoutePlanningServer 计算，Actor 保留本地微观避障。
"""
try:
    import ray
except ModuleNotFoundError:
    ray = None
from uav_sim.entities.uav import UAV
from uav_sim.distributed.local_env_proxy import LocalEnvProxy
from uav_sim.distributed.messages import (
    RouteRequest,
    as_route_update,
    uav_to_compact_state,
)
from uav_sim.distributed.server_control_messages import (
    observations_from_uavs,
    commands_by_uav,
)
from uav_sim.algorithms.avoidance_features import (
    collision_threshold_for_pipe,
    find_safe_lanes,
)
from uav_sim.algorithms.avoidance_actions import desired_speed_for_action


class UAVGroupActor:
    def __init__(
        self,
        group_id,
        uav_configs,
        env_snapshot_ref,
        routing_mode="DIJKSTRA",
        control_mode="legacy_local",
        route_server=None,
    ):
        if routing_mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        self.group_id = group_id
        self.control_mode = control_mode
        self.route_server = route_server
        self._server_authoritative = control_mode == "server_authoritative"
        self._centralized_route = control_mode in (
            "centralized_route",
            "centralized_full",
            "server_authoritative",
        )
        env_snapshot = env_snapshot_ref
        self.local_env = LocalEnvProxy(env_snapshot, routing_mode)
        self.uavs = {}
        self.uav_ids = []
        self._init_uavs(uav_configs)

    def _init_uavs(self, uav_configs):
        from uav_sim.airspace import PickupPoint

        for cfg in uav_configs:
            uav = UAV(
                uav_id=cfg["uav_id"],
                start_pipe_id=cfg["start_pipe_id"],
                lane_id=cfg["lane_id"],
                dest_pipe_id=cfg["dest_pipe_id"],
                env=self.local_env,
                capacity=cfg.get("capacity", 1),
            )
            uav.final_dest_pipe_id = cfg.get("final_dest_pipe_id", cfg["dest_pipe_id"])
            uav.start_region_id = cfg.get(
                "start_region_id",
                getattr(self.local_env.pipelines[uav.start_pipe_id], "region_id", 0),
            )
            uav.current_region_id = cfg.get(
                "current_region_id",
                getattr(self.local_env.pipelines[uav.current_pipe_id], "region_id", 0),
            )
            uav.dest_region_id = cfg.get(
                "dest_region_id",
                getattr(self.local_env.pipelines[uav.dest_pipe_id], "region_id", 0),
            )
            uav.spawn_turn = cfg.get("spawn_turn", 0)
            uav.max_accel = float(cfg.get("max_accel", getattr(uav, "max_accel", 10.0)))
            uav.max_decel = float(cfg.get("max_decel", getattr(uav, "max_decel", 10.0)))
            pi = cfg.get("pickup_info")
            if pi is not None:
                pt = PickupPoint(
                    pi["pt_id"], pi["pipe_id"], pi["pos_1d"], capacity=pi["capacity"]
                )
                pt.region_id = pi.get("region_id")
                uav.pickup_pt = pt
                uav.phase = "TO_PICKUP"
                uav.dest_pipe_id = pt.pipe_id
                uav.dest_region_id = getattr(
                    self.local_env.pipelines[pt.pipe_id], "region_id", 0
                )
            elif cfg.get("capacity", 1) == 0:
                uav.phase = "TO_DEST"
                uav.pickup_pt = None
            self.uavs[cfg["uav_id"]] = uav
            uav.centralized_route_control = self._centralized_route
            uav.needs_route_replan = self._centralized_route
            self.uav_ids.append(cfg["uav_id"])

    def set_routing_mode(self, mode):
        if mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        self.local_env.routing_mode = mode

    def update_shared_state(self, collisions_multiplier, traffic_load, ramp_states):
        self.local_env.refresh_shared_state(
            collisions_multiplier, traffic_load, ramp_states
        )

    def reset_all(self):
        for uav in self.uavs.values():
            uav.reset()

    def plan_all(self, global_context=None):
        if self._centralized_route:
            return {}
        if global_context:
            self.local_env.refresh_shared_state(
                global_context.get("collisions_multiplier", {}),
                global_context.get("traffic_load", {}),
                global_context.get("ramp_states", {}),
            )
        planning_order = list(self.uavs.values())
        for uav in planning_order:
            uav.plan()
        return {}

    def build_route_requests(self, turn=0, force_all=False):
        requests = []
        for uav in self.uavs.values():
            if uav.state == "ARRIVED":
                continue
            needs_route = (
                force_all or getattr(uav, "needs_route_replan", False) or (not uav.path)
            )
            if not needs_route:
                continue
            if uav.phase == "TO_PICKUP" and uav.pickup_pt is not None:
                dest_pipe_id = uav.pickup_pt.pipe_id
                dest_pos = uav.pickup_pt.pos_1d
            else:
                dest_pipe_id = uav.dest_pipe_id
                dest_pos = self.local_env.pipelines[dest_pipe_id].length
            requests.append(
                RouteRequest(
                    uav_id=uav.uav_id,
                    current_pipe_id=uav.current_pipe_id,
                    pos_1d=float(uav.pos_1d),
                    dest_pipe_id=dest_pipe_id,
                    dest_pos=float(dest_pos),
                    phase=uav.phase,
                    spawn_turn=int(getattr(uav, "spawn_turn", 0)),
                    current_turn=int(turn),
                    speed=float(uav.speed),
                    max_accel=float(getattr(uav, "max_accel", 10.0)),
                    max_decel=float(getattr(uav, "max_decel", 10.0)),
                    mandatory_waypoints=getattr(uav, "mandatory_waypoints", None),
                    final_dest_pipe_id=getattr(uav, "final_dest_pipe_id", None),
                )
            )
        return requests

    def apply_route_updates(self, route_updates):
        applied = 0
        for raw_update in route_updates or []:
            update = as_route_update(raw_update)
            uav = self.uavs.get(update.uav_id)
            if uav is None:
                continue
            if update.node_info_patch:
                self.local_env.node_info.update(update.node_info_patch)
            uav._apply_path(update.raw_path, update.dest_pos)
            uav.needs_route_replan = False
            applied += 1
        return applied

    def report_compact_states(self):
        return [uav_to_compact_state(u).to_dict() for u in self.uavs.values()]

    def report_control_observations(self, turn=0):
        for uav in self.uavs.values():
            if uav.state == "WAITING" and turn >= uav.spawn_turn:
                uav.state = "FLYING"
        return observations_from_uavs(self.uavs.values())

    def apply_server_commands(
        self, commands, route_updates=None, turn=0, track_uav_ids=None, ramp_states=None
    ):
        if ramp_states is not None:
            self.local_env.refresh_shared_state(
                self.local_env.pipe_collisions_multiplier,
                self.local_env.pipe_traffic_load,
                ramp_states,
            )
        if route_updates:
            self.apply_route_updates(route_updates)
        command_by_uid = commands_by_uav(commands)
        track_uav_ids = set(track_uav_ids or [])
        results = {
            "uav_updates": {},
            "violations_count": 0,
            "collision_pipe_counts": {},
            "arrivals": [],
            "traffic_counts": {},
            "ramp_throughput": {},
            "trajectories": {},
            "replan_count": 0,
            "pickup_updates": {},
            "safety_modified_count": 0,
            "hold_count": 0,
            "hard_brake_count": 0,
        }
        for u in self.uavs.values():
            if u.state == "WAITING" and turn >= u.spawn_turn:
                u.state = "FLYING"
            if u.state == "FLYING":
                results["traffic_counts"][u.current_pipe_id] = (
                    results["traffic_counts"].get(u.current_pipe_id, 0) + 1
                )
        for uid, u in self.uavs.items():
            old_state = u.state
            old_phase = u.phase
            old_pickup_id = u.pickup_pt.pt_id if u.pickup_pt is not None else None
            cmd = command_by_uid.get(uid)
            if cmd is not None and bool(cmd["safety_modified"]):
                results["safety_modified_count"] += 1
            if cmd is not None and bool(cmd["hold_position"]):
                results["hold_count"] += 1
            if cmd is not None and bool(cmd["hard_brake"]):
                results["hard_brake_count"] += 1
            if u.state not in ("ARRIVED", "WAITING") and cmd is not None:
                self._execute_server_command(u, cmd)
            if (
                old_phase == "TO_PICKUP"
                and u.phase == "TO_DEST"
                and (old_pickup_id is not None)
            ):
                u.needs_route_replan = True
                results["pickup_updates"][old_pickup_id] = (
                    results["pickup_updates"].get(old_pickup_id, 0) + u.capacity
                )
            if (
                old_state == "FLYING"
                and u.state == "SWITCHING"
                and (u.path_index < len(u.path))
            ):
                ramp_id = self.local_env._extract_ramp_id_from_node(
                    u.path[u.path_index]
                )
                if (
                    ramp_id is not None
                    and 0 <= ramp_id < len(self.local_env.ramps)
                    and self.local_env.ramps[ramp_id].is_dynamic
                ):
                    results["ramp_throughput"][ramp_id] = (
                        results["ramp_throughput"].get(ramp_id, 0) + 1
                    )
            if u.state == "ARRIVED":
                results["arrivals"].append(u.uav_id)
            results["uav_updates"][u.uav_id] = {
                "pipe_id": u.current_pipe_id,
                "region_id": getattr(
                    u,
                    "current_region_id",
                    getattr(
                        self.local_env.pipelines[u.current_pipe_id], "region_id", 0
                    ),
                ),
                "lane_id": u.current_lane_id,
                "pos_1d": u.pos_1d,
                "speed": u.speed,
                "state": u.state,
                "phase": u.phase,
                "current_ramp_id": getattr(u, "current_ramp_id", None),
                "gaussian_intensity": getattr(u, "gaussian_intensity", 0.0),
                "last_action": getattr(u, "last_action", None),
                "max_accel": getattr(u, "max_accel", 10.0),
                "max_decel": getattr(u, "max_decel", 10.0),
                "last_acceleration": getattr(u, "last_acceleration", 0.0),
                **self._ramp_export_fields(u),
            }
            if u.uav_id in track_uav_ids and u.state in [
                "FLYING",
                "SWITCHING",
                "ARRIVED",
            ]:
                pipe_obj = self.local_env.pipelines[u.current_pipe_id]
                actual_3d = pipe_obj.start_pt + pipe_obj.direction * u.pos_1d
                results["trajectories"][u.uav_id] = actual_3d.tolist()
        return results

    def _execute_server_command(self, u, cmd):
        if bool(cmd["hold_position"]) or not self._has_executable_route(u):
            u.micro_block_ticks = getattr(u, "micro_block_ticks", 0) + 1
            if bool(cmd["hold_position"]):
                u.apply_speed_command(0.0)
            u.last_action = int(cmd["action_id"])
            return
        if u.state == "SWITCHING":
            self._execute_server_switching_command(u, cmd)
            return
        u.current_lane_id = int(cmd["target_lane_id"])
        if bool(cmd["hard_brake"]):
            u.apply_speed_command(0.0)
        else:
            u.apply_speed_command(float(cmd["target_speed"]))
        u.last_action = int(cmd["action_id"])
        u.micro_block_ticks = 0
        u.step([])

    def _execute_server_switching_command(self, u, cmd):
        u.last_action = int(cmd["action_id"])
        u.switch_timer -= 1
        if u.switch_timer > 0:
            u.micro_block_ticks = 0
            return
        if not u.path or u.path_index >= len(u.path):
            u.switch_timer = 1
            u.micro_block_ticks = getattr(u, "micro_block_ticks", 0) + 1
            return
        next_node = u.path[u.path_index]
        node_info = self.local_env.node_info.get(next_node)
        if node_info is None:
            u.switch_timer = 1
            u.micro_block_ticks = getattr(u, "micro_block_ticks", 0) + 1
            return
        target_pipe_id = int(node_info["pipe_id"])
        target_pos = float(node_info["dist"])
        target_pipe = self.local_env.pipelines[target_pipe_id]
        target_lane = max(0, min(target_pipe.num_lanes - 1, int(cmd["target_lane_id"])))
        u.current_lane_id = target_lane
        u._update_state_from_pipe(target_pipe_id, target_pos)
        if bool(cmd["hard_brake"]):
            u.apply_speed_command(0.0)
        else:
            u.apply_speed_command(float(cmd["target_speed"]))
        u.state = "FLYING"
        if hasattr(u, "_clear_ramp_transition"):
            u._clear_ramp_transition()
        else:
            u.current_ramp_id = None
        u.micro_block_ticks = 0

    def get_local_query_windows(self, radius=2000.0):
        windows = []
        for uav in self.uavs.values():
            if uav.state in ("ARRIVED", "WAITING"):
                continue
            windows.append((uav.current_pipe_id, float(uav.pos_1d), float(radius)))
            next_pipe_id = self._next_route_pipe_id(uav)
            if next_pipe_id is not None:
                windows.append((next_pipe_id, 0.0, float(radius)))
        return windows

    def _next_route_pipe_id(self, uav):
        if not uav.path or uav.path_index >= len(uav.path) - 1:
            return None
        next_node = uav.path[uav.path_index + 1]
        info = self.local_env.node_info.get(next_node)
        if info is None:
            return None
        return info.get("pipe_id")

    def tick(self, turn, mode, global_context):
        """
        执行一个回合的 Sense→Think→Act 循环。

        Args:
            turn: 当前回合数
            mode: 算法模式
            global_context: {
                'pipe_uavs_map': {pipe_id: [uav_info_dicts]},
                'collisions_multiplier': {pipe_id: float},
                'traffic_load': {pipe_id: int},
                'ramp_states': {ramp_id: bool},
            }

        Returns:
            {
                'uav_updates': {uav_id: {pipe_id, lane_id, pos_1d, speed, state, ...}},
                'violations_count': int,
                'collision_pipe_counts': {pipe_id: count},
                'arrivals': [uav_id, ...],
                'traffic_counts': {pipe_id: count},
                'ramp_throughput': {ramp_id: count},
                'trajectories': {uav_id: [3d_pos_array]},
                'replan_count': int,
            }
        """
        pipe_uavs_map = global_context.get("pipe_uavs_map", {})
        collisions_mult = global_context.get("collisions_multiplier", {})
        traffic_load = global_context.get("traffic_load", {})
        ramp_states = global_context.get("ramp_states", {})
        self.local_env.refresh_shared_state(collisions_mult, traffic_load, ramp_states)
        results = {
            "uav_updates": {},
            "violations_count": 0,
            "collision_pipe_counts": {},
            "arrivals": [],
            "traffic_counts": {},
            "ramp_throughput": {},
            "trajectories": {},
            "replan_count": 0,
            "pickup_updates": {},
        }
        track_uav_ids = global_context.get("track_uav_ids", set())
        for uid, uav in self.uavs.items():
            if uav.state == "WAITING" and turn >= uav.spawn_turn:
                uav.state = "FLYING"
        active_local = [u for u in self.uavs.values() if u.state == "FLYING"]
        for u in active_local:
            pid = u.current_pipe_id
            results["traffic_counts"][pid] = results["traffic_counts"].get(pid, 0) + 1
        all_uav_proxy_list = self._build_proxy_list(pipe_uavs_map)
        collisions_changed = False
        for u in active_local:
            u.avoidance_controlled_this_turn = False
            self._do_avoidance(u, pipe_uavs_map, mode)
        uav_step_order = list(self.uavs.values())
        for u in uav_step_order:
            if u.state not in ["ARRIVED", "WAITING"]:
                needs_replan = (
                    not u.avoidance_controlled_this_turn and u.was_avoidance_controlled
                )
                if needs_replan:
                    if self._centralized_route:
                        u.needs_route_replan = True
                    else:
                        u.replan_path()
                    results["replan_count"] += 1
                u.was_avoidance_controlled = u.avoidance_controlled_this_turn
                old_state = u.state
                old_phase = u.phase
                old_pickup_id = u.pickup_pt.pt_id if u.pickup_pt is not None else None
                if self._centralized_route and (not self._has_executable_route(u)):
                    pass
                else:
                    u.step(all_uav_proxy_list)
                if (
                    old_phase == "TO_PICKUP"
                    and u.phase == "TO_DEST"
                    and (old_pickup_id is not None)
                ):
                    if self._centralized_route:
                        u.needs_route_replan = True
                    results["pickup_updates"][old_pickup_id] = (
                        results["pickup_updates"].get(old_pickup_id, 0) + u.capacity
                    )
                if (
                    old_state == "FLYING"
                    and u.state == "SWITCHING"
                    and (u.path_index < len(u.path))
                ):
                    ramp_id = self.local_env._extract_ramp_id_from_node(
                        u.path[u.path_index]
                    )
                    if (
                        ramp_id is not None
                        and 0 <= ramp_id < len(self.local_env.ramps)
                        and self.local_env.ramps[ramp_id].is_dynamic
                    ):
                        results["ramp_throughput"][ramp_id] = (
                            results["ramp_throughput"].get(ramp_id, 0) + 1
                        )
                    else:
                        next_pipe_id = self.local_env.node_info[u.path[u.path_index]][
                            "pipe_id"
                        ]
                        for r in self.local_env.ramps:
                            if r.is_dynamic and (
                                r.pipe_a_id == u.current_pipe_id
                                and r.pipe_b_id == next_pipe_id
                                or (
                                    r.pipe_b_id == u.current_pipe_id
                                    and r.pipe_a_id == next_pipe_id
                                )
                            ):
                                rid = r.ramp_id
                                results["ramp_throughput"][rid] = (
                                    results["ramp_throughput"].get(rid, 0) + 1
                                )
                                break
                if u.state == "ARRIVED":
                    results["arrivals"].append(u.uav_id)
            results["uav_updates"][u.uav_id] = {
                "pipe_id": u.current_pipe_id,
                "region_id": getattr(
                    u,
                    "current_region_id",
                    getattr(
                        self.local_env.pipelines[u.current_pipe_id], "region_id", 0
                    ),
                ),
                "lane_id": u.current_lane_id,
                "pos_1d": u.pos_1d,
                "speed": u.speed,
                "state": u.state,
                "phase": u.phase,
                "current_ramp_id": getattr(u, "current_ramp_id", None),
                "gaussian_intensity": getattr(u, "gaussian_intensity", 0.0),
                "last_action": getattr(u, "last_action", None),
                "max_accel": getattr(u, "max_accel", 10.0),
                "max_decel": getattr(u, "max_decel", 10.0),
                "last_acceleration": getattr(u, "last_acceleration", 0.0),
                **self._ramp_export_fields(u),
            }
            if u.uav_id in track_uav_ids and u.state in [
                "FLYING",
                "SWITCHING",
                "ARRIVED",
            ]:
                pipe_obj = self.local_env.pipelines[u.current_pipe_id]
                actual_3d = pipe_obj.start_pt + pipe_obj.direction * u.pos_1d
                results["trajectories"][u.uav_id] = actual_3d.tolist()
        return results

    def _do_avoidance(self, u, pipe_uavs_map, mode):
        pid = u.current_pipe_id
        pipe_uavs_raw = pipe_uavs_map.get(pid, [])
        if not pipe_uavs_raw:
            return
        pipe = self.local_env.pipelines[pid]
        pipe_max_speed = pipe.max_speed
        sense_range = pipe_max_speed * 5.0
        col_th = collision_threshold_for_pipe(pipe)
        d_lead = sense_range
        lead_speed = pipe_max_speed
        for other in pipe_uavs_raw:
            if other.get("uav_id") == u.uav_id:
                continue
            if (
                other.get("lane_id") == u.current_lane_id
                and other.get("pos_1d", 0) > u.pos_1d
            ):
                d = other["pos_1d"] - u.pos_1d
                if d < d_lead:
                    d_lead = d
                    lead_speed = other.get("speed", pipe_max_speed)
        current_speed_limit = (
            min(pipe_max_speed, lead_speed) if d_lead < col_th * 15 else pipe_max_speed
        )
        if d_lead < sense_range:
            u.avoidance_controlled_this_turn = True
            safe_lanes = find_safe_lanes(
                u, pipe_uavs_raw, self.local_env.pipelines[pid].num_lanes, sense_range
            )
            if safe_lanes:
                u.current_lane_id = min(
                    safe_lanes, key=lambda x: abs(x - u.current_lane_id)
                )
                desired = desired_speed_for_action(
                    2,
                    u.speed,
                    pipe_max_speed,
                    u.max_accel,
                    u.max_decel,
                    current_speed_limit=current_speed_limit,
                )
                u.apply_speed_command(desired)
            else:
                desired = desired_speed_for_action(
                    4,
                    u.speed,
                    pipe_max_speed,
                    u.max_accel,
                    u.max_decel,
                    current_speed_limit=current_speed_limit,
                )
                u.apply_speed_command(desired)
        else:
            action = 1
            desired = desired_speed_for_action(
                action,
                u.speed,
                pipe_max_speed,
                u.max_accel,
                u.max_decel,
                current_speed_limit=current_speed_limit,
            )
            u.apply_speed_command(desired)
        safe_speed_limit = max(0.0, d_lead - col_th * 1.2 + lead_speed)
        if u.speed > safe_speed_limit:
            u.apply_speed_command(safe_speed_limit)

    @staticmethod
    def _has_executable_route(uav):
        return bool(uav.path) and uav.path_index < len(uav.path)

    def _build_proxy_list(self, pipe_uavs_map):
        """从全局 pipe_uavs_map 构建一个轻量级代理列表供 step(active_uavs) 使用"""
        proxies = []
        for pid, uav_infos in pipe_uavs_map.items():
            for info in uav_infos:
                proxies.append(_UAVProxy(info))
        for u in self.uavs.values():
            if u.state == "FLYING":
                found = False
                for p in proxies:
                    if p.uav_id == u.uav_id:
                        p.current_pipe_id = u.current_pipe_id
                        p.current_lane_id = u.current_lane_id
                        p.pos_1d = u.pos_1d
                        p.speed = u.speed
                        found = True
                        break
                if not found:
                    proxies.append(
                        _UAVProxy(
                            {
                                "uav_id": u.uav_id,
                                "pipe_id": u.current_pipe_id,
                                "lane_id": u.current_lane_id,
                                "pos_1d": u.pos_1d,
                                "speed": u.speed,
                                "state": u.state,
                            }
                        )
                    )
        return proxies

    def apply_speed_adjustments(self, adjustments):
        for uid, new_speed in adjustments.items():
            if uid in self.uavs:
                self.uavs[uid].apply_speed_command(new_speed)

    def apply_replan(self, uav_ids):
        count = 0
        for uid in uav_ids:
            if uid in self.uavs:
                if self._centralized_route:
                    self.uavs[uid].needs_route_replan = True
                else:
                    self.uavs[uid].replan_path()
                count += 1
        return count

    def _ramp_export_fields(self, u):
        pipe = self.local_env.pipelines[int(u.current_pipe_id)]
        ramp_id = getattr(u, "current_ramp_id", None)
        ramp = (
            self.local_env.ramp_by_id.get(int(ramp_id)) if ramp_id is not None else None
        )
        return {
            "external_pipe_id": getattr(pipe, "external_pipe_id", pipe.pipe_id),
            "external_ramp_id": None
            if ramp is None
            else getattr(ramp, "external_ramp_id", ramp.ramp_id),
            "ramp_length_m": 0.0 if ramp is None else float(ramp.length),
            "ramp_remaining_seconds": max(0.0, float(getattr(u, "switch_timer", 0))),
        }

    def get_uav_states(self):
        return {
            uid: {
                "pipe_id": u.current_pipe_id,
                "region_id": getattr(
                    u,
                    "current_region_id",
                    getattr(
                        self.local_env.pipelines[u.current_pipe_id], "region_id", 0
                    ),
                ),
                "lane_id": u.current_lane_id,
                "pos_1d": u.pos_1d,
                "speed": u.speed,
                "state": u.state,
                "phase": u.phase,
                "current_ramp_id": getattr(u, "current_ramp_id", None),
                "gaussian_intensity": getattr(u, "gaussian_intensity", 0.0),
                "current_congestion_percentile": getattr(
                    u, "current_congestion_percentile", 0.0
                ),
                "spawn_turn": u.spawn_turn,
                "phase": u.phase,
                "last_action": getattr(u, "last_action", None),
                "max_accel": getattr(u, "max_accel", 10.0),
                "max_decel": getattr(u, "max_decel", 10.0),
                "last_acceleration": getattr(u, "last_acceleration", 0.0),
                **self._ramp_export_fields(u),
            }
            for uid, u in self.uavs.items()
        }

    def get_uav_3d_positions(self):
        positions = {}
        for uid, u in self.uavs.items():
            if u.state not in ["ARRIVED", "WAITING"]:
                pipe = self.local_env.pipelines[u.current_pipe_id]
                pos_3d = pipe.start_pt + pipe.direction * u.pos_1d
                positions[uid] = {
                    "pos_3d": pos_3d.tolist(),
                    "gaussian_intensity": getattr(u, "gaussian_intensity", 0.0),
                }
        return positions

    def set_congestion_percentiles(self, percentiles):
        for uid, pct in percentiles.items():
            if uid in self.uavs:
                self.uavs[uid].current_congestion_percentile = pct


class _UAVProxy:
    """轻量级 UAV 代理 — 供 step(active_uavs) 中的 is_lane_empty 使用"""

    __slots__ = (
        "uav_id",
        "current_pipe_id",
        "current_lane_id",
        "pos_1d",
        "speed",
        "state",
    )

    def __init__(self, info):
        self.uav_id = info.get("uav_id", -1)
        self.current_pipe_id = info.get("pipe_id", 0)
        self.current_lane_id = info.get("lane_id", 0)
        self.pos_1d = info.get("pos_1d", 0.0)
        self.speed = info.get("speed", 0.0)
        self.state = info.get("state", "FLYING")


UAVGroupActorBase = UAVGroupActor
if ray is not None:
    UAVGroupActor = ray.remote(num_cpus=0.1)(UAVGroupActor)


class RegionalUAVRoutingGroup:
    """Worker-private per-UAV routing without the physical world or map service.

    Only the immutable portal summary is shared for the worker lifetime. Every
    aircraft owns a separate planner, which replaces its detailed region bundle
    at a boundary. The bounded result cache contains plans, never old geometry.
    """

    def __init__(self, group_id, summary):
        from collections import OrderedDict

        self.group_id = int(group_id)
        self.summary = summary
        self.planners = {}
        self._dt = None
        self._cache = OrderedDict()
        self._counts = dict(planning_calls=0, requests=0, map_loads=0, cache_hits=0)

    def update_summary(self, summary):
        if (summary.version, summary.cost_revision) != (self.summary.version, self.summary.cost_revision):
            self._cache.clear()
        self.summary = summary

    def plan_many(self, requests, region_bundles, destinations, context):
        from uav_sim.airspace.partition_contracts import RegionPlan
        from uav_sim.airspace.partition_router import RegionalDijkstra
        from uav_sim.distributed.route_planning_server import validate_partition_context

        self._dt = validate_partition_context(context, self.summary.version, self._dt)
        revision = (self.summary.version, self.summary.cost_revision)
        results = {}
        for uid, request in requests.items():
            self._counts["requests"] += 1
            rid = request["region_id"]
            local = region_bundles.get(rid)
            destination = destinations.get(request["goal_node_id"])
            if local is None:
                results[uid] = RegionPlan("NO_ROUTE", version=self.summary.version, reason="unknown current region")
                continue
            if local.region_id != rid or (local.version, local.cost_revision) != revision:
                results[uid] = RegionPlan("STALE_MAP", version=self.summary.version, reason="current bundle does not match summary")
                continue
            planner = self.planners.get(uid)
            if planner is None:
                planner = RegionalDijkstra(self.summary, local)
                self.planners[uid] = planner
                self._counts["map_loads"] += 1
            elif planner.region_map is not local and (
                planner.region_map.region_id != rid
                or (planner.region_map.version, planner.region_map.cost_revision) != revision
            ):
                planner.replace_region(local)
                self._counts["map_loads"] += 1
            # Updating every private planner must precede coalescing results.
            planner.update_summary(self.summary)
            if destination is None:
                results[uid] = RegionPlan("NO_ROUTE", version=self.summary.version, reason="unknown exact goal node")
                continue
            if (destination.version, destination.cost_revision) != revision:
                results[uid] = RegionPlan("STALE_MAP", version=self.summary.version, reason="goal descriptor does not match summary")
                continue
            key = (revision, rid, request["start_node_id"], destination.node_id)
            if key in self._cache:
                result = self._cache[key]
                self._cache.move_to_end(key)
                self._counts["cache_hits"] += 1
            else:
                self._counts["planning_calls"] += 1
                result = planner.plan(request["start_node_id"], destination)
                self._cache[key] = result
                if len(self._cache) > 2048:
                    self._cache.popitem(last=False)
            results[uid] = result
        return results

    def discard(self, uids):
        for uid in uids:
            self.planners.pop(uid, None)

    def snapshots(self):
        return {uid: dict(region_id=planner.region_map.region_id,
                          detailed_region_ids=(planner.region_map.region_id,),
                          node_count=len(planner.region_map.nodes),
                          edge_count=len(planner.region_map.edges), version=planner.region_map.version)
                for uid, planner in self.planners.items()}

    def metrics(self):
        import os
        return dict(self._counts, worker_count=1, worker_pids=(os.getpid(),),
                    private_planners=len(self.planners))


if ray is not None:
    RegionalUAVRoutingActor = ray.remote(RegionalUAVRoutingGroup)
else:
    RegionalUAVRoutingActor = None
