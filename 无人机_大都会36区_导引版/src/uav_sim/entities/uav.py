"""
uav.py — UAV类与辅助函数
"""
import math
import random
from uav_sim.algorithms.motion_constraints import (
    DEFAULT_UAV_MAX_ACCEL,
    DEFAULT_UAV_MAX_DECEL,
    clamp_speed_change,
)


def get_uav_3d_pos(uav, env):
    """计算无人机在空域中的实时 3D 坐标"""
    if getattr(uav,'_road_state',None) is not None:
        import numpy as np
        return np.asarray(uav.sample_partition_state(getattr(env,'sim_time_seconds',0.)).position,dtype=float)
    pipe = env.pipelines[uav.current_pipe_id]
    lane = pipe.lanes[uav.current_lane_id]
    return lane.start_pt + pipe.direction * uav.pos_1d


def is_lane_empty(pipe_id, lane_id, current_pos, safe_dist, active_uavs):
    """判断目标车道附近 safe_dist 距离内是否空闲"""
    for other in active_uavs:
        if other.current_pipe_id == pipe_id and other.current_lane_id == lane_id:
            if abs(other.pos_1d - current_pos) < safe_dist:
                return False
    return True


def get_lead_info(p_id, l_idx, current_pos, pipe_uavs, sense_range, max_spd):
    """辅助函数：获取指定车道正前方飞机的距离和速度"""
    min_dist = sense_range
    lead_v = max_spd
    for other in pipe_uavs:
        if other.current_lane_id == l_idx and other.pos_1d > current_pos:
            d = other.pos_1d - current_pos
            if d < min_dist:
                min_dist = d
                lead_v = other.speed
    return (min_dist, lead_v)


class UAV:
    """无人机模型"""

    def __init__(self, uav_id, start_pipe_id, lane_id, dest_pipe_id, env, capacity=1):
        self.uav_id = uav_id
        self.env = env
        self.start_pipe_id = start_pipe_id
        self.start_lane_id = lane_id
        self.current_pipe_id = start_pipe_id
        self.current_lane_id = lane_id
        self.dest_pipe_id = dest_pipe_id
        start_pipe = env.pipelines[start_pipe_id]
        dest_pipe = env.pipelines[dest_pipe_id]
        self.start_region_id = getattr(start_pipe, "region_id", 0)
        self.current_region_id = self.start_region_id
        self.dest_region_id = getattr(dest_pipe, "region_id", 0)
        self.max_accel = DEFAULT_UAV_MAX_ACCEL
        self.max_decel = DEFAULT_UAV_MAX_DECEL
        self.last_acceleration = 0.0
        self._update_state_from_pipe(start_pipe_id, init_pos=0.0)
        self.path = []
        self.path_index = 0
        self.state = "FLYING"
        self.switch_timer = 0
        self.action_history = []
        self.avoidance_controlled_this_turn = False
        self.last_state = None
        self.last_action = None
        self.reward_this_turn = 0.0
        self.in_collision = False
        self.was_avoidance_controlled = False
        self.final_dest_pipe_id = dest_pipe_id
        self.pickup_pt = None
        self.phase = "TO_PICKUP"
        self.current_ramp_id = None
        self.ramp_entry_speed_mps = 0.0
        self.ramp_total_ticks = 0
        self.capacity = capacity
        self.gaussian_intensity = float(capacity)
        if capacity == 0:
            self.phase = "TO_DEST"
            self.pickup_pt = None
            self.gaussian_intensity = 0.0
        else:
            self.phase = "TO_PICKUP"
        self.state = "WAITING"
        self.spawn_turn = 0
        self.current_congestion_percentile = 0.0
        self.current_turn = 0

    def _update_state_from_pipe(self, pipe_id, init_pos):
        self.current_pipe_id = pipe_id
        pipe = self.env.pipelines[pipe_id]
        self.current_region_id = getattr(pipe, "region_id", 0)
        self.layer = pipe.height
        previous_speed = getattr(self, "speed", pipe.max_speed)
        self.speed = min(max(0.0, float(previous_speed)), float(pipe.max_speed))
        self.allowed_direction = pipe.direction
        self.pos_1d = init_pos

    def get_status(self):
        return f"层高Z={self.layer}m | 管道={self.current_pipe_id} | 车道={self.current_lane_id} | 允许方向={self.allowed_direction.round(2)}"

    def configure_partition_motion(self,graph,start_node_id,destination,lane_id=None):
        from .partition_motion import RoadState,node_position
        lane=self.current_lane_id if lane_id is None else lane_id
        node=graph.nodes[start_node_id]
        self._partition_graph=graph
        self._road_state=RoadState(self.uav_id,start_node_id,node.region_id,lane,
            node_position(graph,start_node_id,lane),destination,visited_regions=(node.region_id,))
        self.speed=0.;self.state='WAITING';self.phase='TO_DEST'
        self.dest_pipe_id=destination.pipe_id;self.dest_pos_1d=destination.distance_m
        self.current_region_id=node.region_id

    def set_partition_route(self,edge_ids):
        self._road_state.route=tuple(edge_ids);self._road_state.route_index=0

    def sample_partition_state(self,time_seconds):
        profile=getattr(self,'_prefix_profile',None)
        return profile.sample(time_seconds) if profile is not None else self._road_state

    def prepare_partition_step(self,acceleration,now,dt,next_region=None):
        from .partition_motion import advance
        return advance(self._road_state,self._partition_graph,acceleration,now,dt,next_region)

    def commit_partition_step(self,candidate):
        s=candidate.state;self._road_state=s
        self.state=s.status;self.speed=s.speed;self.current_region_id=s.region_id
        self.current_lane_id=s.lane_id;self.current_ramp_id=s.transit_ramp
        self.last_acceleration=s.last_acceleration
        self.layer=s.position[2]
        node=self._partition_graph.nodes[s.node_id]
        if node.pipe_id is not None:
            self.current_pipe_id=node.pipe_id
            self.pos_1d=float(node.distance_m or 0.)+s.progress

    def accelerate(self):
        return self.apply_speed_command(self.speed + self.max_accel)

    def decelerate(self):
        return self.apply_speed_command(self.speed - self.max_decel)

    def apply_speed_command(self, desired_speed, dt=1.0):
        pipe = self.env.pipelines[self.current_pipe_id]
        next_speed, acceleration = clamp_speed_change(
            current_speed=self.speed,
            desired_speed=desired_speed,
            pipe_max_speed=pipe.max_speed,
            max_accel=self.max_accel,
            max_decel=self.max_decel,
            dt=dt,
        )
        self.speed = next_speed
        self.last_acceleration = acceleration
        return next_speed

    def _ramp_switch_ticks(self, ramp_id, sim_dt=1.0, entry_speed_mps=None):
        ramp_id = int(ramp_id)
        ramp_by_id = getattr(self.env, "ramp_by_id", None)
        ramp = (
            ramp_by_id[ramp_id] if ramp_by_id is not None else self.env.ramps[ramp_id]
        )
        speed = self.speed if entry_speed_mps is None else entry_speed_mps
        seconds = ramp.travel_time_seconds(speed)
        return max(1, int(math.ceil(seconds / max(float(sim_dt), 1e-09))))

    def _clear_ramp_transition(self):
        self.current_ramp_id = None
        self.ramp_entry_speed_mps = 0.0
        self.ramp_total_ticks = 0

    def step(self, active_uavs=None):
        if self.state == "ARRIVED" or self.state == "WAITING":
            return
        if self.state == "SWITCHING":
            self.switch_timer -= 1
            if self.switch_timer <= 0:
                next_node = self.path[self.path_index]
                node_info = self.env.node_info[next_node]
                target_pipe_id = node_info["pipe_id"]
                target_pos = node_info["dist"]
                safe_lanes = []
                target_pipe = self.env.pipelines[target_pipe_id]
                for l in range(target_pipe.num_lanes):
                    if is_lane_empty(target_pipe_id, l, target_pos, 30.0, active_uavs):
                        safe_lanes.append(l)
                if safe_lanes:
                    self.current_lane_id = random.choice(safe_lanes)
                    self._update_state_from_pipe(target_pipe_id, target_pos)
                    self.state = "FLYING"
                    self._clear_ramp_transition()
                else:
                    self.switch_timer = 1
            return
        if self.path_index < len(self.path) - 1:
            target_node = self.path[self.path_index + 1]
            target_info = self.env.node_info[target_node]
            if target_info["pipe_id"] != self.current_pipe_id:
                target_pipe_id = int(target_info["pipe_id"])
                ramp_id = self.env.find_directed_ramp_id(
                    self.current_pipe_id, target_pipe_id, target_node=target_node
                )
                if ramp_id is None:
                    target_pipe = self.env.pipelines[target_pipe_id]
                    self.current_lane_id = min(
                        self.current_lane_id, target_pipe.num_lanes - 1
                    )
                    self._update_state_from_pipe(
                        target_pipe_id, float(target_info["dist"])
                    )
                    self.path_index += 1
                    return
                self.state = "SWITCHING"
                self.current_ramp_id = int(ramp_id)
                self.ramp_entry_speed_mps = float(self.speed)
                self.ramp_total_ticks = self._ramp_switch_ticks(
                    ramp_id, sim_dt=1.0, entry_speed_mps=self.ramp_entry_speed_mps
                )
                self.switch_timer = self.ramp_total_ticks
                self.path_index += 1
                return
            dist_to_go = target_info["dist"] - self.pos_1d
            if dist_to_go > 0:
                old_pos = self.pos_1d
                move_dist = min(self.speed * 1.0, dist_to_go)
                self.pos_1d += move_dist
                if (
                    self.phase == "TO_PICKUP"
                    and self.pickup_pt
                    and (self.current_pipe_id == self.pickup_pt.pipe_id)
                ):
                    if old_pos <= self.pickup_pt.pos_1d <= self.pos_1d:
                        self.pickup_pt.remaining -= self.capacity
                        self.gaussian_intensity = 0.0
                        self.phase = "TO_DEST"
                        self.dest_pipe_id = self.final_dest_pipe_id
                        self.dest_region_id = getattr(
                            self.env.pipelines[self.dest_pipe_id], "region_id", 0
                        )
                        if getattr(self, "centralized_route_control", False):
                            self.path = []
                            self.path_index = 0
                            self.needs_route_replan = True
                            return
                        self.replan_path()
                        return
                if self.pos_1d >= target_info["dist"]:
                    self.path_index += 1
                    if target_node == f"DEST_{self.uav_id}_TO_DEST":
                        self.state = "ARRIVED"
                    elif target_node == f"DEST_{self.uav_id}_FALLBACK":
                        self.state = "ARRIVED"
            else:
                self.path_index += 1
                self.step(active_uavs)
        else:
            last_node = (
                self.path[self.path_index]
                if self.path and self.path_index < len(self.path)
                else None
            )
            if last_node in (
                f"DEST_{self.uav_id}_TO_DEST",
                f"DEST_{self.uav_id}_FALLBACK",
            ):
                self.state = "ARRIVED"
            else:
                if getattr(self, "centralized_route_control", False):
                    self.path = []
                    self.path_index = 0
                    self.needs_route_replan = True
                    return
                self.replan_path()

    def plan(self):
        self._execute_routing()

    def replan_path(self):
        self._execute_routing()

    def _execute_routing(self):
        dest_pos = (
            self.pickup_pt.pos_1d
            if self.phase == "TO_PICKUP"
            else self.env.pipelines[self.dest_pipe_id].length
        )
        mode = getattr(self.env, "routing_mode", "DIJKSTRA")
        raw_path = self.env.find_shortest_path(
            self.current_pipe_id,
            self.pos_1d,
            self.dest_pipe_id,
            dest_pos,
            uav_speed=self.speed,
        )
        self._apply_path(raw_path, dest_pos)

    def _apply_path(self, raw_path, dest_pos):
        current_node = f"CUR_{self.uav_id}_{self.phase}"
        current_pipe = self.env.pipelines[self.current_pipe_id]
        self.env.node_info[current_node] = {
            "pipe_id": self.current_pipe_id,
            "dist": self.pos_1d,
            "type": "CURRENT",
            "region_id": getattr(current_pipe, "region_id", 0),
            "region_row": getattr(current_pipe, "region_row", 0),
            "region_col": getattr(current_pipe, "region_col", 0),
        }
        if not raw_path:
            fallback_node = f"DEST_{self.uav_id}_FALLBACK"
            fallback_pipe = self.env.pipelines[self.current_pipe_id]
            self.env.node_info[fallback_node] = {
                "pipe_id": self.current_pipe_id,
                "dist": self.env.pipelines[self.current_pipe_id].length,
                "type": "END",
                "region_id": getattr(fallback_pipe, "region_id", 0),
                "region_row": getattr(fallback_pipe, "region_row", 0),
                "region_col": getattr(fallback_pipe, "region_col", 0),
            }
            self.path = [current_node, fallback_node]
            self.path_index = 0
            return
        if raw_path[-1] != "V_END":
            fallback_node = f"DEST_{self.uav_id}_FALLBACK"
            fallback_pipe = self.env.pipelines[self.current_pipe_id]
            self.env.node_info[fallback_node] = {
                "pipe_id": self.current_pipe_id,
                "dist": self.env.pipelines[self.current_pipe_id].length,
                "type": "END",
                "region_id": getattr(fallback_pipe, "region_id", 0),
                "region_row": getattr(fallback_pipe, "region_row", 0),
                "region_col": getattr(fallback_pipe, "region_col", 0),
            }
            self.path = [current_node, fallback_node]
            self.path_index = 0
            return
        valid_path = [current_node]
        for node in raw_path[1:]:
            if node == "V_END":
                end_name = f"DEST_{self.uav_id}_{self.phase}"
                dest_pipe = self.env.pipelines[self.dest_pipe_id]
                self.env.node_info[end_name] = {
                    "pipe_id": self.dest_pipe_id,
                    "dist": dest_pos,
                    "type": "END",
                    "region_id": getattr(dest_pipe, "region_id", 0),
                    "region_row": getattr(dest_pipe, "region_row", 0),
                    "region_col": getattr(dest_pipe, "region_col", 0),
                }
                valid_path.append(end_name)
            else:
                valid_path.append(node)
        self.path = valid_path
        self.path_index = 0

    def reset(self):
        self.current_pipe_id = self.start_pipe_id
        self.current_lane_id = self.start_lane_id
        self._update_state_from_pipe(self.start_pipe_id, init_pos=0.0)
        self.path = []
        self.path_index = 0
        self.state = "WAITING"
        self.switch_timer = 0
        self._clear_ramp_transition()
        self.action_history = []
        self.avoidance_controlled_this_turn = False
        self.last_state = None
        self.last_action = None
        self.last_acceleration = 0.0
        self.reward_this_turn = 0.0
        self.in_collision = False
        self.was_avoidance_controlled = False
        if self.pickup_pt is not None:
            self.phase = "TO_PICKUP"
            self.dest_pipe_id = self.pickup_pt.pipe_id
            self.dest_region_id = getattr(
                self.env.pipelines[self.dest_pipe_id], "region_id", 0
            )
        else:
            self.phase = "TO_DEST"
            self.dest_pipe_id = self.final_dest_pipe_id
            self.dest_region_id = getattr(
                self.env.pipelines[self.dest_pipe_id], "region_id", 0
            )
        self.gaussian_intensity = float(self.capacity)
        self.current_congestion_percentile = 0.0
        self.current_turn = 0
