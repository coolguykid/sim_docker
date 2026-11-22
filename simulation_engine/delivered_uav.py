"""Thin UAV state holder for the delivered regional control implementation."""
from __future__ import annotations

from dataclasses import replace


class DeliveredUAV:
    def __init__(self, road_state, topology, max_accel=10.0, max_decel=10.0,
                 spawn_turn=0):
        """封装飞机路网状态、拓扑、加减速上限和计划起飞帧。

        输入
            road_state：无人机在路网中的运动状态。
            topology：无人机所在区域的拓扑对象。
            max_accel：最大加速度。
            max_decel：最大制动减速度的正值。
            spawn_turn：无人机计划起飞的帧编号。
        输出
            None。
        """
        self._road_state = road_state
        self.topology = topology
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        self.spawn_turn = int(spawn_turn)

    def commit(self, prepared):
        """提交已预测的运动结果，复制其路网状态。

        输入
            prepared：尚未提交的运动预测。
        输出
            None。
        """
        self._road_state = replace(prepared.state)

    @property
    def uav_id(self):
        """读取飞机唯一编号。

        输入
            无显式参数。
        输出
            无人机 ID。
        """
        return self._road_state.uav_id

    @property
    def state(self):
        """读取无人机生命周期状态。

        输入
            无显式参数。
        输出
            状态字符串。
        """
        return self._road_state.status

    @state.setter
    def state(self, value):
        """更新无人机生命周期状态。

        输入
            value：新的生命周期状态字符串。
        输出
            None。
        """
        self._road_state.status = str(value)

    @property
    def speed(self):
        """读取当前速度。

        输入
            无显式参数。
        输出
            速度，单位米每秒。
        """
        return self._road_state.speed

    @property
    def last_acceleration(self):
        """读取上次提交的加速度。

        输入
            无显式参数。
        输出
            加速度，单位米每二次方秒。
        """
        return self._road_state.last_acceleration

    @property
    def current_lane_id(self):
        """读取当前车道编号。

        输入
            无显式参数。
        输出
            车道 ID。
        """
        return self._road_state.lane_id

    @property
    def pos_1d(self):
        """读取飞机在当前路段上的纵向进度。

        输入
            无显式参数。
        输出
            距离，单位米。
        """
        return self._road_state.progress

    @property
    def current_edge_ref(self):
        """根据当前边或路径位置解析数据库边引用。

        输入
            无显式参数。
        输出
            当前边引用。
        """
        edge_id = self._road_state.edge_id
        if not edge_id and self._road_state.route_index < len(self._road_state.route):
            edge_id = self._road_state.route[self._road_state.route_index]
        if not edge_id and self._road_state.route:
            edge_id = self._road_state.route[-1]
        return self.topology.edge_ref(edge_id)

    def position_3d(self):
        """读取飞机的三维位置。

        输入
            无显式参数。
        输出
            ENU 坐标。
        """
        return self._road_state.position

