"""Stateful regional-DT session backed by the delivered planning/motion core."""
from __future__ import annotations

from collections import deque
from copy import copy
from dataclasses import replace
from types import SimpleNamespace

from uav_sim.airspace.partition_graph import PartitionMaps
from uav_sim.algorithms.regional_control import RegionalControl
from uav_sim.distributed.regional_backend import RegionalRoutingBackend
from uav_sim.entities.partition_motion import RoadState

from .csv_output import RegionalCsvOutput
from .routing_planning_input_adapter import (
    build_region_topology, edge_ref, graph_from_dict, guidance_penalties,
    overlay_local_region, pipe_edge_id, ramp_edge_id, regional_node_id)
from .delivered_uav import DeliveredUAV
from .statistics import EdgeStatisticsTracker, FixedIntervalEdgeStatisticsTracker


class LocalSimulationSession:
    """Own one region; cross-region activity is represented only by events."""

    def __init__(self, config):
        """从运行配置创建区域图、规划器、控制器和输出文件，并写初帧。

        输入
            config：区域会话配置，包含拓扑、任务、导引及输出目录。
        输出
            None。
        """
        self.run_id = str(config["run_id"])
        self.attempt_id = int(config.get("attempt_id", 0))
        self.region_id = int(config["region_id"])
        self.dt = float(config.get("step_seconds", 1.0))
        if self.dt <= 0:
            raise ValueError("step_seconds must be positive")
        (self.graph, self.statistics_env, self.node_external_ids,
         self.layer_by_edge) = build_region_topology(
            self.region_id, int(config["topology_version"]), config["network"])
        supplied_global = config.get("global_routing_graph")
        if supplied_global is None:
            raise ValueError("complete centralized_regional requires global_routing_graph")
        self.global_graph = (supplied_global if hasattr(supplied_global, "edges")
                             else graph_from_dict(supplied_global))
        overlay_local_region(self.global_graph, self.graph, self.region_id, self.node_external_ids)
        penalties = guidance_penalties(
            self.global_graph,
            config.get("global_guidance_transfers", ()),
            config.get("guidance_edge_penalty_seconds", 15.0))
        revision = int(config.get("guidance_revision", 1))
        global_maps = PartitionMaps(self.global_graph).with_edge_penalties(penalties, revision)
        local_penalties = {key: cost for key, cost in penalties.items() if key in self.graph.edges}
        # Routing views share one version. CSV output retains the database snapshot version.
        local_maps = PartitionMaps(
            replace(self.graph, version=self.global_graph.version)
        ).with_edge_penalties(local_penalties, revision)
        self.maps = SimpleNamespace(
            graph=self.global_graph, cost_revision=revision,
            region=local_maps.region, summary=global_maps.summary,
            destination=global_maps.destination,
        )
        self.routing_backend = RegionalRoutingBackend(
            self.maps, mode="centralized_regional", use_ray=False)
        self.control = RegionalControl(self.graph)
        self.time = 0.0
        self.step_index = 0
        self.uavs, self.missions, self.epochs, self.exit_portals = {}, {}, {}, {}
        self.pending_handoffs, self.processed_handoff_ids = {}, set()
        self._prepared_substep = None
        self._certified_frame_plan = None
        self._completed_substeps = {}
        self._prepare_sequence = 0
        self._frame_inbound_edges, self._frame_outbound_edges = [], []
        self.event_substeps = 0
        self.planning_failures = []
        self.statistics_history, self.fixed_statistics_history = deque(), deque()
        env = self.statistics_env
        self.statistics = EdgeStatisticsTracker(
            env, config.get("statistics_window_seconds", 60.0),
            config.get("statistics_interval_seconds", 5.0))
        self.fixed_statistics = FixedIntervalEdgeStatisticsTracker(
            env, config.get("fixed_statistics_interval_seconds", 60.0))
        self.output = RegionalCsvOutput(config["output_dir"])
        for mission in config.get("uavs", ()):
            try:
                self._insert_initial(dict(mission))
            except ValueError as exc:
                self.planning_failures.append({
                    "uav_id": mission.get("uav_id"), "reason": str(exc)})
        # WAITING snapshot aircraft have not yet received an airborne safety
        # certificate. Keep rejected admissions cold and retry them on later
        # frames, exactly like newly handed-off aircraft.
        self.pending_admission_ids = {
            uid for uid, uav in self.uavs.items() if uav.state == "WAITING"
        }
        self.output.set_frame_time(0.0)
        self.output.write_frame(self._frame())

    def _database_edge_id(self, ref):
        """根据数据库边类型和编号生成区域图边 ID。

        输入
            ref：数据库边引用。
        输出
            内部图边 ID。
        """
        return (pipe_edge_id(self.region_id, ref["edge_id"])
                if str(ref["edge_type"]) == "pipe"
                else ramp_edge_id(self.region_id, ref["edge_id"]))

    def _lane_position(self, edge, lane, distance):
        """按航段进度和车道偏移插值三维位置。

        输入
            edge：内部图边。
            lane：车道编号。
            distance：沿当前边的纵向距离。
        输出
            ENU 坐标。
        """
        lane = max(0, min(int(lane), edge.lane_count - 1))
        fraction = 0.0 if edge.length_m <= 0 else max(
            0.0, min(1.0, float(distance) / edge.length_m))
        center = tuple(a + (b-a)*fraction for a, b in zip(edge.points[0], edge.points[-1]))
        if not edge.offsets_start or not edge.offsets_end:
            return center
        offset = tuple(a + (b-a)*fraction for a, b in zip(
            edge.offsets_start[lane], edge.offsets_end[lane]))
        return tuple(a+b for a, b in zip(center, offset))

    def _initial_edge(self, mission):
        """为初始任务解析指定航段或起点的出边。

        输入
            mission：无人机起点、目的地及初始状态描述。
        输出
            初始图边 ID。
        """
        if mission.get("current_edge"):
            edge_id = self._database_edge_id(mission["current_edge"])
            if edge_id not in self.graph.edges:
                raise ValueError("snapshot edge {} is closed or absent".format(edge_id))
            return edge_id
        node_id = regional_node_id(mission["node_id"], self.region_id)
        choices = [eid for _, _, eid in self.maps.region(self.region_id).adjacency.get(node_id, ())]
        if not choices:
            raise ValueError("snapshot node has no open outgoing edge")
        return choices[0]

    def _insert_initial(self, mission):
        """建立初始无人机、登记任务及移交代数，并进行路径规划。

        输入
            mission：无人机起点、目的地及初始状态描述。
        输出
            None。
        """
        edge_id = self._initial_edge(mission)
        edge = self.graph.edges[edge_id]
        distance = max(0.0, min(float(mission.get("distance_on_edge") or 0), edge.length_m))
        lane = max(0, min(int(mission.get("lane_id") or 0), edge.lane_count - 1))
        state = RoadState(
            int(mission["uav_id"]), edge.source, self.region_id, lane,
            self._lane_position(edge, lane, distance), None,
            speed=float(mission.get("speed") or 0), status="WAITING",
            route=(edge_id,), progress=distance, edge_id=edge_id,
            visited_regions=(self.region_id,),
            last_acceleration=float(mission.get("acceleration") or 0))
        uav = DeliveredUAV(
            state, self.graph, mission.get("max_accel", 10.0),
            mission.get("max_decel", 10.0), mission.get("spawn_turn", 0))
        self.uavs[uav.uav_id] = uav
        self.missions[uav.uav_id] = mission
        self.epochs[uav.uav_id] = int(mission.get("handoff_epoch") or 0)
        try:
            self._plan(uav)
        except ValueError:
            uav.state = "BLOCKED"
            raise

    def queue_handoffs(self, events):
        """校验目标区域并按事件 ID 去重，将移交事件排入等待队列。

        输入
            events：待处理的事件序列。
        输出
            None。
        """
        for raw in events:
            event = dict(raw)
            event_id = str(event["event_id"])
            if event_id in self.pending_handoffs or event_id in self.processed_handoff_ids:
                continue
            if int(event["to_region"]) != self.region_id:
                raise ValueError("handoff target does not match local region")
            self.pending_handoffs[event_id] = event

    def _accept_due_handoffs(self):
        """接收已到时的移交，创建无人机并规划本区前缀。

        输入
            无显式参数。
        输出
            入境边引用列表和新增无人机 ID 列表。
        """
        accepted, new_ids = [], []
        for event_id, event in list(self.pending_handoffs.items()):
            if float(event["crossing_time"]) > self.time + 1e-9:
                continue
            payload = event["payload"]
            node_id = regional_node_id(event["boundary_node_id"], self.region_id)
            choices = [eid for _, _, eid in self.maps.region(self.region_id).adjacency.get(node_id, ())
                       if self.graph.edges[eid].kind != "BOUNDARY"]
            layer = int(payload.get("target_layer", 0))
            matching = [eid for eid in choices if self.layer_by_edge.get(eid) == layer]
            choices = matching or choices
            if not choices:
                raise ValueError("boundary node has no open target-region edge")
            edge_id, mission = choices[0], dict(payload["mission"])
            edge = self.graph.edges[edge_id]
            state = RoadState(
                int(event["uav_id"]), edge.source, self.region_id, 0,
                self._lane_position(edge, 0, 0), None,
                speed=float(event.get("speed") or 0), status="WAITING",
                route=(edge_id,), edge_id=edge_id,
                visited_regions=tuple(payload.get("visited_regions", ())) + (self.region_id,),
                last_acceleration=float(event.get("acceleration") or 0))
            uav = DeliveredUAV(state, self.graph, spawn_turn=self.step_index)
            self.uavs[uav.uav_id] = uav
            self.missions[uav.uav_id] = mission
            self.epochs[uav.uav_id] = int(event["handoff_epoch"])
            try:
                self._plan(uav)
            except ValueError as exc:
                uav.state = "BLOCKED"
                self.planning_failures.append({"uav_id": uav.uav_id, "reason": str(exc)})
            accepted.append(edge_ref(self.graph.edges[edge_id]))
            new_ids.append(uav.uav_id)
            self.processed_handoff_ids.add(event_id)
            del self.pending_handoffs[event_id]
        return accepted, new_ids

    def _routing_context(self):
        """封装当前区域和导引设置供路由后端调用。

        输入
            无显式参数。
        输出
            区域寻路上下文。
        """
        return {
            "step_index": int(self.step_index),
            "sim_time_seconds": float(self.time),
            "sim_dt_seconds": float(self.dt),
            "topology_version": int(self.global_graph.version),
        }

    def _route(self, uav, target):
        """请求从飞机当前状态到目标的路径。

        输入
            uav：待规划或推进的无人机对象。
            target：目的地或局部目标节点。
        输出
            路由规划结果。

        原有说明
            Call the delivered official mode once for the final destination.
            
            The delivered planner accepts a node as its start.  A snapshot can be
            part-way along an edge, so that physical edge is kept as an immutable
            prefix and the official planner starts at its target node.
        """
        state = uav._road_state
        current = self.graph.edges[state.edge_id or state.route[state.route_index]]
        plan = self.routing_backend.plan_many({
            int(uav.uav_id): {
                "start_node_id": current.target,
                "region_id": self.region_id,
                "goal_node_id": str(target),
            }
        }, self._routing_context()).get(int(uav.uav_id))
        if plan is None or plan.status != "OK":
            reason = "missing route result" if plan is None else plan.reason
            raise ValueError("centralized_regional: {}".format(reason))
        planned = tuple(plan.edge_ids)
        if planned and self.graph.edges[planned[0]].source != current.target:
            raise ValueError("centralized_regional returned a noncontiguous prefix")
        return plan, (current.edge_id,) + planned

    def _set_goal(self, state, target, route):
        """设置道路状态的局部目标和可执行路径。

        输入
            state：当前或已预测的路网状态。
            target：目的地或局部目标节点。
            route：计划执行的有序内部边序列。
        输出
            None。
        """
        edge = self.graph.edges[route[-1]]
        state.route, state.route_index = tuple(route), 0
        state.goal = SimpleNamespace(
            node_id=target, lane_id=state.lane_id,
            position=self._lane_position(edge, state.lane_id, edge.length_m))

    def _plan(self, uav):
        """生成可执行的本区路径，并登记必要的出境门户。

        输入
            uav：待规划或推进的无人机对象。
        输出
            None。
        """
        state = uav._road_state
        mission = self.missions[uav.uav_id]
        try:
            destination_region = int(mission["destination_region_id"])
            destination_node = int(mission["destination_node_id"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("complete mode requires destination_region_id and destination_node_id") from exc
        target = regional_node_id(destination_node, destination_region)
        _plan, route = self._route(uav, target)
        last = self.global_graph.edges[route[-1]]
        if last.kind == "BOUNDARY":
            executable = route[:-1]
            if not executable:
                raise ValueError("boundary-only prefix requires strict zero-time handoff support")
            transition = self.global_graph.metadata["boundary_transitions"][last.edge_id]
            portal = SimpleNamespace(
                boundary_edge_id=last.edge_id,
                boundary_node_id=int(transition["shared_node_id"]),
                from_region=int(transition["source_region"]),
                to_region=int(transition["target_region"]),
            )
            self._set_goal(state, last.source, executable)
            self.exit_portals[uav.uav_id] = (portal, int(transition["target_layer"]))
        else:
            self._set_goal(state, target, route)
            self.exit_portals.pop(uav.uav_id, None)

    def _active_uavs(self):
        """筛选已到起飞时间且可参与控制的无人机。

        输入
            无显式参数。
        输出
            以 ID 为键的无人机字典。
        """
        return {
            uid: uav for uid, uav in self.uavs.items()
            if uav.state not in ("BLOCKED", "NO_ROUTE", "ARRIVED")
            and self.step_index >= uav.spawn_turn
        }

    def _emit_arrived_portals(self, frame_end):
        """从本区状态移除到达出境口的飞机并生成精确时刻的移交。

        输入
            frame_end：当前输出帧的结束时刻。
        输出
            移交载荷列表和出境边引用列表。

        原有说明
            Remove locally completed prefixes and create exact-time handoffs.
        """
        outbound, outbound_edges = [], []
        cached = self._certified_frame_plan
        for uid, uav in list(self.uavs.items()):
            if uav.state != "ARRIVED" or uid not in self.exit_portals:
                continue
            outbound.append(self._portal_event(uid, uav._road_state, frame_end))
            self.exit_portals.pop(uid)
            self.epochs[uid] += 1
            self.missions.pop(uid)
            outbound_edges.append(uav.current_edge_ref)
            self.pending_admission_ids.discard(uid)
            if cached is not None:
                cached["proposals"].pop(uid, None)
                cached["predictions"].pop(uid, None)
            del self.uavs[uid]
        return outbound, outbound_edges

    def prepare_batch_frame(self, frame_index, start, end):
        """保存可回退的本区帧首状态并预测本帧出境。

        输入
            frame_index：从零开始的输出帧编号。
            start：起点坐标或帧开始时刻，依所在函数而定。
            end：终点坐标或帧结束时刻，依所在函数而定。
        输出
            包含候选移交的字典。

        原有说明
            Checkpoint mutable state before preparing a speculative frame.
        """
        if self._prepared_substep is not None or self._certified_frame_plan is not None:
            raise ValueError("batch must start at a frame boundary")
        checkpoint = {}
        for name in (
            "time", "uavs", "missions", "epochs", "exit_portals", "pending_handoffs",
            "processed_handoff_ids", "pending_admission_ids", "_completed_substeps",
            "_prepare_sequence", "_frame_inbound_edges", "_frame_outbound_edges",
            "event_substeps", "planning_failures",
        ):
            checkpoint[name] = copy(getattr(self, name))
        checkpoint["uavs"] = {uid: copy(uav) for uid, uav in self.uavs.items()}
        for uav in checkpoint["uavs"].values():
            uav._road_state = copy(uav._road_state)
        self._batch_checkpoint = checkpoint
        self.prepare_substep(frame_index, start, end - start)
        proposals = self._prepared_substep["proposals"]
        events = [
            self._portal_event(uid, proposals[uid].state, end)
            for uid in self.uavs
            if uid in self.exit_portals and uid in proposals
            and proposals[uid].state.status == "ARRIVED"
        ]
        return {"events": events}

    def apply_batch_frame(self, frame_index, end, events, expected):
        """按本区入境和出境时间重放一帧，校验预测移交。

        输入
            frame_index：从零开始的输出帧编号。
            end：终点坐标或帧结束时刻，依所在函数而定。
            events：待处理的事件序列。
            expected：帧首已预报、供核对的出境事件列表。
        输出
            含 matches 标志的结果字典。

        原有说明
            Replay local crossings and arrivals without publishing any frame.
        """
        self.queue_handoffs(events)
        arrivals = iter(sorted(float(event["crossing_time"]) for event in events))
        next_arrival = next(arrivals, end)
        outbound = []
        while self.time < end - 1e-8:
            while next_arrival < end and next_arrival <= self.time + 1e-9:
                next_arrival = next(arrivals, end)
            prepared = self.prepare_substep(frame_index, self.time, end - self.time)
            crossing = prepared["next_event_time"]
            deadline = min(end, next_arrival, end if crossing is None else crossing)
            if abs(deadline - end) <= 1e-8:
                deadline = end
            if abs(deadline - self.time) <= 1e-8 and next_arrival > self.time + 1e-9:
                # Keep a distinct inbound deadline when it is outside admission tolerance.
                if deadline != next_arrival or crossing is not None and crossing <= self.time + 1e-8:
                    deadline = self.time
            previous = self.time
            result = self.commit_substep(prepared["prepare_token"], deadline)
            outbound.extend(result["outbound_handoffs"])
            if any(event not in expected for event in result["outbound_handoffs"]):
                return {"matches": False}
            if self.time <= previous and not result["outbound_handoffs"]:
                return {"matches": False}
        return {"matches": sorted(outbound, key=lambda e: e["event_id"]) ==
                sorted(expected, key=lambda e: e["event_id"])}

    def discard_batch_frame(self, rollback):
        """按标志回滚区域状态或丢弃检查点。

        输入
            rollback：是否将检查点恢复到帧首。
        输出
            空字典。
        """
        if rollback:
            self.__dict__.update(self._batch_checkpoint)
            self._prepared_substep = None
            self._certified_frame_plan = None
        del self._batch_checkpoint
        return {}

    def _portal_event(self, uid, state, frame_end):
        """根据出境飞机的预测状态构造跨区移交载荷。

        输入
            uid：无人机唯一编号。
            state：当前或已预测的路网状态。
            frame_end：当前输出帧的结束时刻。
        输出
            移交事件字典。
        """
        portal, target_layer = self.exit_portals[uid]
        epoch = self.epochs[uid] + 1
        crossing = float(state.arrival_time if state.arrival_time is not None else self.time)
        return {
            "event_id": "{}:{}:{}:{}".format(self.run_id, self.attempt_id, uid, epoch),
            "run_id": self.run_id, "attempt_id": self.attempt_id,
            "uav_id": uid, "handoff_epoch": epoch,
            "from_region": self.region_id, "to_region": portal.to_region,
            "boundary_node_id": portal.boundary_node_id,
            "crossing_time": crossing,
            "remaining_step_time": max(0.0, float(frame_end) - crossing),
            "speed": state.speed, "acceleration": state.last_acceleration,
            "payload": {"target_layer": target_layer, "mission": self.missions[uid],
                        "visited_regions": list(state.visited_regions)},
        }

    def prepare_substep(self, frame_index, sim_time, max_duration):
        """处理到时入境并预测本区最早出境，保存待提交方案。

        输入
            frame_index：从零开始的输出帧编号。
            sim_time：当前仿真时刻，单位秒。
            max_duration：该子步允许推进的最长仿真秒数。
        输出
            包含准备令牌和最早事件时刻的字典。

        原有说明
            Prepare, but do not commit, a safety-certified event substep.
            
            The coordinator calls every region with the same interval.  Each
            region reports its earliest local portal crossing.  This is phase one
            of the distributed event barrier.
        """
        frame_index = int(frame_index)
        sim_time = float(sim_time)
        max_duration = float(max_duration)
        if frame_index != self.step_index or abs(sim_time-self.time) > 1e-8:
            raise ValueError("regional event substep is out of sequence")
        if max_duration <= 0:
            raise ValueError("event substep duration must be positive")
        frame_end = (frame_index + 1) * self.dt
        if sim_time + max_duration > frame_end + 1e-8:
            raise ValueError("event substep exceeds the current output frame")
        if self._prepared_substep is not None:
            prepared = self._prepared_substep
            if (prepared["frame_index"] == frame_index and
                    abs(prepared["start_time"]-sim_time) <= 1e-8 and
                    abs(prepared["duration"]-max_duration) <= 1e-8):
                return dict(prepared["public"])
            raise ValueError("a different event substep is already prepared")

        inbound_edges, inbound_ids = self._accept_due_handoffs()
        self._frame_inbound_edges.extend(inbound_edges)
        self.pending_admission_ids.update(inbound_ids)

        cached = self._certified_frame_plan
        if cached is not None and cached["frame_index"] == frame_index:
            if inbound_ids:
                inbound_active = {
                    uid: self.uavs[uid]
                    for uid in inbound_ids
                    if uid in self.uavs
                    and self.uavs[uid].state not in ("BLOCKED", "NO_ROUTE", "ARRIVED")
                    and self.step_index >= self.uavs[uid].spawn_turn
                }
                if inbound_active:
                    inc_result = self.control.compute(
                        inbound_active, self.time, max_duration,
                        new_ids=set(inbound_active),
                        locked_predictions=cached["predictions"])
                    cached["proposals"].update(inc_result["proposals"])
                    cached["predictions"].update(inc_result.get("predictions", {}))
                    cached["deferred_ids"].update(inc_result["deferred_ids"])
                    cached["due_admissions"].update(inbound_active)
            proposals = cached["proposals"]
            deferred_ids = cached["deferred_ids"]
            due_admissions = cached["due_admissions"]
            future_admissions = cached["future_admissions"]
        else:
            active = self._active_uavs()
            due_admissions = self.pending_admission_ids.intersection(active)
            future_admissions = self.pending_admission_ids.difference(active)
            control_result = self.control.compute(
                active, self.time, max_duration, new_ids=due_admissions)
            proposals = dict(control_result["proposals"])
            predictions = dict(control_result.get("predictions", {}))
            deferred_ids = set(control_result["deferred_ids"])
            self._certified_frame_plan = {
                "frame_index": frame_index,
                "base_time": self.time,
                "proposals": proposals,
                "predictions": predictions,
                "deferred_ids": deferred_ids,
                "due_admissions": set(due_admissions),
                "future_admissions": set(future_admissions),
            }

        local_events = []
        for uid, proposal in proposals.items():
            if uid not in self.exit_portals or proposal.state.status != "ARRIVED":
                continue
            crossing = proposal.state.arrival_time
            if crossing is not None and crossing <= sim_time + max_duration + 1e-8:
                local_events.append((float(crossing), int(uid)))
        next_event_time = min((item[0] for item in local_events), default=None)
        event_uav_ids = [uid for when, uid in local_events
                         if next_event_time is not None and
                         abs(when-next_event_time) <= 1e-8]
        self._prepare_sequence += 1
        token = "{}:{}:{}".format(frame_index, self._prepare_sequence, sim_time)
        public = {
            "region_id": self.region_id,
            "frame_index": frame_index,
            "sim_time": sim_time,
            "max_duration": max_duration,
            "prepare_token": token,
            "next_event_time": next_event_time,
            "event_uav_ids": event_uav_ids,
        }
        self._prepared_substep = {
            "token": token, "public": public,
            "frame_index": frame_index, "start_time": sim_time,
            "duration": max_duration, "frame_end": frame_end,
            "proposals": proposals,
            "deferred_ids": deferred_ids,
            "due_admissions": set(due_admissions),
            "future_admissions": set(future_admissions),
            "event_uav_ids": set(event_uav_ids),
        }
        return dict(public)

    def commit_substep(self, prepare_token, advance_to):
        """根据准备令牌推进至统一截止时刻并生成出境移交。

        输入
            prepare_token：区域准备阶段生成的提交令牌。
            advance_to：本事件子步统一推进至的仿真时刻。
        输出
            包含时钟和出境事件的字典。

        原有说明
            Commit a prepared interval only up to the global event deadline.
        """
        token = str(prepare_token)
        if token in self._completed_substeps:
            return dict(self._completed_substeps[token])
        prepared = self._prepared_substep
        if prepared is None or token != prepared["token"]:
            raise ValueError("unknown or stale event prepare token")
        advance_to = float(advance_to)
        start = prepared["start_time"]
        horizon_end = start + prepared["duration"]
        if advance_to < start - 1e-8 or advance_to > horizon_end + 1e-8:
            raise ValueError("event commit deadline is outside the prepared interval")
        duration = max(0.0, advance_to-start)
        proposals = prepared["proposals"]
        next_ev = prepared["public"].get("next_event_time")
        due_event_ids = (
            prepared["event_uav_ids"]
            if next_ev is not None and abs(float(next_ev) - advance_to) <= 1e-8
            else set()
        )
        full_horizon = abs(advance_to-horizon_end) <= 1e-8

        if duration <= 1e-10 and not full_horizon:
            committed_ids = due_event_ids
            for uid in committed_ids:
                if uid in self.uavs and uid in proposals:
                    self.uavs[uid].commit(proposals[uid])
            self.pending_admission_ids = (
                prepared["future_admissions"] |
                prepared["deferred_ids"] |
                (prepared["due_admissions"] - committed_ids))
        elif full_horizon:
            for uid, proposal in list(proposals.items()):
                if uid in self.uavs:
                    self.uavs[uid].commit(proposal)
            self.pending_admission_ids = (
                prepared["future_admissions"] | prepared["deferred_ids"])
            self._certified_frame_plan = None
        else:
            # Intermediate event substep: commit only due portal arrivals now;
            # remaining aircraft continue lazily along their certified frame plan
            # and are materialized at frame_end.
            for uid in due_event_ids:
                if uid in self.uavs and uid in proposals:
                    self.uavs[uid].commit(proposals[uid])
            self.pending_admission_ids = (
                prepared["future_admissions"] | prepared["deferred_ids"])

        self.time = advance_to
        outbound, outbound_edges = self._emit_arrived_portals(prepared["frame_end"])
        self._frame_outbound_edges.extend(outbound_edges)
        self.event_substeps += 1
        response = {
            "region_id": self.region_id,
            "frame_index": self.step_index,
            "sim_time": self.time,
            "prepare_token": token,
            "outbound_handoffs": outbound,
            "uav_count": len(self.uavs),
        }
        self._completed_substeps[token] = dict(response)
        if len(self._completed_substeps) > 8:
            del self._completed_substeps[next(iter(self._completed_substeps))]
        self._prepared_substep = None
        return response

    def finish_frame(self, frame_index, frame_time):
        """核对帧边界，写出无人机状态和统计。

        输入
            frame_index：从零开始的输出帧编号。
            frame_time：帧结束时刻，单位秒。
        输出
            区域帧摘要。

        原有说明
            Publish one CSV/statistics frame after all same-frame events settle.
        """
        frame_index = int(frame_index)
        frame_time = float(frame_time)
        if self._prepared_substep is not None:
            raise ValueError("cannot finish a frame with a prepared substep")
        if frame_index != self.step_index or abs(frame_time-self.time) > 1e-8:
            raise ValueError("regional frame finalization is out of sequence")
        expected = (frame_index + 1) * self.dt
        if abs(frame_time-expected) > 1e-8:
            raise ValueError("regional frame has not reached its output boundary")
        inbound_edges, inbound_ids = self._accept_due_handoffs()
        self._frame_inbound_edges.extend(inbound_edges)
        self.pending_admission_ids.update(inbound_ids)
        self._certified_frame_plan = None
        if hasattr(self, '_batch_checkpoint'):
            self.discard_batch_frame(rollback=False)
        self.step_index += 1
        statistics = self.statistics.observe(
            self.time, self.uavs.values(),
            self._frame_inbound_edges, self._frame_outbound_edges)
        fixed = self.fixed_statistics.observe(self.statistics.last_observation)
        frame = self._frame()
        self.output.set_frame_time(self.time)
        self.output.write_frame(frame)
        self._record_statistics(statistics, fixed)
        self._frame_inbound_edges.clear()
        self._frame_outbound_edges.clear()
        return {
            "region_id": self.region_id, "attempt_id": self.attempt_id,
            "step_index": self.step_index, "sim_time": self.time,
            "uav_count": len(frame),
            "planning_failure_count": len(self.planning_failures),
            "statistics_emitted": statistics is not None,
            "fixed_statistics_emitted": fixed is not None,
        }

    def step(self, step_index, sim_time):
        """以单步接口推进本区运动并写入输出。

        输入
            step_index：从零开始的仿真步编号。
            sim_time：当前仿真时刻，单位秒。
        输出
            本次推进的区域结果。
        """
        if int(step_index) != self.step_index or abs(float(sim_time)-self.time) > 1e-9:
            raise ValueError("regional step is out of sequence")
        inbound_edges, inbound_ids = self._accept_due_handoffs()
        self.pending_admission_ids.update(inbound_ids)
        active = self._active_uavs()
        due_admissions = self.pending_admission_ids.intersection(active)
        future_admissions = self.pending_admission_ids.difference(active)
        control_result = self.control.compute(
            active, self.time, self.dt,
            new_ids=due_admissions)
        proposals = control_result["proposals"]
        self.pending_admission_ids = (
            future_admissions | set(control_result["deferred_ids"]))
        for uid, prepared in proposals.items():
            self.uavs[uid].commit(prepared)

        outbound, outbound_edges = self._emit_arrived_portals(self.time+self.dt)

        self.time += self.dt
        if hasattr(self, '_batch_checkpoint'):
            self.discard_batch_frame(rollback=False)
        self.step_index += 1
        statistics = self.statistics.observe(
            self.time, self.uavs.values(), inbound_edges, outbound_edges)
        fixed = self.fixed_statistics.observe(self.statistics.last_observation)
        frame = self._frame()
        self.output.set_frame_time(self.time)
        self.output.write_frame(frame)
        self._record_statistics(statistics, fixed)
        return {"region_id": self.region_id, "attempt_id": self.attempt_id,
                "step_index": self.step_index, "sim_time": self.time,
                "outbound_handoffs": outbound, "uav_count": len(frame),
                "planning_failure_count": len(self.planning_failures),
                "statistics_emitted": statistics is not None,
                "fixed_statistics_emitted": fixed is not None}

    def _record_statistics(self, statistics, fixed):
        """完整写入 CSV，仅缓存最近 60 秒内产生的统计批次。

        输入
            statistics：当前滑动统计批次，未到输出时刻时为 None。
            fixed：当前固定周期统计批次，未到周期末时为 None。
        输出
            None。写入 CSV 并清理超过保留窗口的批次。
        """
        if statistics is not None:
            self.output.write_statistics(statistics)
            self.statistics_history.append(statistics)
        if fixed is not None:
            self.output.write_fixed_statistics(fixed)
            self.fixed_statistics_history.append(fixed)
        cutoff = self.time - 60.0
        for history in (self.statistics_history, self.fixed_statistics_history):
            while history and history[0]["time"] <= cutoff:
                history.popleft()


    def _frame(self):
        """将当前无人机状态投影为 CSV 写入所需的帧记录。

        输入
            无显式参数。
        输出
            无人机状态字典列表。
        """
        frame = []
        for uid in sorted(self.uavs):
            uav, state = self.uavs[uid], self.uavs[uid]._road_state
            try:
                ref = uav.current_edge_ref
                edge = self.graph.edges[
                    pipe_edge_id(self.region_id, ref.edge_id)
                    if ref.edge_type == "pipe"
                    else ramp_edge_id(self.region_id, ref.edge_id)]
            except (KeyError, IndexError):
                continue
            frame.append({"time": self.time, "uav_id": uid,
                          "region_id": self.region_id,
                          "topology_version": self.graph.version,
                          "state": state.status, "edge_type": ref.edge_type,
                          "edge_id": ref.edge_id,
                          "node_id": self.node_external_ids.get(state.node_id, -1),
                          "lane_id": state.lane_id,
                          "distance_on_edge": state.progress,
                          "edge_length": edge.length_m, "speed": state.speed,
                          "acceleration": state.last_acceleration,
                          "position": list(state.position)})
        return frame

    def summary(self):
        """汇总会话时钟、无人机数量及规划指标。

        输入
            无显式参数。
        输出
            区域会话摘要字典。
        """
        states = {}
        for uav in self.uavs.values():
            states[uav.state] = states.get(uav.state, 0)+1
        return {"run_id": self.run_id, "attempt_id": self.attempt_id,
                "region_id": self.region_id, "step_index": self.step_index,
                "sim_time": self.time, "uav_states": states,
                "pending_handoffs": len(self.pending_handoffs),
                "pending_admissions": len(self.pending_admission_ids),
                "event_substeps": self.event_substeps,
                "event_prepare_pending": self._prepared_substep is not None,
                "planning_failures": list(self.planning_failures),
            "engine": "uav_sim_delivery_partition_core",
            "routing_mode": "centralized_regional",
            "routing_metrics": self.routing_backend.metrics()}

    def result(self):
        """附加最近 60 秒的统计批次和完整 CSV 路径以生成会话结果。

        输入
            无显式参数。
        输出
            结果字典。
        """
        return {**self.summary(), "statistics": list(self.statistics_history),
                "fixed_statistics": list(self.fixed_statistics_history),
                "frame_csv": str(self.output.frame_path),
                "statistics_csv": str(self.output.statistics_path),
                "fixed_statistics_csv": str(self.output.fixed_statistics_path)}

    def close(self):
        """关闭区域路由后端和 CSV 输出流。

        输入
            无显式参数。
        输出
            None。
        """
        self.routing_backend.close()
        self.output.close()
