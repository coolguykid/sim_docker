import numpy as np
from types import SimpleNamespace
from uav_sim.algorithms.avoidance_actions import desired_speed_for_action
from uav_sim.algorithms.motion_constraints import (
    max_accel_from_observation,
    max_decel_from_observation,
)
from uav_sim.distributed.route_planning_server import RoutePlanningCore
from uav_sim.algorithms.safety_shield import apply_safety_shield, build_pipe_lane_index
from uav_sim.distributed.server_control_messages import (
    STATE_FLYING,
    STATE_SWITCHING,
    make_command_array,
)

try:
    import ray
except ModuleNotFoundError:
    ray = None


class AuthoritativeControlCore:
    @classmethod
    def for_partition(cls, graph, max_accel=10., max_decel=10., **options):
        """Use the existing authoritative core with pure partition proposals.

        The normal constructor and command protocol remain unchanged.  This
        factory does not build or import a second airspace/routing model.
        """
        from uav_sim.algorithms.regional_control import RegionalControl
        instance = cls.__new__(cls)
        instance.routing_mode = 'DIJKSTRA'
        instance.avoidance_policy = 'rule'
        instance._partition_control = RegionalControl(graph, max_accel, max_decel, **options)
        return instance

    def compute_partition_proposals(self, uavs, now, dt, next_region_callback=None, new_ids=()):
        """Prepare a certified fleet step without committing physical state."""
        return self._partition_control.compute(uavs, now, dt, next_region_callback, new_ids)

    @property
    def prefix_reservations(self):
        """Cached exact prefix certificates owned by this authoritative core."""
        if not hasattr(self, '_prefix_reservations'):
            from uav_sim.algorithms.prefix_reservations import PrefixReservations
            self._prefix_reservations = PrefixReservations()
        return self._prefix_reservations

    def __init__(
        self,
        env_snapshot,
        routing_mode="DIJKSTRA",
        avoidance_policy="rule",
        safety_substep=0.2,
    ):
        if routing_mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        self.route_core = RoutePlanningCore(env_snapshot, routing_mode=routing_mode)
        self.env = self.route_core.env
        self.routing_mode = routing_mode
        self.avoidance_policy = avoidance_policy
        self.safety_substep = safety_substep
        self._last_route_updates = []
        self._last_ramp_state_version = None

    def set_routing_mode(self, mode):
        if mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        self.routing_mode = mode
        self.route_core.set_routing_mode(mode)

    def set_avoidance_policy(self, policy):
        if policy != "rule":
            raise ValueError("Only rule avoidance is available")
        self.avoidance_policy = "rule"

    def compute_commands(self, turn, observations, shared_context=None):
        shared_context = shared_context or {}
        self._refresh_route_context(shared_context)
        route_updates = self._plan_needed_routes(turn, observations, shared_context)
        candidate_commands = self._build_candidate_commands(observations)
        safe_commands, conflicts = apply_safety_shield(
            candidate_commands,
            observations,
            self.env,
            delta_t=self.safety_substep,
            horizon=1.0,
        )
        return {
            "commands": safe_commands,
            "route_updates": route_updates,
            "conflicts": conflicts,
            "metrics": {
                "safety_modified_count": int(
                    np.count_nonzero(safe_commands["safety_modified"])
                ),
                "hold_count": int(np.count_nonzero(safe_commands["hold_position"])),
                "hard_brake_count": int(np.count_nonzero(safe_commands["hard_brake"])),
                "route_update_count": len(route_updates),
            },
        }

    def prepare_initial_routes(self, turn, observations, shared_context=None):
        shared_context = shared_context or {}
        self._refresh_route_context(shared_context)
        return self._plan_needed_routes(turn, observations, shared_context)

    def _refresh_route_context(self, shared_context):
        self.route_core._refresh_context(shared_context)

    def _plan_needed_routes(self, turn, observations, shared_context):
        from uav_sim.distributed.messages import RouteRequest

        ramp_state_version = int(shared_context.get("ramp_state_version", 0))
        force_flying_replan = (
            self._last_ramp_state_version is not None
            and ramp_state_version != self._last_ramp_state_version
        )
        requests = []
        for obs in observations:
            if int(obs["state_code"]) == 3:
                continue
            needs_route = bool(obs["needs_route"])
            if not needs_route and (
                not (force_flying_replan and int(obs["state_code"]) == STATE_FLYING)
            ):
                continue
            dest_pipe_id = int(obs["dest_pipe_id"])
            dest_pos = float(self.env.pipelines[dest_pipe_id].length)
            if int(obs["phase_code"]) == 0 and int(obs["pickup_pipe_id"]) >= 0:
                dest_pipe_id = int(obs["pickup_pipe_id"])
                dest_pos = float(obs["pickup_pos_1d"])
            requests.append(
                RouteRequest(
                    uav_id=int(obs["uav_id"]),
                    current_pipe_id=int(obs["pipe_id"]),
                    pos_1d=float(obs["pos_1d"]),
                    dest_pipe_id=dest_pipe_id,
                    dest_pos=dest_pos,
                    phase="TO_PICKUP" if int(obs["phase_code"]) == 0 else "TO_DEST",
                    spawn_turn=int(obs["spawn_turn"]),
                    current_turn=int(turn),
                    speed=float(obs["speed"]),
                    max_accel=max_accel_from_observation(obs),
                    max_decel=max_decel_from_observation(obs),
                    mandatory_waypoints=None,
                    final_dest_pipe_id=int(obs["final_dest_pipe_id"]),
                )
            )
        route_updates = (
            self.route_core.plan_routes(requests, shared_context) if requests else []
        )
        self._last_ramp_state_version = ramp_state_version
        self._last_route_updates = route_updates
        return route_updates

    def _pipe_uavs_map(self, observations):
        pipe_map = {}
        for obs in observations:
            if int(obs["state_code"]) != STATE_FLYING:
                continue
            pid = int(obs["pipe_id"])
            pipe_map.setdefault(pid, []).append(
                {
                    "uav_id": int(obs["uav_id"]),
                    "pipe_id": pid,
                    "lane_id": int(obs["lane_id"]),
                    "pos_1d": float(obs["pos_1d"]),
                    "speed": float(obs["speed"]),
                    "state": "FLYING",
                }
            )
        return pipe_map

    def _uav_proxy(self, obs):
        return SimpleNamespace(
            uav_id=int(obs["uav_id"]),
            current_pipe_id=int(obs["pipe_id"]),
            current_lane_id=int(obs["lane_id"]),
            pos_1d=float(obs["pos_1d"]),
            speed=float(obs["speed"]),
            max_accel=max_accel_from_observation(obs),
            max_decel=max_decel_from_observation(obs),
            last_acceleration=float(obs["last_acceleration"])
            if "last_acceleration" in obs.dtype.names
            else 0.0,
            path=[],
            path_index=0,
            dest_pipe_id=int(obs["dest_pipe_id"]),
        )

    def _build_candidate_commands(self, observations):
        active_index = build_pipe_lane_index(observations)
        commands = []
        for obs in observations:
            uid = int(obs["uav_id"])
            state = int(obs["state_code"])
            target_speed = float(obs["speed"])
            target_lane = int(obs["lane_id"])
            action_id = 0
            hold = state not in (STATE_FLYING, STATE_SWITCHING)
            if state == STATE_FLYING:
                target_speed, target_lane, action_id = self._rule_candidate(
                    obs, observations, active_index
                )
            commands.append(
                {
                    "uav_id": uid,
                    "target_speed": target_speed,
                    "target_lane_id": target_lane,
                    "action_id": action_id,
                    "hold_position": hold,
                    "enter_ramp_id": int(obs["current_ramp_id"]),
                    "safety_modified": False,
                    "route_update": False,
                    "hard_brake": False,
                }
            )
        return make_command_array(commands)

    def _rule_candidate(self, obs, observations, active_index):
        pipe = self.env.pipelines[int(obs["pipe_id"])]
        max_speed = float(pipe.max_speed)
        lane = int(obs["lane_id"])
        speed = float(obs["speed"])
        target_speed = desired_speed_for_action(
            action=1,
            current_speed=speed,
            pipe_max_speed=max_speed,
            max_accel=max_accel_from_observation(obs),
            max_decel=max_decel_from_observation(obs),
        )
        action_id = 1
        return (target_speed, lane, action_id)

    def _free_flow_candidate(self, obs):
        pipe = self.env.pipelines[int(obs["pipe_id"])]
        max_speed = float(pipe.max_speed)
        speed = float(obs["speed"])
        lane = int(obs["lane_id"])
        return (
            desired_speed_for_action(
                action=2,
                current_speed=speed,
                pipe_max_speed=max_speed,
                max_accel=max_accel_from_observation(obs),
                max_decel=max_decel_from_observation(obs),
            ),
            lane,
            2,
        )

    def _lead_info(self, obs, observations, active_index):
        pipe_id = int(obs["pipe_id"])
        pipe = self.env.pipelines[pipe_id]
        lane = int(obs["lane_id"])
        pos = float(obs["pos_1d"])
        sense_range = max(1.0, float(pipe.max_speed) * 5.0)
        d_lead = sense_range
        lead_speed = float(pipe.max_speed)
        for idx in active_index.get((pipe_id, lane), []):
            other = observations[idx]
            if int(other["uav_id"]) == int(obs["uav_id"]):
                continue
            other_pos = float(other["pos_1d"])
            if other_pos > pos:
                d = other_pos - pos
                if d < d_lead:
                    d_lead = d
                    lead_speed = float(other["speed"])
        return (d_lead, lead_speed, sense_range)

    def _safe_lanes(self, obs, observations, active_index):
        pipe = self.env.pipelines[int(obs["pipe_id"])]
        current_lane = int(obs["lane_id"])
        pos = float(obs["pos_1d"])
        sense_range = float(pipe.max_speed) * 5.0
        candidates = []
        for lane in range(pipe.num_lanes):
            if lane == current_lane:
                continue
            blocked = False
            for idx in active_index.get((int(obs["pipe_id"]), lane), []):
                other = observations[idx]
                if abs(float(other["pos_1d"]) - pos) < sense_range:
                    blocked = True
                    break
            if not blocked:
                candidates.append(lane)
        return candidates

    def _nearest_empty_lane(self, obs, observations, active_index):
        pipe = self.env.pipelines[int(obs["pipe_id"])]
        current_lane = int(obs["lane_id"])
        pos = float(obs["pos_1d"])
        sense_range = float(pipe.max_speed) * 5.0
        candidates = []
        for lane in range(pipe.num_lanes):
            if lane == current_lane:
                continue
            blocked = False
            for idx in active_index.get((int(obs["pipe_id"]), lane), []):
                other = observations[idx]
                if abs(float(other["pos_1d"]) - pos) < sense_range:
                    blocked = True
                    break
            if not blocked:
                candidates.append(lane)
        if not candidates:
            return None
        return min(candidates, key=lambda lane: abs(lane - current_lane))


class _AuthoritativeControlServer:
    def __init__(
        self,
        env_snapshot,
        routing_mode="DIJKSTRA",
        avoidance_policy="rule",
        safety_substep=0.2,
    ):
        if routing_mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        self.core = AuthoritativeControlCore(
            env_snapshot,
            routing_mode=routing_mode,
            avoidance_policy=avoidance_policy,
            safety_substep=safety_substep,
        )

    def set_routing_mode(self, mode):
        if mode != "DIJKSTRA":
            raise ValueError("Only DIJKSTRA routing is available")
        return self.core.set_routing_mode(mode)

    def set_avoidance_policy(self, policy):
        if policy != "rule":
            raise ValueError("Only rule avoidance is available")
        self.avoidance_policy = "rule"

    def compute_commands(self, turn, observations, shared_context=None):
        shared_context = dict(shared_context or {})
        return self.core.compute_commands(turn, observations, shared_context)

    def prepare_initial_routes(self, turn, observations, shared_context=None):
        return self.core.prepare_initial_routes(turn, observations, shared_context)


if ray is not None:
    AuthoritativeControlServer = ray.remote(num_cpus=1)(_AuthoritativeControlServer)
else:
    AuthoritativeControlServer = _AuthoritativeControlServer
