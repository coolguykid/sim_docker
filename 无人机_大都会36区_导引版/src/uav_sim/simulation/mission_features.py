"""Optional first-version panel features shared by both enhanced executors."""
from dataclasses import replace
import math
import random

from uav_sim.airspace.environment import PickupPoint
from uav_sim.entities.partition_motion import PhysicalInvariantError
from uav_sim.utils.trace_logger import TurnTraceLogger


class MissionFeatures:
    def __init__(self, env, config):
        self.env, self.config = env, dict(config)
        count = max(1, int(config.get("num_uavs", 10000)))
        weights = {
            int(k): float(v)
            for k, v in config.get("capacity_weights", {0: 100}).items()
        }
        if any(k < 0 or not math.isfinite(v) or v < 0 for k, v in weights.items()):
            raise ValueError(
                "Capacity levels and weights must be nonnegative and finite"
            )
        total = sum(weights.values())
        capacities = []
        for cap in sorted(weights):
            capacities.extend(
                [cap] * round(count * weights[cap] / total) if total else []
            )
        capacities = capacities[:count]
        capacities.extend(
            [max(weights, key=weights.get) if total else 0] * (count - len(capacities))
        )
        random.Random(int(config.get("random_seed", 42))).shuffle(capacities)
        self._capacities = tuple(capacities)
        self.pickup_points, specs = [], []
        number = int(config.get("num_pickup_points", 0))
        locations = config.get("pickup_locations", [])
        if number < 0 or number > len(locations):
            raise ValueError("Pickup count must match configured pickup coordinates")
        for index, location in enumerate(locations[:number]):
            pid, distance = int(location["pipe_id"]), float(location["pos_1d"])
            pipe = env.pipelines[pid]
            if not math.isfinite(distance) or not 0 <= distance <= pipe.length:
                raise ValueError("Pickup lies outside the selected physical pipe")
            self.pickup_points.append(PickupPoint(index, pid, distance, capacity=0))
            specs.append(
                dict(
                    node_id="PICKUP_%d" % index,
                    pipe_id=pid,
                    distance_m=distance,
                    lane_id=int(location.get("lane_id", 0)),
                )
            )
        self.additional_specs = tuple(specs)
        self._waypoints = {}
        self.kl_history, self.gain_history = [], []
        self.trace = TurnTraceLogger(
            config.get("trace_uav_ids", []), config.get("trace_log_path")
        )

    def capacity(self, uid):
        return self._capacities[uid % len(self._capacities)]

    def initial_goal(self, uid, final_goal, maps):
        if self.capacity(uid) > 0 and self.additional_specs:
            # Spatially assign the closest configured service point to the
            # actual launch slot; no global RNG or hidden intermediate goal.
            slots = [
                (p, lane)
                for p in self.env.pipelines
                if p.height == 100
                for lane in range(p.num_lanes)
            ]
            pipe, lane = slots[uid % len(slots)]
            position = pipe.lanes[lane].start_pt
            goals = [
                maps.destination(spec["node_id"]) for spec in self.additional_specs
            ]
            index = min(
                range(len(goals)),
                key=lambda i: (math.dist(position, goals[i].position), i),
            )
            self._waypoints[uid] = index
            return goals[index]
        return final_goal

    def configure(self, uav, final_goal):
        uav.final_partition_goal = final_goal
        uav.final_dest_pipe_id = final_goal.pipe_id
        uav.phase = "TO_DEST"
        if uav.uav_id in self._waypoints:
            uav.pickup_pt = self.pickup_points[self._waypoints[uav.uav_id]]
            uav.pickup_pt.capacity += uav.capacity
            uav.pickup_pt.remaining += uav.capacity
            uav.phase = "TO_PICKUP"

    def is_final_leg(self, uav):
        return uav.uav_id not in self._waypoints

    def on_goal(self, uav, state, now):
        index = self._waypoints.get(uav.uav_id)
        if index is None:
            return True
        if (
            state.node_id != state.goal.node_id
            or math.dist(state.position, state.goal.position) > 1e-5
        ):
            raise PhysicalInvariantError(
                "Pickup must be reached at its actual coordinate"
            )
        if state.speed > 1e-6:
            raise PhysicalInvariantError("Intermediate pickup requires a physical stop")
        self._waypoints.pop(uav.uav_id)
        point = self.pickup_points[index]
        point.remaining = max(0, point.remaining - uav.capacity)
        uav.pickup_pt = None
        uav.gaussian_intensity = 0.0
        uav.phase = "TO_DEST"
        uav._road_state = replace(
            state,
            goal=uav.final_partition_goal,
            status="FLYING",
            arrival_time=None,
            route=(),
            route_index=0,
        )
        uav.state = "FLYING"
        return False

    def record_grid(self, session):
        if self.config.get("enable_kl_divergence", False):
            from uav_sim.algorithms.kl_divergence import compute_kl_divergence

            self.kl_history.append(
                float(
                    compute_kl_divergence(
                        list(session.active.values()), self.pickup_points, self.env
                    )
                )
            )
        if self.config.get("enable_channel_gain", False):
            count = sum(not r.is_dynamic or r.is_active for r in self.env.ramps)
            self.gain_history.append(40.0 * 8 * 8 / count if count else 0.0)
        if self.trace.enabled:
            from uav_sim.visualization.state_adapter import partition_update

            updates = {}
            for uid in self.trace.uav_ids:
                u = (
                    session.active.get(uid)
                    or session.pending_uavs.get(uid)
                    or session.world_objects.get(uid)
                )
                if u is not None:
                    updates[uid] = partition_update(
                        u, self.env, session.clock.time_seconds
                    )
            self.trace.log_turn(
                "1", session.routing_mode, session.clock.step_index, updates, self.env
            )

    def close(self):
        self.trace.close()
