"""Predictive finite-acceleration safety for partitioned original airspace.

Every accepted step includes an executable full-braking continuation.  That
continuation is continuously audited in 3D, including its stationary tail.
The controller prepares proposals only; its caller owns the atomic commit.

This is a conservative backup controller, not a complete multi-agent motion
planner.  A pre-existing state without a safe braking certificate is reported
explicitly.  No aircraft is teleported, clipped, or removed to repair it.
"""
from collections import defaultdict
from collections.abc import Mapping
from dataclasses import dataclass, replace
import math
from types import SimpleNamespace

from uav_sim.algorithms.avoidance_features import collision_threshold_for_height
from uav_sim.algorithms.continuous_safety import (
    Conflict, TrajectoryPiece, _swept_bounds, audit_trajectories, min_distance,
)
from uav_sim.entities.partition_motion import PhysicalInvariantError, advance, endpoints


class InfeasibleSafetyState(RuntimeError):
    """The supplied airborne state lacks the controller's braking invariant."""


@dataclass(frozen=True)
class _Prediction:
    proposal: object
    profile: tuple
    acceleration: float
    required_distance: float
    piece_boxes: tuple = ()
    bounds_lower: tuple = (0.0, 0.0, 0.0)
    bounds_upper: tuple = (0.0, 0.0, 0.0)


def _make_prediction(proposal, profile, acceleration, required_distance):
    profile = tuple(profile)
    if not profile:
        pos = proposal.state.position
        return _Prediction(proposal, profile, acceleration, required_distance, (), pos, pos)
    boxes = tuple(_swept_bounds(p) for p in profile)
    low0 = min(b[0][0] for b in boxes)
    low1 = min(b[0][1] for b in boxes)
    low2 = min(b[0][2] for b in boxes)
    high0 = max(b[1][0] for b in boxes)
    high1 = max(b[1][1] for b in boxes)
    high2 = max(b[1][2] for b in boxes)
    return _Prediction(
        proposal, profile, acceleration, required_distance,
        boxes, (low0, low1, low2), (high0, high1, high2),
    )


def _fast_pair_conflict(uid_a, pred_a, uid_b, pred_b):
    """Exact closest conflict between two _Prediction objects using cached AABBs."""
    if not pred_a.profile or not pred_b.profile:
        return None
    req_a = pred_a.required_distance
    req_b = pred_b.required_distance
    threshold = req_a if req_a >= req_b else req_b
    la, ua = pred_a.bounds_lower, pred_a.bounds_upper
    lb, ub = pred_b.bounds_lower, pred_b.bounds_upper
    if (
        la[0] - ub[0] > threshold or lb[0] - ua[0] > threshold
        or la[1] - ub[1] > threshold or lb[1] - ua[1] > threshold
        or la[2] - ub[2] > threshold or lb[2] - ua[2] > threshold
    ):
        return None
    tolerance = 16 * math.ulp(max(1.0, threshold))
    limit = threshold + tolerance
    best_dist = math.inf
    best_time = math.inf
    found = False
    prof_b = pred_b.profile
    boxes_b = pred_b.piece_boxes
    for pa, (pla, pua) in zip(pred_a.profile, pred_a.piece_boxes):
        ta0, ta1 = pa.t0, pa.t1
        for pb, (plb, pub) in zip(prof_b, boxes_b):
            if max(ta0, pb.t0) > min(ta1, pb.t1):
                continue
            if (
                pla[0] - pub[0] > threshold or plb[0] - pua[0] > threshold
                or pla[1] - pub[1] > threshold or plb[1] - pua[1] > threshold
                or pla[2] - pub[2] > threshold or plb[2] - pua[2] > threshold
            ):
                continue
            distance, time = min_distance(pa, pb)
            if distance <= limit:
                if not found or (distance, time) < (best_dist, best_time):
                    best_dist = distance
                    best_time = time
                    found = True
    if not found:
        return None
    return Conflict(uid_a, uid_b, best_time, best_dist, threshold)


def _fast_fleet_audit(selected):
    """Sweep precomputed UAV AABBs and run exact piece-level closest approach."""
    items = [(uid, pred) for uid, pred in selected.items() if pred.profile]
    if len(items) < 2:
        return []
    spreads = [
        max(p.bounds_lower[axis] for _, p in items) - min(p.bounds_lower[axis] for _, p in items)
        for axis in range(3)
    ]
    axis = 0
    if spreads[1] > spreads[axis]:
        axis = 1
    if spreads[2] > spreads[axis]:
        axis = 2
    max_req = max(p.required_distance for _, p in items)
    items.sort(key=lambda item: item[1].bounds_lower[axis])
    active = []
    conflicts = []
    for uid, pred in items:
        limit_val = pred.bounds_lower[axis] - max_req
        active = [other for other in active if other[1].bounds_upper[axis] >= limit_val]
        for other_uid, other_pred in active:
            c = _fast_pair_conflict(other_uid, other_pred, uid, pred)
            if c is not None:
                conflicts.append(c)
        active.append((uid, pred))
    return conflicts


class RegionalControl:
    """Bounded acceleration choices with continuous current+backup validation."""

    def __init__(self, graph, max_accel=10., max_decel=10., min_distance_m=None,
                 safety_margin_m=1e-4, max_iterations=None):
        self.graph = graph
        self.max_accel = self._positive(max_accel, 'max_accel')
        self.max_decel = self._positive(max_decel, 'max_decel')
        self.min_distance_m = min_distance_m
        self.safety_margin_m = float(safety_margin_m)
        if not math.isfinite(self.safety_margin_m) or self.safety_margin_m < 0:
            raise ValueError('safety_margin_m must be finite and nonnegative')
        if max_iterations is not None and int(max_iterations) < 1:
            raise ValueError('max_iterations must be positive')
        self.max_iterations = max_iterations

    @staticmethod
    def _positive(value, name):
        value = float(value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError('%s must be finite and positive' % name)
        return value

    def _bounds(self, uav):
        return (min(self.max_accel, self._positive(getattr(uav, 'max_accel', self.max_accel), 'max_accel')),
                min(self.max_decel, self._positive(getattr(uav, 'max_decel', self.max_decel), 'max_decel')))

    def _distance(self, uid, uav):
        configured = self.min_distance_m
        if isinstance(configured, Mapping):
            if uid not in configured:
                raise ValueError('missing minimum distance for UAV %r' % (uid,))
            value = float(configured[uid])
        elif configured is not None:
            value = float(configured)
        else:
            state = uav._road_state
            value = collision_threshold_for_height(math.ceil((state.position[2]-1e-7) / 100.) * 100.)
        if not math.isfinite(value) or value < 0:
            raise ValueError('minimum distance must be finite and nonnegative')
        return value + self.safety_margin_m

    def _profile_distance(self, uid, uav, profile):
        value = self._distance(uid, uav)
        if self.min_distance_m is not None:
            return value
        # Reserve the increased height separation before the part of a ramp
        # actually reachable by this step and its braking continuation.  A
        # distant high destination must not reduce current low-lane capacity.
        for piece in profile:
            for height in (piece.position0[2], piece.position_at(piece.t1)[2]):
                layer = math.ceil((height-1e-7) / 100.) * 100.
                value = max(value, collision_threshold_for_height(layer) + self.safety_margin_m)
        return value

    @staticmethod
    def _priority(uid, uav):
        # Integer IDs retain age ordering; repr supplies deterministic ordering
        # for third-party hashable IDs without requiring mixed-type comparisons.
        return (getattr(uav, 'spawn_turn', 0),
                (0, uid) if isinstance(uid, (int, float)) else (1, repr(uid)))

    @staticmethod
    def _stationary(uid, start, end, position):
        return TrajectoryPiece(uid, start, end, position, (0., 0., 0.), (0., 0., 0.))

    def _forward_segments(self, state):
        """Known forward lane geometry, used only at an actual pair conflict."""
        state = replace(state)
        result = []
        for index, edge_id in enumerate(state.route[state.route_index:]):
            edge = self.graph.edges[edge_id]
            if edge.kind == 'RAMP' and state.transit_ramp != edge.ramp_id:
                state.transit_ramp = edge.ramp_id
                state.entry_lane = state.lane_id
                state.exit_lane = min(state.goal.lane_id, edge.lane_count - 1)
            a, b = endpoints(edge, state, self.graph)
            result.append((state.position if index == 0 else a, b))
            if edge.ramp_id is not None and self.graph.nodes[edge.target].pipe_id is not None:
                state.lane_id = state.exit_lane
                state.transit_ramp = None
        return result

    @staticmethod
    def _point_segment_distance(point, a, b):
        vx, vy, vz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        length2 = vx * vx + vy * vy + vz * vz
        if length2:
            fraction = ((point[0] - a[0]) * vx + (point[1] - a[1]) * vy + (point[2] - a[2]) * vz) / length2
            if fraction < 0.0:
                fraction = 0.0
            elif fraction > 1.0:
                fraction = 1.0
        else:
            fraction = 0.0
        dx = point[0] - (a[0] + fraction * vx)
        dy = point[1] - (a[1] + fraction * vy)
        dz = point[2] - (a[2] + fraction * vz)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _active_edge_id(self, state):
        if state.edge_id:
            return state.edge_id
        if state.route_index < len(state.route):
            return state.route[state.route_index]
        return ''

    def _order_pair(self, pair, fleet, cold, same_lane_positions):
        """Return (loser, winner, is_following) so trailing or entering aircraft yield first."""
        uid_a, uid_b = pair
        # 1. Exact same-lane following order
        for positions in same_lane_positions:
            if uid_a in positions and uid_b in positions:
                if positions[uid_a] <= positions[uid_b]:
                    return uid_a, uid_b, True
                return uid_b, uid_a, True

        # 2. Uncertified pending admission yields to already-airborne traffic
        a_cold, b_cold = (uid_a in cold), (uid_b in cold)
        if a_cold != b_cold:
            return (uid_a, uid_b, True) if a_cold else (uid_b, uid_a, True)

        state_a = fleet[uid_a]._road_state
        state_b = fleet[uid_b]._road_state
        edge_a = self._active_edge_id(state_a)
        edge_b = self._active_edge_id(state_b)

        # 3. Same physical edge (including adjacent lanes within 3D separation threshold)
        if edge_a and edge_a == edge_b and abs(state_a.progress - state_b.progress) > 1e-4:
            if state_a.progress < state_b.progress:
                return uid_a, uid_b, True
            return uid_b, uid_a, True

        # 4. Consecutive edges along the planned route corridor
        upcoming_a = state_a.route[state_a.route_index + 1 : state_a.route_index + 5]
        upcoming_b = state_b.route[state_b.route_index + 1 : state_b.route_index + 5]
        if edge_b and edge_b in upcoming_a and edge_a not in upcoming_b:
            return uid_a, uid_b, True
        if edge_a and edge_a in upcoming_b and edge_b not in upcoming_a:
            return uid_b, uid_a, True

        # 5. General priority fallback (older spawn_turn / smaller UID wins)
        loser, winner = sorted(pair, key=lambda uid: self._priority(uid, fleet[uid]), reverse=True)
        return loser, winner, False

    def compute(self, uavs, now, dt, next_region_callback=None, new_ids=(),
                locked_predictions=None):
        now, dt = float(now), float(dt)
        if not math.isfinite(now) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('now must be finite and dt finite and positive')
        fleet = {uid: u for uid, u in uavs.items() if u._road_state.status != 'ARRIVED'}
        bounds = {uid: self._bounds(u) for uid, u in fleet.items()}
        thresholds = {uid: self._distance(uid, u) for uid, u in fleet.items()}
        # All WAITING aircraft in new_ids (both ground spawns and cross-region
        # portal handoffs) lack a local regional certificate and are cold admissions.
        cold = {uid for uid in new_ids if uid in fleet and
                fleet[uid]._road_state.status == 'WAITING'}
        deferred = set()
        metrics = {'certified': False, 'safety_modified_count': 0, 'hard_brake_count': 0,
                   'rollback_count': 0, 'resolved_conflict_count': 0,
                   'prediction_count': 0, 'following_bound_count': 0,
                   'deferred_count': 0, 'audit_count': 0, 'fallback_count': 0}
        if not fleet:
            metrics['certified'] = True
            return {'proposals': {}, 'predictions': {}, 'deferred_ids': deferred, 'metrics': metrics}
        horizon = dt + max((u._road_state.speed + bounds[uid][0] * dt) / bounds[uid][1]
                           for uid, u in fleet.items()) + 1e-8
        end_time = now + horizon
        if locked_predictions:
            locked_end = max(
                (p.profile[-1].t1 for p in locked_predictions.values() if p.profile),
                default=end_time,
            )
            if locked_end > end_time:
                end_time = locked_end
                horizon = end_time - now
        cache = {}
        forward_cache = {}

        def get_corridor(uid):
            if uid not in forward_cache:
                forward_cache[uid] = self._forward_segments(fleet[uid]._road_state)
            return forward_cache[uid]

        def invalidate_cache(uid):
            for key in [k for k in cache if k[0] == uid]:
                del cache[key]

        def defer_cold(uid):
            deferred.add(uid)
            rs = fleet[uid]._road_state
            if rs.speed > 0.0:
                rs.speed = 0.0
                rs.last_acceleration = 0.0
                invalidate_cache(uid)

        def predict(uid, acceleration):
            key = (uid, acceleration)
            if key in cache:
                return cache[key]
            metrics['prediction_count'] += 1
            state = fleet[uid]._road_state
            try:
                proposed = advance(state, self.graph, acceleration, now, dt, next_region_callback)
                profile = list(proposed.pieces)
                if proposed.state.status != 'ARRIVED':
                    if not profile:
                        profile.append(self._stationary(uid, now, now + dt, state.position))
                    braking = advance(proposed.state, self.graph, -bounds[uid][1],
                                      now + dt, max(0., end_time - now - dt))
                    profile.extend(braking.pieces)
                    if braking.state.status != 'ARRIVED':
                        if braking.state.speed > 1e-7:
                            raise PhysicalInvariantError('backup did not stop')
                        covered = profile[-1].t1 if profile else now
                        if covered < end_time:
                            profile.append(self._stationary(uid, covered, end_time, braking.state.position))
                result = _make_prediction(
                    proposed, profile, acceleration,
                    self._profile_distance(uid, fleet[uid], profile))
            except PhysicalInvariantError:
                result = None
            cache[key] = result
            return result

        def highest_feasible(uid, upper, predicate=lambda candidate: True, steps=8):
            candidate = predict(uid, upper)
            if candidate is not None and predicate(candidate):
                return candidate
            lower = -bounds[uid][1]
            best = predict(uid, lower)
            if best is None or not predicate(best):
                return None
            # Every returned value is actually simulated and certified; the
            # binary search only improves progress, not numerical safety.
            for _ in range(steps):
                middle = (lower + upper) * .5
                trial = predict(uid, middle)
                if trial is not None and predicate(trial):
                    lower, best = middle, trial
                else:
                    upper = middle
            return best

        def pair_safe(a, b):
            return _fast_pair_conflict(a.proposal.state.uav_id, a, b.proposal.state.uav_id, b) is None

        # Fast incremental admission mode when existing airborne trajectories
        # in the current frame are already certified and locked.
        if locked_predictions is not None:
            predictions = {}

            def clear_of_all(uid, cand):
                for l_uid, l_pred in locked_predictions.items():
                    if l_uid != uid and _fast_pair_conflict(uid, cand, l_uid, l_pred) is not None:
                        return False
                for o_uid, o_pred in predictions.items():
                    if o_uid != uid and _fast_pair_conflict(uid, cand, o_uid, o_pred) is not None:
                        return False
                return True

            for uid in sorted(fleet, key=lambda key: self._priority(key, fleet[key])):
                occ = self._stationary(uid, now, now, fleet[uid]._road_state.position)
                occ_pred = _make_prediction(
                    SimpleNamespace(state=fleet[uid]._road_state),
                    (occ,), 0.0, thresholds[uid])
                if not clear_of_all(uid, occ_pred):
                    defer_cold(uid)
                    continue
                chosen = highest_feasible(uid, bounds[uid][0], lambda cand: clear_of_all(uid, cand), steps=8)
                if chosen is None and fleet[uid]._road_state.speed > 0.0:
                    orig_speed = fleet[uid]._road_state.speed
                    for factor in (0.5, 0.0):
                        fleet[uid]._road_state.speed = orig_speed * factor
                        fleet[uid]._road_state.last_acceleration = 0.0
                        invalidate_cache(uid)
                        chosen = highest_feasible(uid, bounds[uid][0], lambda cand: clear_of_all(uid, cand), steps=8)
                        if chosen is not None:
                            break
                if chosen is None:
                    defer_cold(uid)
                else:
                    predictions[uid] = chosen

            metrics['certified'] = True
            metrics['certified_horizon_seconds'] = horizon
            metrics['deferred_count'] = len(deferred)
            return {
                'proposals': {uid: value.proposal for uid, value in predictions.items()},
                'predictions': predictions,
                'deferred_ids': deferred,
                'metrics': metrics,
            }

        # Admission happens outside the airspace.  Initial overlap may reject a
        # cold WAITING request, but never discards an airborne stationary UAV.
        accepted = set(fleet).difference(cold)
        if cold:
            occupants = [self._stationary(uid, now, now, u._road_state.position) for uid, u in fleet.items()]
            admission_neighbors = defaultdict(set)
            for conflict in audit_trajectories(occupants, thresholds):
                admission_neighbors[conflict.uav_id_a].add(conflict.uav_id_b)
                admission_neighbors[conflict.uav_id_b].add(conflict.uav_id_a)
            for uid in sorted(cold, key=lambda key: self._priority(key, fleet[key])):
                if admission_neighbors[uid].intersection(accepted):
                    defer_cold(uid)
                else:
                    accepted.add(uid)

        predictions = {}
        for uid in fleet:
            if uid in deferred:
                continue
            prediction = highest_feasible(uid, bounds[uid][0])
            if prediction is None and uid in cold and fleet[uid]._road_state.speed > 0.0:
                # A handed-off aircraft may arrive faster than the target
                # region's entry ramp/edge speed cap allows.  Modulate its
                # portal entry speed before deferring it.
                orig_speed = fleet[uid]._road_state.speed
                for factor in (0.5, 0.25, 0.0):
                    fleet[uid]._road_state.speed = orig_speed * factor
                    fleet[uid]._road_state.last_acceleration = 0.0
                    invalidate_cache(uid)
                    prediction = highest_feasible(uid, bounds[uid][0])
                    if prediction is not None:
                        break
            if prediction is None:
                if uid in cold:
                    defer_cold(uid)
                    continue
                raise InfeasibleSafetyState('Infeasible braking certificate for airborne UAV %r' % (uid,))
            predictions[uid] = prediction

        # An index gives following traffic a stopping-distance bound before
        # the general crossing/ramp audit.  Support both synthetic node.pipe_id
        # and database edge.pipe_id topologies.
        lane_index = defaultdict(list)
        for uid in predictions:
            state = fleet[uid]._road_state
            if state.transit_ramp is not None:
                continue
            edge_id = self._active_edge_id(state)
            if not edge_id or edge_id not in self.graph.edges:
                continue
            edge = self.graph.edges[edge_id]
            if edge.kind != 'FLY':
                continue
            node = self.graph.nodes[state.node_id]
            if node.pipe_id is not None:
                lane_key = ('pipe', node.pipe_id, state.lane_id)
                longitudinal = float(node.distance_m or 0.) + state.progress
            else:
                lane_key = ('edge', edge.edge_id, state.lane_id)
                longitudinal = float(state.progress)
            lane_index[lane_key].append((longitudinal, uid))

        same_lane_positions = []
        for indexed in lane_index.values():
            if len(indexed) < 2:
                continue
            indexed.sort(key=lambda row: row[0])
            same_lane_positions.append({uid: pos for pos, uid in indexed})
            # Resolve from the front backward, so a platoon may accelerate
            # together against its selected common-dt braking certificate.
            for (rear_pos, rear), (front_pos, front) in reversed(list(zip(indexed, indexed[1:]))):
                front_state = fleet[front]._road_state
                front_candidate = predictions[front].proposal.state
                front_budget = (math.dist(front_state.position, front_candidate.position) +
                                front_candidate.speed ** 2 / (2 * bounds[front][1]))
                budget = (front_pos - rear_pos - max(predictions[rear].required_distance, predictions[front].required_distance) +
                          front_budget)
                rear_state = fleet[rear]._road_state
                def follows(candidate):
                    moved = math.dist(rear_state.position, candidate.proposal.state.position)
                    return moved + candidate.proposal.state.speed ** 2 / (2 * bounds[rear][1]) <= budget
                if follows(predictions[rear]):
                    continue
                limited = highest_feasible(rear, predictions[rear].acceleration, follows, steps=6)
                if limited is None and rear in cold and fleet[rear]._road_state.speed > 0.0:
                    fleet[rear]._road_state.speed = 0.0
                    fleet[rear]._road_state.last_acceleration = 0.0
                    invalidate_cache(rear)
                    base_pred = highest_feasible(rear, bounds[rear][0])
                    if base_pred is not None:
                        predictions[rear] = base_pred
                        limited = highest_feasible(rear, base_pred.acceleration, follows, steps=6)
                if limited is not None and limited.acceleration < predictions[rear].acceleration:
                    predictions[rear] = limited
                    metrics['following_bound_count'] += 1

        def audit(selected):
            metrics['audit_count'] += 1
            return _fast_fleet_audit(selected)

        def reduce_for_pair(uid, other, require_corridor_clearance=True):
            original = predictions[uid]
            low = -bounds[uid][1]
            if original.acceleration <= low + 1e-10:
                return None
            upper = original.acceleration
            corridor = get_corridor(other) if require_corridor_clearance else ()
            other_pred = predictions[other]

            def permits_priority_passage(candidate):
                if not require_corridor_clearance or candidate.proposal.state.status == 'ARRIVED':
                    return True
                point = candidate.profile[-1].position_at(candidate.profile[-1].t1)
                clearance = max(candidate.required_distance, other_pred.required_distance)
                return all(self._point_segment_distance(point, a, b) > clearance for a, b in corridor)

            # A crossing need not be monotone in acceleration: intermediate
            # timing can be safe even when both endpoint choices conflict.
            for index in range(1, 9):
                acceleration = original.acceleration + (low - original.acceleration) * index / 8
                trial = predict(uid, acceleration)
                if trial is not None and pair_safe(trial, other_pred) and permits_priority_passage(trial):
                    best, lower = trial, acceleration
                    for _ in range(5):
                        middle = (lower + upper) * .5
                        candidate = predict(uid, middle)
                        if candidate is not None and pair_safe(candidate, other_pred) and permits_priority_passage(candidate):
                            best, lower = candidate, middle
                        else:
                            upper = middle
                    return best
                upper = acceleration
            return None

        def search_partner_with_fixed(moving_uid, fixed_pred):
            """Find highest safe acceleration for moving_uid against a fixed partner prediction."""
            high = predictions[moving_uid].acceleration
            low = -bounds[moving_uid][1]
            for index in range(0, 9):
                acc = high + (low - high) * index / 8
                trial = predict(moving_uid, acc)
                if trial is not None and pair_safe(trial, fixed_pred):
                    return trial
            return None

        # Build initial conflict table once, then update incrementally per modified UAV.
        conflicts_by_pair = {
            frozenset((c.uav_id_a, c.uav_id_b)): c
            for c in audit(predictions)
        }

        def update_uav_conflicts(changed_uids):
            metrics['audit_count'] += 1
            changed_set = set(changed_uids)
            for pair_key in [k for k in conflicts_by_pair if k & changed_set]:
                del conflicts_by_pair[pair_key]
            for uid in changed_set:
                pred_u = predictions.get(uid)
                if pred_u is None:
                    continue
                for other_uid, pred_o in predictions.items():
                    if other_uid == uid:
                        continue
                    c = _fast_pair_conflict(uid, pred_u, other_uid, pred_o)
                    if c is not None:
                        conflicts_by_pair[frozenset((uid, other_uid))] = c

        limit = self.max_iterations or max(32, 6 * len(predictions))
        for iteration in range(limit + 1):
            if not conflicts_by_pair:
                break
            metrics['rollback_count'] += 1
            if iteration == limit:
                # The previous committed certificate supplies this finite
                # fallback.  Recheck it against current geometry and closures.
                metrics['fallback_count'] += 1
                fallback = {uid: predict(uid, -bounds[uid][1]) for uid in predictions}
                # A pending admission has no previous committed certificate.
                # It may therefore be retried instead of invalidating the
                # guaranteed backup of the already airborne fleet.
                for uid in cold.intersection(fallback):
                    if fallback[uid] is None:
                        del fallback[uid]
                        defer_cold(uid)
                if not any(value is None for value in fallback.values()):
                    fallback_conflicts = audit(fallback)
                    for value in fallback_conflicts:
                        for uid in (value.uav_id_a, value.uav_id_b):
                            if uid in cold and uid in fallback:
                                del fallback[uid]
                                defer_cold(uid)
                if any(value is None for value in fallback.values()) or audit(fallback):
                    raise InfeasibleSafetyState('Infeasible fleet braking certificate after bounded safety search')
                predictions = fallback
                break

            conflict = min(conflicts_by_pair.values(), key=lambda value: value.time_s)
            pair = (conflict.uav_id_a, conflict.uav_id_b)
            loser, winner, is_following = self._order_pair(pair, fleet, cold, same_lane_positions)

            changed_uids = []
            if is_following:
                # Trailing or newly entering aircraft yields along the shared corridor
                # before the leading aircraft is ever asked to decelerate.
                for uid, other in ((loser, winner), (winner, loser)):
                    improved = reduce_for_pair(uid, other, require_corridor_clearance=False)
                    if improved is not None:
                        predictions[uid] = improved
                        changed_uids = [uid]
                        break
            else:
                # Crossing/merging routes: try keeping loser outside winner's corridor
                # first, then allow loser to yield in-place before slowing winner.
                for uid, other, req_clear in (
                    (loser, winner, True),
                    (loser, winner, False),
                    (winner, loser, True),
                    (winner, loser, False),
                ):
                    improved = reduce_for_pair(uid, other, require_corridor_clearance=req_clear)
                    if improved is not None:
                        predictions[uid] = improved
                        changed_uids = [uid]
                        break

            # Pass 3: if loser is a cold handoff with speed > 0, try resetting its
            # portal entry speed to 0 before deferring or hard-braking the pair
            if not changed_uids:
                for uid, other in ((loser, winner), (winner, loser)):
                    if uid in cold and fleet[uid]._road_state.speed > 0.0:
                        fleet[uid]._road_state.speed = 0.0
                        fleet[uid]._road_state.last_acceleration = 0.0
                        invalidate_cache(uid)
                        trial = highest_feasible(
                            uid, bounds[uid][0],
                            lambda cand: pair_safe(cand, predictions[other]))
                        if trial is not None:
                            predictions[uid] = trial
                            changed_uids = [uid]
                            break

            # Pass 4: defer unsafe cold admission, or coordinate joint braking
            if not changed_uids:
                candidates = [uid for uid in pair if uid in cold]
                if candidates:
                    uid = max(candidates, key=lambda key: self._priority(key, fleet[key]))
                    del predictions[uid]
                    defer_cold(uid)
                    changed_uids = [uid]
                else:
                    # Try full brake on loser while finding compatible acceleration for winner
                    brake_loser = predict(loser, -bounds[loser][1])
                    brake_winner = predict(winner, -bounds[winner][1])
                    if brake_loser is not None and brake_winner is not None and pair_safe(brake_loser, brake_winner):
                        partner_w = search_partner_with_fixed(winner, brake_loser)
                        predictions[loser] = brake_loser
                        predictions[winner] = partner_w if partner_w is not None else brake_winner
                        changed_uids = [loser, winner]
                    elif brake_loser is not None:
                        partner_w = search_partner_with_fixed(winner, brake_loser)
                        if partner_w is not None:
                            predictions[loser] = brake_loser
                            predictions[winner] = partner_w
                            changed_uids = [loser, winner]
                    if not changed_uids and brake_winner is not None:
                        partner_l = search_partner_with_fixed(loser, brake_winner)
                        if partner_l is not None:
                            predictions[winner] = brake_winner
                            predictions[loser] = partner_l
                            changed_uids = [loser, winner]
                    if not changed_uids:
                        raise InfeasibleSafetyState(
                            'Infeasible airborne separation/braking certificate for UAVs %r and %r' % pair)

            update_uav_conflicts(changed_uids)
            metrics['resolved_conflict_count'] += 1

        metrics['certified'] = True
        metrics['certified_horizon_seconds'] = horizon
        metrics['deferred_count'] = len(deferred)
        metrics['safety_modified_count'] = sum(value.acceleration < bounds[uid][0] - 1e-7
                                             for uid, value in predictions.items())
        metrics['hard_brake_count'] = sum(value.acceleration <= -bounds[uid][1] + 1e-7
                                        for uid, value in predictions.items())
        return {'proposals': {uid: value.proposal for uid, value in predictions.items()},
                'predictions': predictions,
                'deferred_ids': deferred, 'metrics': metrics}
