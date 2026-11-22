"""Central ATC forecasts, separate from genuine region-by-region navigation.

The safety server retains the world and predicts a finite complete journey so
that aircraft cannot deadlock at mutually occupied intermediate stops. These
reference routes are never sent to regional/onboard planners. Their actual
local decisions are recomputed at each reached boundary and checked against
the certificate before the next region is executed.
"""
from uav_sim.distributed.route_planning_server import PartitionRoutePlanningCore


class ReferenceMismatch(RuntimeError):
    pass


class JourneyRoutes:
    def __init__(self, session, plan_many=None):
        self.session = session
        self.reference = PartitionRoutePlanningCore(session.maps, full_route=True)
        self.plan_many = self._plan_default if plan_many is None else plan_many
        self.forecasts = 0
        self.verified_entries = 0

    def _plan_default(self, backend, requests, now):
        return backend.plan_many(requests, self.session.clock.context(self.session.graph.version, now))

    def _expected(self, edges):
        edges = tuple(edges)
        if self.session.routing_mode == 'centralized_full':
            return edges
        result = []
        for edge_id in edges:
            result.append(edge_id)
            if self.session.graph.edges[edge_id].kind == 'BOUNDARY':
                break
        return tuple(result)

    def forecast_states(self, states, now):
        requests = {uid: dict(start_node_id=s.node_id, region_id=s.region_id, goal_node_id=s.goal.node_id)
                    for uid, s in states.items()}
        plans = self.plan_many(self.reference, requests, now)
        self.forecasts += len(plans)
        results = {}
        for uid, plan in plans.items():
            if plan.status != 'OK':
                raise ReferenceMismatch('ATC cannot certify a reachable complete journey for UAV %s: %s' % (uid, plan.reason))
            results[uid] = tuple(plan.edge_ids)
        return results

    def forecast_many(self, uavs, now):
        if self.session.routing_mode == 'centralized_full':
            results = {uid: tuple(u._road_state.route[u._road_state.route_index:]) for uid, u in uavs.items()}
        else:
            results = self.forecast_states({uid:u._road_state for uid,u in uavs.items()}, now)
        for uid, edges in results.items():
            actual = tuple(uavs[uid]._road_state.route[uavs[uid]._road_state.route_index:])
            if actual != self._expected(edges):
                raise ReferenceMismatch('Initial actual route differs from ATC reference for UAV %s' % uid)
        return results

    def verify_many(self, entries, now):
        if self.session.routing_mode == 'centralized_full' or not entries:
            return
        requests = {uid:dict(start_node_id=s.node_id, region_id=s.region_id, goal_node_id=s.goal.node_id)
                    for uid,(s,edges) in entries.items()}
        plans = self.plan_many(self.session.backend, requests, now)
        for uid, plan in plans.items():
            if plan.status != 'OK' or tuple(plan.edge_ids) != self._expected(entries[uid][1]):
                raise ReferenceMismatch('Actual region-entry route differs from the continuous safety certificate for UAV %s at %.12gs' % (uid,now))
            self.verified_entries += 1

    def discard(self, uids):
        self.reference.discard(uids)

    def metrics(self):
        return dict(server_safety_forecast_enabled=True, server_safety_forecast_requests=self.forecasts,
                    verified_regional_entry_plans=self.verified_entries,
                    server_safety_forecast_routing=self.reference.metrics())
