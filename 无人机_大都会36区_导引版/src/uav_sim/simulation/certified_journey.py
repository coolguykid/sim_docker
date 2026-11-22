"""Finite central safety forecasts with genuine local navigation at entry.

Only the authoritative traffic server holds these full future trajectories.
Regional navigation independently reproduces its own current prefix at each
physically reached boundary before the certified continuation executes.
"""
from bisect import bisect_right
from dataclasses import replace
import math

from uav_sim.entities.partition_motion import PreparedMotion
from uav_sim.entities.trajectory_profile import build_prefix_profile
from .certified_traffic import CertifiedTraffic
from .journey_routes import JourneyRoutes


class _CompositeJourney:
    def __init__(self, parts):
        self.parts = tuple(parts)
        self.start_time = self.parts[0].start_time
        self.end_time = self.parts[-1].end_time
        self.pieces = tuple(piece for part in self.parts for piece in part.pieces)
        self.starts = tuple(part.start_time for part in self.parts)

    @property
    def duration(self):
        return self.end_time-self.start_time

    @property
    def end_state(self):
        return self.parts[-1].end_state

    @property
    def region_events(self):
        return tuple(event for part in self.parts for event in part.region_events)

    def sample(self, now):
        return self.parts[max(0,bisect_right(self.starts,now)-1)].sample(now)

    def shifted(self, start):
        return _CompositeJourney(tuple(part.shifted(start+part.start_time-self.start_time) for part in self.parts))


class CertifiedJourney(CertifiedTraffic):
    """Reserve a finite complete journey before admitting a cold aircraft."""
    def __init__(self, session):
        super().__init__(session)
        self.routes = JourneyRoutes(session, session._plan_many)
        self.world_candidates = {}
        self.serial_fallbacks = 0
        self.boundary_stop = bool(session.config.get('stop_at_region_boundaries', True))
        self.search_continuations = 0
        self.max_search_chunks = session.config.get('max_admission_search_chunks', 16)
        if isinstance(self.max_search_chunks,bool) or not isinstance(self.max_search_chunks,int) or self.max_search_chunks < 1:
            raise ValueError('max_admission_search_chunks must be a positive integer')

    def _build(self, state, route, start=0., stop=False):
        s = self.session
        self.profile_builds += 1
        return build_prefix_profile(state,s.graph,route,start_time=start,
                    max_accel=s.config.get('max_accel',10.),max_decel=s.config.get('max_decel',10.),
                    stop_at_end=stop,stop_at_boundaries=self.boundary_stop)

    def _plan_ready(self, ready, now):
        s = self.session
        fresh = {uid:self._uav(uid) for uid in ready
                 if self._uav(uid) is not None and uid not in self.profiles and uid not in self.world_candidates}
        if not fresh:return
        references = self.routes.forecast_many(fresh,now)
        first,continuations = {},{}
        for uid,u in fresh.items():
            part = self._build(u._road_state,references[uid],stop=not s.features.is_final_leg(u))
            first[uid] = part
            if not s.features.is_final_leg(u):
                continuations[uid] = replace(part.end_state,goal=u.final_partition_goal,
                    status='FLYING',arrival_time=None,route=(),route_index=0)
        following = self.routes.forecast_states(continuations,now) if continuations else {}
        for uid,part in first.items():
            if uid in following:
                second = self._build(continuations[uid],following[uid],start=part.end_time)
                profile = _CompositeJourney((part,second))
            else:profile = part
            thresholds = tuple(s._threshold(profile.sample(p.t0)) for p in profile.pieces)
            self.world_candidates[uid] = (profile,thresholds)

    def _offer(self, uid, now):
        s = self.session
        u = self._uav(uid)
        if u is None or uid in self.profiles:return
        if uid not in s.pending_uavs:
            raise RuntimeError('Finite-journey admission must start outside the occupied airspace')
        profile,thresholds = self.world_candidates[uid]
        earliest = now
        for chunk in range(self.max_search_chunks):
            result = self.reservations.reserve(uid,profile.pieces,earliest,
                        start_position=u._road_state.position,end_position=profile.end_state.position,
                        end_hold=False,cold=True,thresholds=thresholds)
            if result.accepted:
                break
            retry = result.retry_start_time
            if retry is None or not math.isfinite(retry) or retry <= earliest:
                break
            earliest = retry
            if chunk+1 < self.max_search_chunks:
                self.search_continuations += 1
        if not result.accepted:
            clearance = self.reservations.clearance_time(now)
            if not math.isfinite(clearance):
                raise RuntimeError('An infinite external hold prevents finite-journey admission')
            self.serial_fallbacks += 1
            result = self.reservations.reserve(uid,profile.pieces,max(now,clearance+1e-6),
                        start_position=u._road_state.position,end_position=profile.end_state.position,
                        end_hold=False,cold=True,thresholds=thresholds)
            if not result.accepted:
                raise RuntimeError('Finite safety clearance did not yield an admissible complete journey')
        self.prefix_count += 1
        self.motion_seconds[uid] += profile.duration
        self.max_wait = max(self.max_wait,result.start_time-now)
        self.total_wait += result.start_time-now
        retimed = profile.shifted(result.start_time)
        self.profiles[uid] = retimed
        u._prefix_profile = retimed
        self.world_candidates.pop(uid)
        self.remaining.pop(uid,None)
        self._push(result.start_time,'DEPART',uid)
        parts = retimed.parts if isinstance(retimed,_CompositeJourney) else (retimed,)
        for index,part in enumerate(parts):
            for when,state in part.region_events:
                self._push(when,'REGION',(uid,state,tuple(state.route[state.route_index:])))
            if index < len(parts)-1:
                self._push(part.end_time,'PICKUP',(uid,part.end_state,parts[index+1]))
        self._push(result.end_time,'FINISH',uid)

    def _extra_events(self,batch,now):
        s = self.session
        entries = {}
        def verify():
            if entries:
                self.routes.verify_many(entries,now)
                entries.clear()
        for _,_,kind,payload in batch:
            if kind == 'REGION':
                uid,state,remaining = payload
                if uid in entries:verify()
                if self.boundary_stop and state.speed > 1e-6:
                    raise RuntimeError('Region-entry validation requires a physical stationary crossing')
                s.active[uid].commit_partition_step(PreparedMotion(state,(),()))
                entries[uid] = (state,remaining)
            elif kind == 'PICKUP':
                verify()
                uid,state,following = payload
                u = s.active[uid]
                u.commit_partition_step(PreparedMotion(state,(),()))
                if s.on_partition_goal(u,state,now):
                    raise RuntimeError('An intermediate pickup was treated as the final sink')
                self.routes.verify_many({uid:(u._road_state,tuple(following.end_state.route))},now)
        verify()

    def _arrive(self,uid,u,state,now):
        super()._arrive(uid,u,state,now)
        self.session._mission_cost_maps.pop(uid, None)
        self.routes.discard((uid,))

    def metrics(self):
        values = super().metrics()
        values.update(execution_engine='certified_journey',ground_serialization_fallbacks=self.serial_fallbacks,
                      ground_search_continuations=self.search_continuations,
                      max_admission_search_chunks=self.max_search_chunks,
                      journey_routing=self.routes.metrics(),region_boundary_stop=self.boundary_stop)
        return values
