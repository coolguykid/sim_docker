"""单机运动仿真：有向几何最短路、匝道瞬移、外边界随机到达与离开。"""
from __future__ import annotations

from bisect import bisect_left, bisect_right
from collections import Counter, defaultdict, deque
from dataclasses import dataclass, field
import heapq
import math
import random

from .control import Controller
from .model import Network, Point


@dataclass(frozen=True)
class Station:
    id: int
    pipe_id: str
    distance_m: float
    region_id: str


@dataclass
class Drone:
    id: int
    origin: Station
    destination: Station
    pipe_id: str
    distance_m: float
    started_s: float = 0.0  # 外部请求时刻；任务总用时包含入网前的排队。
    waiting: bool = False
    route: list[tuple[str, float]] = field(default_factory=list)
    route_index: int = 0


@dataclass
class ArrivalClock:
    """安全等待之后的剩余指数风险量；变速只改变未来的累计速率。"""
    last_request_s: float
    clock_s: float
    remaining_hazard: float

    def advance(self, stop, headway, rate):
        active_start = max(self.clock_s, self.last_request_s+headway)
        self.remaining_hazard = max(0.0, self.remaining_hazard-rate*max(0.0, stop-active_start))
        self.clock_s = stop


class FlightGraph:
    """节点位于匝道端点、区域分界和任务站；只允许沿管道方向前进。

    权重为飞行米数，匝道为零。等距离时以边数作第二判据，消除零权环。
    边长以整数微米计算；同一图实例按目标节点共享反向距离表。
    实际当前位置和段内退出前缀逐架处理，并列最短路线逐架随机选择。
    """
    DISTANCE_SCALE = 1_000_000
    def __init__(self, net: Network, stations: list[Station], blocked_segments=()):
        self.net = net
        self.blocked_segments = frozenset(blocked_segments)
        self.offsets = {}
        points = defaultdict(set)
        for p in net.pipes.values():
            if p.is_open:
                points[p.id].update((0.0, round(math.dist(p.start, p.end), 6)))
                end = 0.0
                self.offsets[p.id] = []
                for sid in p.segment_ids:
                    end = round(end+net.segments[sid].length_m, 6)
                    self.offsets[p.id].append((end,sid))
                    points[p.id].add(end)
        for s in stations:
            if s.pipe_id in points:
                points[s.pipe_id].add(round(s.distance_m, 6))
        ramps = []
        for r in net.ramps.values():
            if not net.ramp_is_open(r.id):
                continue
            a = net.segments[r.source_segment].pipe_id
            b = net.segments[r.target_segment].pipe_id
            x = round(math.dist(net.pipes[a].start, r.start), 6)
            y = round(math.dist(net.pipes[b].start, r.end), 6)
            points[a].add(x)
            points[b].add(y)
            ramps.append(((a, x), (b, y)))
        self.positions = {p: sorted(v) for p, v in points.items()}
        self.nodes = [(p, x) for p, values in self.positions.items() for x in values]
        self.index = {n: i for i, n in enumerate(self.nodes)}
        self.edges = [[] for _ in self.nodes]
        self.integer_edges = [[] for _ in self.nodes]
        self.reverse_edges = [[] for _ in self.nodes]
        def edge(a, b, distance):
            i, j = self.index[a], self.index[b]
            self.edges[i].append((j, distance))
            weight = round(distance*self.DISTANCE_SCALE)
            self.integer_edges[i].append((j,weight))
            self.reverse_edges[j].append((i,weight))
        for p, values in self.positions.items():
            for a, b in zip(values, values[1:]):
                if self.segment_at(p,(a+b)/2) not in self.blocked_segments:
                    edge((p, a), (p, b), b-a)
        for a, b in ramps:
            if self.segment_at(*b) not in self.blocked_segments:
                edge(a, b, 0.0)
        self._reachable_exits = {}
        self._target_tables = {}
        self.routing_stats = dict(route_requests=0,reverse_searches=0,cache_hits=0)

    def segment_at(self, pid, distance):
        for end,sid in self.offsets.get(pid, ()):
            if distance < end-1e-6:
                return sid
        return self.offsets[pid][-1][1]

    def _escape_segment(self, pid, distance, allow_origin):
        # 外部任务不能以“已经在段内”为由进入超标段。
        if allow_origin and distance > 1e-6 and pid in self.offsets:
            sid = self.segment_at(pid,distance)
            if sid in self.blocked_segments:
                return sid
        return None

    def _range_allowed(self, pid, start, end, escape=None):
        lower = 0.0
        for upper,sid in self.offsets.get(pid, ()):
            if min(end,upper)-max(start,lower) > 1e-6:
                if sid in self.blocked_segments and sid != escape:
                    return False
            lower = upper
        return pid in self.positions

    def _source_prefix(self, pipe_id, distance, allow_origin):
        """合法的初始前进节点；超标源段的退出豁免仅属于本次查询。"""
        values = self.positions.get(pipe_id, [])
        k = bisect_left(values, round(distance, 6))
        if k == len(values):
            return []
        escape = self._escape_segment(pipe_id,distance,allow_origin)
        if not self._range_allowed(pipe_id,distance,values[k],escape):
            return []
        # 仅给已在超标段内的任务一个向前离开的前缀；图本身仍禁止重入。
        escape_end = next((end for end,sid in self.offsets[pipe_id] if sid == escape), values[k])
        prefix = []
        for position in values[k:]:
            if position > escape_end+1e-6:
                break
            node = self.index[(pipe_id,position)]
            prefix.append((node,max(0,round(round(position-distance,6)*self.DISTANCE_SCALE)),len(prefix)+1))
        return prefix

    def search(self, pipe_id, distance, rng, destination=None, *, allow_origin=True):
        """整数距离的独立正向Dijkstra参考接口；返回距离单位仍为米。"""
        costs, parents = {}, {}
        queue, previous_node = [],None
        for node,length,hops in self._source_prefix(pipe_id,distance,allow_origin):
            costs[node] = (length,hops)
            parents[node] = previous_node
            previous_node = node
            queue.append((*costs[node],node))
        heapq.heapify(queue)
        goal, bound = None, math.inf
        if destination is not None:
            target_values = self.positions.get(destination.pipe_id, [])
            j = bisect_right(target_values,round(destination.distance_m,6))-1
            if j >= 0:
                goal = self.index[(destination.pipe_id,target_values[j])]
            if (destination.pipe_id == pipe_id and destination.distance_m >= distance
                    and self._range_allowed(pipe_id,distance,destination.distance_m,
                                            self._escape_segment(pipe_id,distance,allow_origin))):
                bound = destination.distance_m-distance
        while queue:
            length, hops, node = heapq.heappop(queue)
            if (length,hops) != costs[node]:
                continue
            if destination is not None and (node == goal or length/self.DISTANCE_SCALE > bound):
                break
            for nxt, weight in self.integer_edges[node]:
                candidate = (length+weight, hops+1)
                previous = costs.get(nxt, (math.inf,math.inf))
                if candidate < previous:
                    costs[nxt], parents[nxt] = candidate, node
                    heapq.heappush(queue, (*candidate,nxt))
                elif candidate == previous and rng.randrange(2):
                    parents[nxt] = node
        return {node:(cost[0]/self.DISTANCE_SCALE,cost[1]) for node,cost in costs.items()}, parents

    def finish(self, pipe_id, distance, destination, search, *, allow_origin=True):
        if pipe_id not in self.positions or destination.pipe_id not in self.positions:
            return None
        costs, parents = search
        values = self.positions.get(destination.pipe_id, [])
        k = bisect_right(values, round(destination.distance_m,6))-1
        best = math.inf
        escape = self._escape_segment(pipe_id,distance,allow_origin)
        if (destination.pipe_id == pipe_id and destination.distance_m >= distance
                and self._range_allowed(pipe_id,distance,destination.distance_m,escape)):
            best = destination.distance_m-distance
        route = [(destination.pipe_id, destination.distance_m)] if math.isfinite(best) else None
        if k >= 0:
            node = self.index[(destination.pipe_id,values[k])]
            # 离段豁免只覆盖当前位置前方的初始前缀，不允许绕一圈从后方重入。
            tail_escape = escape if destination.pipe_id == pipe_id and values[k] >= distance-1e-6 else None
            if node in costs and self._range_allowed(destination.pipe_id,values[k],destination.distance_m,tail_escape):
                length = costs[node][0]+destination.distance_m-values[k]
                if length < best-1e-6:
                    route = [(destination.pipe_id,destination.distance_m)]
                    while node is not None:
                        route.append(self.nodes[node])
                        node = parents[node]
                    route.reverse()
                    if len(route)>1 and route[-1]==route[-2]:
                        route.pop()
        return route

    def _reverse_table(self, goal):
        if goal in self._target_tables:
            self.routing_stats['cache_hits'] += 1
            return self._target_tables[goal]
        self.routing_stats['reverse_searches'] += 1
        distances, hops = [None]*len(self.nodes), [0]*len(self.nodes)
        successors = [()]*len(self.nodes)
        distances[goal] = 0
        queue = [(0,0,goal)]
        while queue:
            length, steps, node = heapq.heappop(queue)
            if distances[node] != length or hops[node] != steps:
                continue
            for previous, weight in self.reverse_edges[node]:
                candidate, count = length+weight, steps+1
                old = distances[previous]
                if old is None or candidate < old or (candidate == old and count < hops[previous]):
                    distances[previous], hops[previous] = candidate, count
                    successors[previous] = (node,)
                    heapq.heappush(queue,(candidate,count,previous))
                elif candidate == old and count == hops[previous] and node not in successors[previous]:
                    successors[previous] += (node,)
        table = distances,hops,successors
        self._target_tables[goal] = table
        return table

    def route(self, pipe_id, distance, destination, rng, *, allow_origin=True):
        """复用到目标节点的距离表，当前位置前缀和随机路线不共享。"""
        self.routing_stats['route_requests'] += 1
        if pipe_id not in self.positions or destination.pipe_id not in self.positions:
            return None
        escape = self._escape_segment(pipe_id,distance,allow_origin)
        direct = math.inf
        if (destination.pipe_id == pipe_id and destination.distance_m >= distance
                and self._range_allowed(pipe_id,distance,destination.distance_m,escape)):
            direct = destination.distance_m-distance
        direct_route = [(destination.pipe_id,destination.distance_m)] if math.isfinite(direct) else None
        if direct == 0:
            return direct_route
        values = self.positions[destination.pipe_id]
        k = bisect_right(values,round(destination.distance_m,6))-1
        if k < 0:
            return direct_route
        tail_escape = escape if destination.pipe_id == pipe_id and values[k] >= distance-1e-6 else None
        if not self._range_allowed(destination.pipe_id,values[k],destination.distance_m,tail_escape):
            return direct_route
        prefix = self._source_prefix(pipe_id,distance,allow_origin)
        if not prefix:
            return direct_route
        goal = self.index[(destination.pipe_id,values[k])]
        distances,hops,successors = self._reverse_table(goal)
        best, candidates = None,[]
        for i,(node,length,steps) in enumerate(prefix):
            if distances[node] is None:
                continue
            cost = length+distances[node],steps+hops[node]
            if best is None or cost < best:
                best,candidates = cost,[i]
            elif cost == best:
                candidates.append(i)
        if best is None or best[0]/self.DISTANCE_SCALE+destination.distance_m-values[k] >= direct-1e-6:
            return direct_route
        selected = candidates[0] if len(candidates)==1 else rng.choice(candidates)
        route = [self.nodes[n] for n,_,_ in prefix[:selected+1]]
        node = prefix[selected][0]
        while node != goal:
            choices = successors[node]
            node = choices[0] if len(choices)==1 else rng.choice(choices)
            route.append(self.nodes[node])
        final = destination.pipe_id,destination.distance_m
        if route[-1] != final:
            route.append(final)
        return route

    def reachable_exits(self, pipe_id):
        """当前入口能到达的外边界出口；仅缓存连通性，不共享最短路。

        缓存属于这个图实例，拓扑或禁入状态变化重建图后自动失效。
        必须到达真正的末端节点；到过某根管道不代表能通过其后方禁入段。
        """
        if pipe_id in self._reachable_exits:
            return self._reachable_exits[pipe_id]
        seen = set()
        start = self.index.get((pipe_id,0.0))
        if start is not None:
            seen.add(start)
            stack = [start]
            while stack:
                for nxt,_ in self.edges[stack.pop()]:
                    if nxt not in seen:
                        seen.add(nxt)
                        stack.append(nxt)
        exits = frozenset(pid for pid,values in self.positions.items()
                          if self.index[(pid,values[-1])] in seen)
        self._reachable_exits[pipe_id] = exits
        return exits


class DroneSimulation:
    """初始可达出口等概率抽样；入口率随速度更新，暂时不可达时排队。"""
    def __init__(self, network: Network, seed=42, adaptive=True, control_interval_s=15,
                 arrival_alpha=0.5, entry_alphas=None, predictor=None):
        if not math.isfinite(control_interval_s) or control_interval_s <= 0:
            raise ValueError('控制间隔必须为有限正数')
        alphas = dict(entry_alphas or {})
        if alphas.keys()-network.pipes.keys():
            raise ValueError('入口管道不存在')
        if any(not math.isfinite(v) or v < 0 for v in [arrival_alpha, *alphas.values()]):
            raise ValueError('入口alpha必须为有限非负数')
        self.network, self.seed = network, seed
        self.initial_pipe_ids = frozenset(pid for pid,p in network.pipes.items() if p.is_open)
        self.arrival_alpha = arrival_alpha
        self.entry_alphas = {pid:alphas.get(pid,arrival_alpha) for pid in self.initial_pipe_ids}
        self.controller = Controller(network) if adaptive else None
        self.control_interval_s = control_interval_s
        self.predictor = predictor
        self.time_s = 0.0
        self.next_control_s = control_interval_s
        self.completed = self.entered = self.generated = 0
        self.travel_time_s = self.distance_flown_m = 0.0
        self.ramp_transfers = 0
        self.expansion_distance_m = 0.0
        self.expansion_users = set()
        self.events, self.arrival_log, self.entry_log, self.exit_log = [], [], [], []
        self.drones = {}
        self.tracks = defaultdict(list)
        self.stations = []
        self.graph = FlightGraph(network, [])
        self._routing_totals = Counter()
        # 起终点按初始连通性固定；运行时按出口共享距离表、逐架重建随机路径。
        self.destination_options = {
            pid: tuple(sorted(self.graph.reachable_exits(pid) & self.initial_pipe_ids))
            for pid in sorted(self.initial_pipe_ids)
        }
        self.blocked_segments = frozenset()
        self.routing_revision = 0
        self.entry_retry_revision = {}
        self.pending_entries = defaultdict(deque)
        self.last_admitted = {}
        self.passage_events = deque()
        self.entries = {}
        self.entry_rng = {pid: random.Random(f'{seed}/arrival/{pid}') for pid in network.pipes}
        self.od_rng = {pid: random.Random(f'{seed}/destination/{pid}') for pid in network.pipes}
        self.offsets = {}
        for pid,p in network.pipes.items():
            end = 0.0
            self.offsets[pid] = []
            for sid in p.segment_ids:
                end += network.segments[sid].length_m
                self.offsets[pid].append((end,sid))
        self._sync_entries()
        self._update_stats()

    @staticmethod
    def sample_interval(rng, rate, minimum_s):
        """Exp(rate)在(minimum_s,∞)上的条件分布，等价于拒绝重抽。"""
        return minimum_s+rng.expovariate(rate)

    def _headway(self, pid):
        return self.network.throughput_model.headway(self.network.pipes[pid].speed_mps)

    def entry_parameters(self, pid):
        """当前入口频率与间距，lambda为截断前指数率，单位架/秒。"""
        speed = self.network.pipes[pid].speed_mps
        headway = self._headway(pid)
        alpha = self.entry_alphas[pid]
        lambda0 = 1.0/headway
        rate = alpha*lambda0
        if not math.isfinite(lambda0) or not math.isfinite(rate):
            raise ValueError('入口派生频率必须为有限数')
        return dict(speed_mps=speed,safety_distance_m=headway*speed,headway_s=headway,
                    lambda0_per_s=lambda0,alpha=alpha,lambda_per_s=rate,
                    mean_rate_per_s=lambda0*(alpha/(1.0+alpha)))

    @property
    def entry_lambdas(self):
        """只有初始入口；读取时按当前速度重新计算，不缓存初始值。"""
        return {pid:self.entry_parameters(pid)['lambda_per_s'] for pid in sorted(self.initial_pipe_ids)}

    def _sync_entries(self):
        for pid in sorted(self.initial_pipe_ids):
            rate = self.entry_parameters(pid)['lambda_per_s']
            if rate == 0 or not self.destination_options[pid]:
                continue
            if pid not in self.entries:
                self.entries[pid] = ArrivalClock(self.time_s,self.time_s,self.entry_rng[pid].expovariate(1.0))

    def _next_entry(self, pid):
        clock = self.entries[pid]
        params = self.entry_parameters(pid)
        rate = params['lambda_per_s']
        if rate == 0:
            return math.inf
        # 累积已消耗的风险量不回滚；新速度只改变此刻以后等待多久。
        return max(clock.clock_s,clock.last_request_s+params['headway_s'])+clock.remaining_hazard/rate

    def _station(self, pid, end, identity):
        p = self.network.pipes[pid]
        return Station(identity,pid,math.dist(p.start,p.end) if end else 0.0,
                       self.network.segments[p.segment_ids[-1 if end else 0]].region_id)

    def _request(self, pid, time_s):
        """仅从该起点的初始可达出口等概率抽样，不受之后的拥堵影响。"""
        rng = self.od_rng[pid]
        target = rng.choice(self.destination_options[pid])
        identity = self.generated
        self.arrival_log.append(dict(id=identity,pipe_id=pid,destination_pipe_id=target,time_s=time_s,
                                     **self.entry_parameters(pid)))
        origin = self._station(pid,False,identity)
        destination = self._station(target,True,identity)
        drone = Drone(identity,origin,destination,pid,0.0,started_s=time_s)
        self.generated += 1
        return drone

    def _admit(self, drone, time_s, route):
        pid = drone.origin.pipe_id
        drone.route = route or []
        drone.waiting = not drone.route
        self.drones[drone.id] = drone
        self.tracks[drone.id].append([time_s,pid,0.0,int(drone.waiting)])
        self.entered += 1
        self.last_admitted[pid] = time_s
        self.entry_log.append(dict(id=drone.id,pipe_id=pid,time_s=time_s,headway_s=self._headway(pid)))

    def _spawn(self, pid, time_s):
        """直接入网的低层接口；正常仿真先排队再检查路径和安全间隔。"""
        drone = self._request(pid,time_s)
        route = self.graph.route(pid,0.0,drone.destination,
                                random.Random(f'{self.seed}/route/{drone.id}/{time_s:.9f}'),allow_origin=False)
        if route is None:
            self.pending_entries[pid].append(drone)
        else:
            self._admit(drone,time_s,route)
        return drone

    def _admit_pending(self, pid, stop):
        pending = self.pending_entries[pid]
        if self.entry_retry_revision.get(pid) == self.routing_revision or not self.network.pipes[pid].is_open:
            return
        while pending:
            drone = pending[0]
            at = max(self.time_s,drone.started_s,
                     self.last_admitted.get(pid,-math.inf)+self._headway(pid))
            if at >= stop-1e-9:
                return
            route = self.graph.route(pid,0.0,drone.destination,
                                    random.Random(f'{self.seed}/route/{drone.id}/{at:.9f}'),allow_origin=False)
            if route is None:
                # 保留FIFO任务与原终点，仅在可通行图变化后重试。
                self.entry_retry_revision[pid] = self.routing_revision
                return
            pending.popleft()
            self._admit(drone,at,route)
            self.passage_events.extend(self._walk(drone,stop-at,at))

    def _route(self, drone):
        route = self.graph.route(drone.pipe_id,drone.distance_m,drone.destination,
                                 random.Random(f'{self.seed}/route/{drone.id}/{self.time_s:.9f}'))
        # 不可达时保留原计划，供需求预测使用；运动仍原地等待。
        if route is not None:
            drone.route, drone.route_index = route, 0
        drone.waiting = route is None

    def position(self, drone) -> Point:
        p = self.network.pipes[drone.pipe_id]
        ratio = drone.distance_m/math.dist(p.start,p.end)
        return tuple(a+(b-a)*ratio for a,b in zip(p.start,p.end))

    def _segment(self, pid, distance):
        for end,sid in self.offsets[pid]:
            if distance < end-1e-6:
                return sid
        return self.offsets[pid][-1][1]

    def counts(self):
        return dict(Counter(self._segment(d.pipe_id,d.distance_m) for d in self.drones.values()))

    def _walk(self, drone, duration, start, forecast=False):
        """遍历实际路径，逐次记录跨区域段、离开匝道或外边界的通过事件。"""
        pid, distance, index = drone.pipe_id, drone.distance_m, drone.route_index
        remaining = duration
        passages = []
        while remaining > 1e-9:
            if index == len(drone.route):
                if not forecast:
                    arrival = start+duration-remaining
                    self.completed += 1
                    self.travel_time_s += arrival-drone.started_s
                    self.tracks[drone.id].append([arrival,pid,distance,0])
                    self.exit_log.append(dict(id=drone.id,pipe_id=pid,time_s=arrival))
                    self.drones.pop(drone.id,None)
                break
            target_pid, target_distance = drone.route[index]
            now = start+duration-remaining
            if target_pid != pid:
                # 管道末端已经通过纵向越界记录；内部匝道计离开源段一次。
                if distance > 1e-6 and not any(abs(distance-end)<1e-6 for end,_ in self.offsets[pid]):
                    passages.append((now,self._segment(pid,distance)))
                if not forecast:
                    self.tracks[drone.id].append([now,pid,distance,0])
                    self.tracks[drone.id].append([now,target_pid,target_distance,0])
                pid,distance,index = target_pid,target_distance,index+1
                if not forecast:
                    self.ramp_transfers += 1
                continue
            speed = self.network.pipes[pid].speed_mps
            length = max(0,target_distance-distance)
            flown = min(length,remaining*speed)
            for end,sid in self.offsets[pid]:
                if distance+1e-7 < end <= distance+flown+1e-7:
                    passages.append((now+(end-distance)/speed,sid))
            distance += flown
            remaining -= flown/speed
            if not forecast:
                self.distance_flown_m += flown
                if pid not in self.initial_pipe_ids and flown > 1e-7:
                    self.expansion_distance_m += flown
                    self.expansion_users.add(drone.id)
            if length-flown <= 1e-7:
                distance = target_distance
                index += 1
            else:
                break
        if not forecast and index == len(drone.route) and drone.id in self.drones:
            arrival = start+duration-remaining
            self.completed += 1
            self.travel_time_s += arrival-drone.started_s
            self.tracks[drone.id].append([arrival,pid,distance,0])
            self.exit_log.append(dict(id=drone.id,pipe_id=pid,time_s=arrival))
            self.drones.pop(drone.id,None)
        if not forecast:
            drone.pipe_id,drone.distance_m,drone.route_index = pid,distance,index
        return passages

    def predict_demand(self):
        """基础预测：当前所有任务按当前速度继续飞，未来窗口内计划离段次数。

        不可达任务沿保留计划预测，避免关匝道后需求凭空消失；可用predictor替换。
        尚未生成的随机任务不预测，因此空网启动期预测偏低。
        """
        if self.predictor is not None:
            return self.predictor(self, self.network.config.throughput_window_s)
        result = Counter()
        for d in self.drones.values():
            result.update(sid for _,sid in self._walk(d,self.network.config.throughput_window_s,self.time_s,True))
        return dict(result)

    def passed(self):
        cutoff = self.time_s-self.network.config.throughput_window_s
        # 不同无人机的事件追加顺序未必按时间排序。
        self.passage_events = deque((t,s) for t,s in self.passage_events if t > cutoff+1e-9)
        return dict(Counter(s for _,s in self.passage_events))

    def _update_stats(self):
        counts, passed = self.counts(), self.passed()
        for sid,s in self.network.segments.items():
            s.count, s.passed = counts.get(sid,0),passed.get(sid,0)

    def refresh_prediction(self):
        """显式刷新用于查看的预测快照；运动和控制无需预测。"""
        demand = self.predict_demand()
        for sid,s in self.network.segments.items():
            s.demand = demand.get(sid,0)
        return demand

    def _control(self):
        self.record_positions()
        self._update_stats()
        events = self.controller.step(self.time_s,self.passed(),counts=self.counts()) if self.controller else []
        self.events.extend(events)
        blocked = frozenset(sid for sid,s in self.network.segments.items()
                            if self.network.pipes[s.pipe_id].is_open
                            and (s.passed >= self.network.segment_throughput(sid)
                                 or (sid in self.blocked_segments and s.passed >
                                     self.network.config.reopen_ratio*self.network.segment_throughput(sid))))
        if blocked != self.blocked_segments or any(e.action in (
                'pairs_expanded','pairs_shrunk','retained_pair_closed','ramps_closed','ramps_reopened') for e in events):
            self.blocked_segments = blocked
            self._routing_totals.update(self.graph.routing_stats)
            self.graph = FlightGraph(self.network,[],blocked)
            self.routing_revision += 1
            for drone in self.drones.values():
                self._route(drone)
        self._sync_entries()

    def step(self, dt_s=1.0):
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError('dt_s必须为有限正数')
        end = self.time_s+dt_s
        while self.time_s < end-1e-9:
            stop = min(end,self.next_control_s)
            self._sync_entries()
            # 同一控制区间速度不变；每架按真实小数到达时刻推进到stop。
            for drone in list(self.drones.values()):
                if not drone.waiting:
                    self.passage_events.extend(self._walk(drone,stop-self.time_s,self.time_s))
            for pid in sorted(self.entries.keys() | self.pending_entries.keys()):
                if pid in self.entries:
                    at = self._next_entry(pid)
                    while at < stop-1e-9:
                        self.pending_entries[pid].append(self._request(pid,at))
                        self.entries[pid] = ArrivalClock(at,at,self.entry_rng[pid].expovariate(1.0))
                        at = self._next_entry(pid)
                    params = self.entry_parameters(pid)
                    self.entries[pid].advance(stop,params['headway_s'],params['lambda_per_s'])
                self._admit_pending(pid,stop)
            self.time_s = stop
            if self.time_s >= self.next_control_s-1e-9:
                self._control()
                self.next_control_s += self.control_interval_s
        self._update_stats()
        return self.metrics()

    def record_positions(self):
        for d in self.drones.values():
            self.tracks[d.id].append([self.time_s,d.pipe_id,d.distance_m,int(d.waiting)])

    def routing_metrics(self):
        """跨图版本累计的请求、反向Dijkstra及表复用次数。"""
        return dict({key:self._routing_totals[key]+value for key,value in self.graph.routing_stats.items()},
                    graph_versions=self.routing_revision+1,target_tables_current=len(self.graph._target_tables))

    def metrics(self):
        net = self.network
        deficits = [max(0,s.passed-net.segment_throughput(sid)) for sid,s in net.segments.items()]
        recent_exits = sum(e['time_s'] > self.time_s-net.config.throughput_window_s+1e-9 for e in self.exit_log)
        return dict(time_s=round(self.time_s,3),population=len(self.drones),entered=self.entered,
                    routing=self.routing_metrics(),
                    requests=len(self.arrival_log),
                    generated=self.generated,outside_waiting=sum(len(q) for q in self.pending_entries.values()),
                    blocked_segments=len(self.blocked_segments),
                    expansion_users=len(self.expansion_users),expansion_distance_km=round(self.expansion_distance_m/1000,2),
                    expansion_passed=sum(s.passed for s in net.segments.values() if s.pipe_id not in self.initial_pipe_ids),
                    completed=self.completed,exit_throughput=recent_exits,
                    passed=sum(s.passed for s in net.segments.values()),
                    speed_changes=sum(e.action=='speed_changed' for e in self.events),
                    speed_recoveries=sum(e.action=='speed_recovered' for e in self.events),
                    waiting=sum(d.waiting for d in self.drones.values()),
                    overloaded_segments=sum(v>1e-8 for v in deficits),
                    excess=round(sum(deficits),2),open_pairs=sum(p.is_open for p in net.pipes.values())//2,
                    mean_trip_s=round(self.travel_time_s/self.completed,1) if self.completed else None,
                    distance_km=round(self.distance_flown_m/1000,2),ramp_transfers=self.ramp_transfers,
                    mean_speed_mps=round(sum(net.pipes[d.pipe_id].speed_mps if not d.waiting else 0
                                            for d in self.drones.values())/max(1,len(self.drones)),2))
