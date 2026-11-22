from dataclasses import dataclass, asdict
from math import floor, isfinite
from .model import Network


@dataclass
class Event:
    time_s: float
    region_id: str
    segment_id: str
    action: str
    details: dict


class Controller:
    """每步输入过去窗口的实际通过量；counts用于保护仍有无人机的管道。

    区域按总实际通过量降序，区域内按管道段实际通过量降序；每个段独立检查，
    避免区域总量掩盖局部瓶颈。同一连续管道的速度全局共享。
    """
    def __init__(self, network: Network):
        self.network = network
        self.last_time_s = None
        self.region_order = []
        self.events: list[Event] = []

    def step(self, time_s: float, passed: dict[str, float], *, counts: dict[str, int] | None = None) -> list[Event]:
        net = self.network
        if not isfinite(time_s) or time_s < 0 or (self.last_time_s is not None and time_s <= self.last_time_s):
            raise ValueError("时间必须为有限非负数并严格递增")
        unknown = passed.keys() - net.segments.keys()
        if unknown:
            raise ValueError(f"未知管道段: {sorted(unknown)}")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not isfinite(v) or v < 0 for v in passed.values()):
            raise ValueError("窗口实际通过量必须为有限非负数")
        if counts is not None and (counts.keys()-net.segments.keys() or any(type(v) is not int or v < 0 for v in counts.values())):
            raise ValueError("在网数量必须为非负整数且管道段存在")
        self.last_time_s = time_s
        for sid, segment in net.segments.items():
            segment.passed = passed.get(sid, 0.0)
            if counts is not None:
                segment.count = counts.get(sid, 0)
        events = []
        def emit(sid, action, **details):
            events.append(Event(time_s, net.segments[sid].region_id, sid, action, details))
        self.region_order = sorted(net.regions, key=lambda r: (-net.region_totals(r)[0], r))
        for rid in self.region_order:
            for sid in sorted(net.regions[rid].segment_ids, key=lambda s: (-net.segments[s].passed, s)):
                segment = net.segments[sid]
                pipe = net.pipes[segment.pipe_id]
                throughput = net.segment_throughput(sid)
                if not pipe.is_open:
                    if segment.passed > 0:
                        emit(sid, "closed_pipe_flow", count=segment.passed)
                    continue
                if segment.passed > 0 and segment.passed >= throughput:
                    # 使用整根管道沿途最大的实测通过量；调速不会改写本窗口观测。
                    flow_max = max(net.segments[s].passed for s in pipe.segment_ids)
                    new_speed = net.throughput_model.speed_for_demand(
                        flow_max*1.01, net.config.throughput_window_s,
                        net.config.min_speed_mps, net.config.max_speed_mps)
                    new_throughput = net.throughput_model.throughput(new_speed, net.config.throughput_window_s)
                    old_integer_throughput = floor(throughput+1e-9)
                    new_integer_throughput = floor(new_throughput+1e-9)
                    # 只有窗口内能多支持至少一架时才接受超载调速；统计仍保留连续值。
                    if (abs(new_speed-pipe.speed_mps) > 1e-10 and
                            new_throughput > throughput+1e-9 and
                            new_integer_throughput > old_integer_throughput):
                        old = pipe.speed_mps
                        pipe.speed_mps = new_speed
                        emit(sid, "speed_changed", pipe_id=pipe.id, old_speed=old,
                             new_speed=new_speed, old_throughput=throughput,
                             new_throughput=new_throughput,
                             old_integer_throughput=old_integer_throughput,
                             new_integer_throughput=new_integer_throughput,
                             observed_throughput=flow_max, window_s=net.config.throughput_window_s)
                    throughput = net.segment_throughput(sid)
                    if segment.passed > 0 and segment.passed >= throughput:
                        changed = 0
                        for ramp_id in net.incoming[sid]:
                            ramp = net.ramps[ramp_id]
                            if not ramp.congestion_blocked:
                                ramp.congestion_blocked = True
                                changed += 1
                        if changed:
                            emit(sid, "ramps_closed", ramp_count=changed, count=segment.passed, throughput=throughput)
                        # 关匝道只限制后续汇入，已经发生的实际通过量保持不变。
                        emit(sid, "unresolved", count=segment.passed, throughput=throughput)
            # 前两阶段完成后，逐层判断，不能用其他方向层的通过能力抵消。
            if any(n > 0 and n >= c for layer in net.layers
                   for n, c in [net.region_totals(rid, layer)]):
                self._expand_region(rid, time_s, events)
            else:
                self._shrink_region(rid, time_s, events)
        self._release_retained(time_s, events)
        self._recover_speeds(time_s, events)
        # 所有全局速度变更完成后，再用最终通过能力检查恢复，避免区域处理顺序影响重开。
        for sid, segment in net.segments.items():
            pipe = net.pipes[segment.pipe_id]
            if pipe.is_open and segment.passed <= net.config.reopen_ratio * net.segment_throughput(sid):
                changed = 0
                for ramp_id in net.incoming[sid]:
                    ramp = net.ramps[ramp_id]
                    if ramp.congestion_blocked:
                        ramp.congestion_blocked = False
                        changed += 1
                if changed:
                    emit(sid, "ramps_reopened", ramp_count=changed)
        self.events.extend(events)
        return events

    def _recover_speeds(self, time_s, events):
        """整管空闲时逐步提速；提速后的各段与沿途区域仍须有通过能力余量。"""
        net = self.network
        skip = {e.details['pipe_id'] for e in events if e.action == 'speed_changed'}
        for event in events:
            if event.action == 'pairs_expanded':
                skip.update(pid for pair in event.details['opened'] for pid in net.pairs[pair].pipe_ids)
        for pipe in net.pipes.values():
            old = pipe.speed_mps
            if not pipe.is_open or pipe.id in skip or old >= net.config.initial_speed_mps-1e-9:
                continue
            observed = [net.segments[s] for s in pipe.segment_ids if net.segments[s].passed]
            ratio = net.config.speed_recovery_ratio
            if any(s.passed > ratio*net.segment_throughput(s.id) for s in observed):
                continue
            upper = min(net.config.initial_speed_mps, old+net.config.speed_recovery_step_mps)
            speed = min((net.throughput_model.speed_for_demand(s.passed/ratio,net.config.throughput_window_s,old,upper)
                         for s in observed), default=upper)
            if speed <= old+1e-9:
                continue
            regions = {net.segments[s].region_id for s in pipe.segment_ids}
            previous = {rid:net.region_totals(rid, pipe.layer_id)[1] for rid in regions}
            pipe.speed_mps = speed
            if any(cap < previous[rid] and count > net.config.reopen_ratio*cap
                   for rid in regions for count,cap in [net.region_totals(rid, pipe.layer_id)]):
                pipe.speed_mps = old
                continue
            sid = observed[0].id if observed else pipe.segment_ids[0]
            events.append(Event(time_s,net.segments[sid].region_id,sid,'speed_recovered',
                                dict(pipe_id=pipe.id,old_speed=old,new_speed=speed)))

    def _open_pairs(self):
        net = self.network
        return {pid for pid, pair in net.pairs.items() if net.pipes[pair.pipe_ids[0]].is_open}

    def _throughput_for(self, region_id, opened, new_speed=None, layer_id=None):
        net = self.network
        total = 0
        for sid in net.regions[region_id].segment_ids:
            pipe = net.pipes[net.segments[sid].pipe_id]
            if pipe.pair_id in opened and (layer_id is None or pipe.layer_id == layer_id):
                speed = new_speed if new_speed is not None and not pipe.is_open else pipe.speed_mps
                total += net.throughput_model.throughput(speed, net.config.throughput_window_s)
        return total

    def _compatible(self, proposed, shrinking=False, new_speed=None):
        """跨区开关不能降低其他区域所需服务能力；缩容另留恢复裕量。"""
        net = self.network
        for rid in net.regions:
            for layer in net.layers:
                count, old = net.region_totals(rid, layer)
                new = self._throughput_for(rid, proposed, new_speed, layer)
                if new < old-1e-9:
                    required = count/net.config.reopen_ratio if shrinking else min(count, old)
                    if new < required-1e-9:
                        return False
        return True

    def _expand_region(self, region_id, time_s, events):
        """一次控制内反复细分叶间隙，直到区域各层实际通过量低于能力或候选耗尽。

        每轮开一对不意味着每步只开一对；循环避免深层递归的栈限制。
        """
        net = self.network
        layouts = [l for l in net.layouts.values() if l.region_id == region_id and l.selected_pair_ids]
        opening_speed = net.throughput_model.best_speed(net.config.min_speed_mps, net.config.max_speed_mps)
        while True:
            totals = {layer: net.region_totals(region_id, layer) for layer in net.layers}
            overloaded = {layer for layer, (n, c) in totals.items() if n > 0 and n >= c}
            if not overloaded:
                break
            def deficit(layout):
                n, c = totals[layout.layer_id]
                return n-c
            before = self._open_pairs()
            chosen = None
            candidates = [(layout, pid) for layout in layouts if layout.layer_id in overloaded
                          for pid in net.expansion_candidates(layout.id, before)]
            candidates.sort(key=lambda item: (net.split_nodes[item[1]].depth,
                                              -deficit(item[0]), item[0].layer_id, item[1]))
            for layout, pid in candidates:
                selected = layout.selected_pair_ids+[pid]
                proposed = before | {pid}
                count, old_throughput = totals[layout.layer_id]
                predicted = self._throughput_for(region_id, proposed, opening_speed, layout.layer_id)
                if predicted > old_throughput and self._compatible(proposed, new_speed=opening_speed):
                    chosen = (layout, pid, selected)
                    break
            if chosen is None:
                for layer in sorted(overloaded):
                    count, throughput = totals[layer]
                    events.append(Event(time_s, region_id, "", "expansion_exhausted",
                                        dict(layer_id=layer, count=count, throughput=throughput,
                                             reason="该区域该层没有可提升通过能力的内部管道对方案")))
                break
            layout, pid, selected = chosen
            old_k = layout.extra_pairs
            k = old_k+1
            node = net.split_nodes[pid]
            changes = net.apply_layout(layout.id, k, selected_pair_ids=selected)
            for pair_id in changes["opened"]:
                for pipe_id in net.pairs[pair_id].pipe_ids:
                    net.pipes[pipe_id].speed_mps = opening_speed
            events.append(Event(time_s, region_id, "", "pairs_expanded",
                                dict(layout_id=layout.id, layer_id=layout.layer_id, old_k=old_k, new_k=k,
                                     pair_id=pid, depth=node.depth,
                                     bounds=[node.left_pair_id, node.right_pair_id],
                                     selected=layout.selected_pair_ids[:], **changes,
                                     old_throughput=old_throughput, new_throughput=net.region_totals(region_id, layout.layer_id)[1])))

    def _shrink_region(self, region_id, time_s, events):
        net = self.network
        count, throughput = net.region_totals(region_id)
        if count > net.config.reopen_ratio*throughput:
            return
        before = self._open_pairs()
        candidates = [(layout, pid) for layout in net.layouts.values()
                      if layout.region_id == region_id and layout.extra_pairs > 0
                      and net.region_totals(region_id, layout.layer_id)[0] <= net.config.reopen_ratio*net.region_totals(region_id, layout.layer_id)[1]
                      for pid in layout.selected_pair_ids if pid not in layout.initial_pair_ids
                      and not net.has_open_children(pid, before) and not net.pair_has_count(pid)]
        candidates.sort(key=lambda item: (-net.split_nodes[item[1]].depth,
                                          item[0].layer_id, item[1]))
        for layout, pid in candidates:
            k = layout.extra_pairs-1
            selected = [p for p in layout.selected_pair_ids if p != pid]
            desired = net.planned_open_pairs((layout.id, selected))
            proposed = desired | net._with_ancestors({p for p in before-desired
                                                      if net.pair_has_count(p) or p in net.retained_pair_ids})
            if not self._compatible(proposed, shrinking=True):
                continue
            changes = net.apply_layout(layout.id, k, selected_pair_ids=selected)
            events.append(Event(time_s, region_id, "", "pairs_shrunk",
                                dict(layout_id=layout.id, old_k=k+1, new_k=k,
                                     pair_id=pid, depth=net.split_nodes[pid].depth, **changes,
                                     old_throughput=throughput, new_throughput=net.region_totals(region_id)[1])))
            break  # 每个区域每步最多撤回一个引用，跨区共享的对在最后一个引用释放后关闭。

    def _release_retained(self, time_s, events):
        net = self.network
        for pid in sorted(net.retained_pair_ids, key=lambda p: (-net.split_nodes[p].depth, p)):
            proposed = self._open_pairs()-{pid}
            if (not net.pair_has_count(pid) and not net.has_open_children(pid)
                    and self._compatible(proposed, shrinking=True)):
                net.set_pair_open(pid, False)
                events.append(Event(time_s, "", "", "retained_pair_closed", dict(pair_id=pid)))

    def event_dicts(self):
        return [asdict(event) for event in self.events]
