"""规则平行管道裁剪至边界后按网格切段；不模拟单机轨迹。"""
from math import cos, sin, radians, hypot, floor
from .model import (Config, ThroughputModel, Layer, Pipe, PipePair, Segment,
                    Region, Ramp, Network)


def _clip(offset, dx, dy, half):
    # p = offset * normal + t * direction
    px, py = -dy * offset, dx * offset
    lo, hi = -float("inf"), float("inf")
    for p, d in ((px, dx), (py, dy)):
        if abs(d) < 1e-10:
            if abs(p) >= half:
                return None
        else:
            a, b = sorted(((-half-p)/d, (half-p)/d))
            lo, hi = max(lo, a), min(hi, b)
    return (px, py, lo, hi) if hi-lo > 1e-6 else None


def _mid(segment):
    return tuple((a+b)/2 for a, b in zip(segment.start, segment.end))


def _vertical_endpoints(source, target):
    """只有俯视投影在两个段内相交，才允许设置竖直层间匝道。"""
    ax, ay, az = source.start
    bx, by, bz = target.start
    ux, uy = source.end[0]-ax, source.end[1]-ay
    vx, vy = target.end[0]-bx, target.end[1]-by
    cross = ux*vy-uy*vx
    if abs(cross) < 1e-9:
        return None
    t = ((bx-ax)*vy-(by-ay)*vx)/cross
    u = ((bx-ax)*uy-(by-ay)*ux)/cross
    if not (0 <= t <= 1 and 0 <= u <= 1):
        return None
    x, y = ax+t*ux, ay+t*uy
    return (x, y, az), (x, y, bz)


def build_network(config: Config | None = None, throughput_model: ThroughputModel | None = None) -> Network:
    """返回全关闭的网络。调用 network.initialize() 才开放初始管道对。"""
    cfg = config or Config()
    net = Network(cfg, throughput_model or ThroughputModel())
    half, cell = cfg.boundary_m / 2, cfg.boundary_m / cfg.grid_n
    for row in range(cfg.grid_n):
        for col in range(cfg.grid_n):
            rid = f"R{row+1}-{col+1}"
            net.regions[rid] = Region(rid, row, col)
    for layer_id in range(6):
        angle = layer_id * 30
        dx, dy = cos(radians(angle)), sin(radians(angle))
        z = cfg.bottom_m + cfg.diameter_m/2 + layer_id*(cfg.diameter_m+cfg.vertical_clearance_m)
        net.layers[layer_id] = Layer(layer_id, angle, z)
        width = cfg.boundary_m * (abs(dx)+abs(dy))
        count = int(width / cfg.pitch_m)
        count -= count % 2
        for index in range(count):
            number = index+1
            offset = (index-(count-1)/2)*cfg.pitch_m
            clipped = _clip(offset, dx, dy, half)
            if clipped is None:
                raise ValueError("管道未穿过边界")
            px, py, lo, hi = clipped
            def point(t):
                return (px+t*dx, py+t*dy, z)
            ts = [lo, hi]
            for k in range(1, cfg.grid_n):
                boundary = -half + k*cell
                for p, d in ((px, dx), (py, dy)):
                    if abs(d) > 1e-10:
                        t = (boundary-p)/d
                        if lo+1e-7 < t < hi-1e-7:
                            ts.append(t)
            ts = sorted(set(round(t, 8) for t in ts))
            intervals = list(zip(ts, ts[1:]))
            direction = 1 if number % 2 else -1
            if direction < 0:
                intervals = [(b, a) for a, b in reversed(intervals)]
            pid, pair_id = f"L{layer_id+1}-P{number:04}", f"L{layer_id+1}-G{index//2+1:04}"
            pipe = Pipe(pid, layer_id, number, pair_id, direction,
                        point(lo if direction == 1 else hi), point(hi if direction == 1 else lo), cfg.initial_speed_mps)
            net.pipes[pid] = pipe
            for a, b in intervals:
                mid = point((a+b)/2)
                col = max(0, min(cfg.grid_n-1, int((mid[0]+half)/cell)))
                row = max(0, min(cfg.grid_n-1, int((mid[1]+half)/cell)))
                rid = f"R{row+1}-{col+1}"
                sid = f"{pid}@{rid}"
                net.segments[sid] = Segment(sid, pid, rid, point(a), point(b), abs(b-a))
                pipe.segment_ids.append(sid)
                net.regions[rid].segment_ids.append(sid)
                net.incoming[sid] = []
            net.continuations.extend(zip(pipe.segment_ids, pipe.segment_ids[1:]))
            if number % 2 == 0:
                net.pairs[pair_id] = PipePair(pair_id, (f"L{layer_id+1}-P{number-1:04}", pid))

    midpoints = {sid: _mid(segment) for sid, segment in net.segments.items()}

    def add_ramp(source, target, kind, endpoints=None, station=None):
        ramp_id = f"{source.id}>{target.id}" + (f"#U{station:04d}" if station is not None else "")
        if ramp_id in net.ramps:
            return
        if kind == "interlayer":
            endpoints = _vertical_endpoints(source, target)
            if endpoints is None:
                return
            a, b = endpoints
        else:
            a, b = endpoints
        net.ramps[ramp_id] = Ramp(ramp_id, source.id, target.id, kind, a, b)
        net.incoming[target.id].append(ramp_id)

    # 每个区内，各段连接其他五层正反向最近的两根候选管道，
    # 另连接同层配对反向管道；默认开放集合与最近候选未必重合，
    # 因此按开放编号规则补充候选，使初始跨层接续可用。
    net.prepare_layouts()
    selected = {pid for layout in net.layouts.values() for pid in layout.initial_pair_ids}
    for region in net.regions.values():
        by_layer_dir = {}
        for sid in region.segment_ids:
            seg = net.segments[sid]
            pipe = net.pipes[seg.pipe_id]
            by_layer_dir.setdefault((pipe.layer_id, pipe.direction), []).append(seg)
        for sid in region.segment_ids:
            source = net.segments[sid]
            pipe = net.pipes[source.pipe_id]
            a = midpoints[source.id]
            for (target_layer, direction), candidates in by_layer_dir.items():
                if target_layer == pipe.layer_id:
                    continue
                intersecting = [s for s in candidates if _vertical_endpoints(source, s) is not None]
                ranked = sorted(intersecting, key=lambda s: hypot(midpoints[s.id][0]-a[0], midpoints[s.id][1]-a[1]))
                for target in ranked[:2]:
                    add_ramp(source, target, "interlayer")
                active_candidate = next((s for s in ranked if net.pipes[s.pipe_id].pair_id in selected), None)
                if active_candidate:
                    add_ramp(source, active_candidate, "interlayer")
    # 沿一对管道的共同纵向覆盖区布点，间隔固定，不在区域边界重置。
    # 正向入口为零点，首站取半个间距，避免把连接直接放在全域入口。
    for pair in net.pairs.values():
        p, q = (net.pipes[pid] for pid in pair.pipe_ids)
        theta = radians(net.layers[p.layer_id].angle_deg)
        dx, dy = cos(theta), sin(theta)
        along = lambda point: point[0]*dx+point[1]*dy
        lo=max(min(along(p.start),along(p.end)),min(along(q.start),along(q.end)))
        hi=min(max(along(p.start),along(p.end)),max(along(q.start),along(q.end)))
        spacing=cfg.reverse_ramp_spacing_m
        def endpoint(pipe,t):
            offset=-dy*pipe.start[0]+dx*pipe.start[1]
            return (-dy*offset+dx*t,dx*offset+dy*t,pipe.start[2])
        def segment_at(pipe,point):
            col=max(0,min(cfg.grid_n-1,int((point[0]+half)/cell)))
            row=max(0,min(cfg.grid_n-1,int((point[1]+half)/cell)))
            return net.segments[f"{pipe.id}@R{row+1}-{col+1}"]
        for i in range(max(0, floor((hi-lo-spacing/2)/spacing)+1)):
            t=lo+spacing/2+i*spacing
            if t>=hi-1e-7: continue
            a,b=endpoint(p,t),endpoint(q,t)
            sa,sb=segment_at(p,a),segment_at(q,b)
            add_ramp(sa,sb,"reverse",(a,b),i+1)
            add_ramp(sb,sa,"reverse",(b,a),i+1)
    return net
