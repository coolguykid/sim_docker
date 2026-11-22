"""运行 python demo.py，生成可离线打开的交互可视化及控制日志。"""
import argparse
import gzip
import json
from math import isfinite
from dataclasses import asdict
from pathlib import Path
from airway import Config, build_network, Controller


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--boundary-km", type=float, default=12)
    parser.add_argument("--grid-n", type=int, default=4)
    parser.add_argument("--step-s", type=float, default=60)
    parser.add_argument("--reopen-ratio", type=float, default=0.8)
    parser.add_argument("--all-closed", action="store_true", help="覆盖默认等间距方案，初始全关闭")
    parser.add_argument("--export-network", action="store_true", help="额外输出完整 network.json.gz")
    parser.add_argument("--output", type=Path, default=Path("outputs"))
    parser.add_argument("--pair-spacing-m", type=float, default=None)
    parser.add_argument("--reverse-spacing-m", type=float, default=1000)
    parser.add_argument("--comparison-child", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    cfg = Config(boundary_m=args.boundary_km*1000, grid_n=args.grid_n,
                 reopen_ratio=args.reopen_ratio, initial_pair_spacing_m=args.pair_spacing_m,
                 reverse_ramp_spacing_m=args.reverse_spacing_m)
    if args.step_s <= 0 or not isfinite(args.step_s):
        parser.error("--step-s 必须为有限正数")
    print("正在生成连续管道与匝道…", flush=True)
    net = build_network(cfg)
    assert not any(p.is_open for p in net.pipes.values())
    net.initialize([] if args.all_closed else None)
    initial_open_pipes = {pid for pid, p in net.pipes.items() if p.is_open}
    initial_audit = {}
    for lid, layout in net.layouts.items():
        actual = [net.segments[s].pipe_id for s in net.regions[layout.region_id].segment_ids
                  if net.segments[s].pipe_id in initial_open_pipes
                  and net.pipes[net.segments[s].pipe_id].layer_id == layout.layer_id]
        pairs = sorted({net.pipes[pid].pair_id for pid in actual})
        initial_audit[lid] = dict(pair_spacing_m=layout.spacing_m,
                                 actual_pipes=len(actual),actual_pairs=len(pairs), pair_ids=pairs,
                                 target_offsets_m=[layout.center_offset_m-layout.spacing_m/2,
                                                   layout.center_offset_m+layout.spacing_m/2],
                                 actual_offsets_m=[net.pair_offset(p) for p in pairs])
        if not args.all_closed:
            assert set(pairs)==set(layout.initial_pair_ids), f"初始管道引用不一致：{lid}"
    ctrl = Controller(net)
    region_id = f"R{max(1, cfg.grid_n//2)}-{max(1, cfg.grid_n//2)}"
    candidates = [s for s in net.regions[region_id].segment_ids
                  if net.pipes[net.segments[s].pipe_id].is_open and net.incoming[s]]
    focus = candidates[len(candidates)//2] if candidates else next(iter(net.segments))
    focus_pipe = net.segments[focus].pipe_id
    initial_throughputs = {sid: net.segment_throughput(sid) for sid in net.segments}
    max_throughputs = {sid: net.throughput_model.throughput(net.throughput_model.best_speed(cfg.min_speed_mps,cfg.max_speed_mps),cfg.throughput_window_s)
                      for sid, seg in net.segments.items()}
    frames = []
    # 外部给定过去窗口实际通过量（合成示例）；没有单机运动、自动转移或时间传播。
    scenarios = [("初始等间距布局", max(1, initial_throughputs[focus]//2)),
                 ("触发调速", initial_throughputs[focus]),
                 ("局部满载 · 关匝道", max_throughputs[focus]+1),
                 ("区域层实测超标 · 整对扩容", max_throughputs[focus]+1)]
    # 让真实控制器逐步撤回所有扩容管道，而不是截断在两次缩容之后。
    recovery_limit = 2+sum(len(l.interior_pair_ids) for l in net.layouts.values())
    for i in range(len(scenarios)+recovery_limit):
        label, count = scenarios[i] if i < len(scenarios) else (f"逐步恢复 {i-3}", int(max_throughputs[focus]*cfg.reopen_ratio*0.5))
        flows = {sid: int(initial_throughputs[sid]*0.35) for sid, seg in net.segments.items() if seg.pipe_id in initial_open_pipes}
        if i == 3:
            for sid in net.regions[region_id].segment_ids:
                if net.segments[sid].pipe_id in initial_open_pipes:
                    flows[sid] = max_throughputs[sid]+max(1,max_throughputs[sid]//4)
        flows[focus] = count if candidates else 0
        events = ctrl.step(i*args.step_s, flows)
        current_open = {pid for pid,p in net.pipes.items() if p.is_open}
        restored = (current_open == initial_open_pipes and not net.retained_pair_ids
                    and all(l.extra_pairs == 0 for l in net.layouts.values()))
        if i >= len(scenarios) and restored:
            label = "已恢复初始管道布局"
        frames.append(dict(label=label, time=i*args.step_s,
                           restored=restored,
                           focus_flow=flows[focus],
                           speeds={pid: p.speed_mps for pid, p in net.pipes.items()},
                           open_pipes=sorted(pid for pid,p in net.pipes.items() if p.is_open),
                           flows={sid:s.passed for sid,s in net.segments.items() if s.passed},
                           throughputs={sid:net.segment_throughput(sid) for sid in net.segments},
                           spacings={pid:net.throughput_model.safe_spacing(p.speed_mps) for pid,p in net.pipes.items()},
                           layouts={lid:dict(k=l.extra_pairs,selected=l.selected_pair_ids[:],
                                            spacing_m=l.spacing_m) for lid,l in net.layouts.items()},
                           blocked_targets=sorted({r.target_segment for r in net.ramps.values() if r.congestion_blocked}),
                           region_totals={rid: net.region_totals(rid) for rid in net.regions},
                           order=ctrl.region_order[:], events=[asdict(e) for e in events]))
        if i >= len(scenarios):
            if restored:
                break
            if not any(e.action in ("pairs_shrunk", "retained_pair_closed") for e in events):
                raise RuntimeError("恢复示例未能继续缩容，不能标记为已恢复初始布局")
    else:
        raise RuntimeError("恢复示例超过步数上限")
    frame_open_sets = [set(f["open_pipes"]) for f in frames]
    # 紧凑展示格式；完整数据通过 --export-network 导出。
    payload = dict(config=dict(asdict(cfg), pair_spacing_m=cfg.pair_spacing_m),
                   comparison_url="../network.html" if args.comparison_child else "dense/network.html",
                   comparison_label="返回默认间距" if args.comparison_child else "查看较密布局（0°每格三对）", focus=focus, focus_pipe=focus_pipe,
                   layers=[asdict(l) for l in net.layers.values()],
                   pipes={pid: dict(layer=p.layer_id, number=p.number, pair=p.pair_id,
                                    direction=p.direction) for pid, p in net.pipes.items()},
                   segments={sid: dict(pipe=s.pipe_id, region=s.region_id,
                                       length_m=s.length_m, a=[round(x, 3) for x in s.start], b=[round(x, 3) for x in s.end]) for sid, s in net.segments.items()},
                   # 显示任一示例时刻可涉及的候选匝道，页面按当前帧开关过滤。
                   ramps=[dict(source=r.source_segment, target=r.target_segment,
                               station=r.id.split("#")[-1] if "#" in r.id else None,
                               points=[[round(x, 3) for x in p] for p in (r.start, r.end)], kind=r.kind)
                          for r in net.ramps.values()
                          if any(net.segments[r.source_segment].pipe_id in opened
                                 and net.segments[r.target_segment].pipe_id in opened for opened in frame_open_sets)],
                   frames=frames,
                   summary=dict(pipes=len(net.pipes), pairs=len(net.pairs), segments=len(net.segments),
                                ramps=len(net.ramps), continuations=len(net.continuations),
                                initial_open_pipes=len(initial_open_pipes),
                                open_pipes=sum(p.is_open for p in net.pipes.values())))
    args.output.mkdir(parents=True, exist_ok=True)
    template = Path(__file__).with_name("viewer.html").read_text(encoding="utf-8")
    (args.output / "network.html").write_text(template.replace("__NETWORK_DATA__", json.dumps(payload, ensure_ascii=False, separators=(",", ":"))), encoding="utf-8")
    (args.output / "events.json").write_text(json.dumps(ctrl.event_dicts(), ensure_ascii=False, indent=2), encoding="utf-8")
    (args.output / "initial_layout_audit.json").write_text(json.dumps(initial_audit,ensure_ascii=False,indent=2),encoding="utf-8")
    (args.output / "summary.json").write_text(json.dumps(dict(config=asdict(cfg), **payload["summary"],
        per_region_layer_counts={rid: {str(l.angle_deg): sum(net.pipes[net.segments[sid].pipe_id].layer_id == l.id for sid in r.segment_ids)
                                      for l in net.layers.values()} for rid, r in net.regions.items()}), ensure_ascii=False, indent=2), encoding="utf-8")
    if args.export_network:
        with gzip.open(args.output / "network.json.gz", "wt", encoding="utf-8") as f:
            json.dump(net.to_dict(), f, ensure_ascii=False)
    print(json.dumps(payload["summary"], ensure_ascii=False))
    print(f"可视化：{(args.output / 'network.html').resolve()}")
    if not args.comparison_child:
        child=["--boundary-km",str(args.boundary_km),"--grid-n",str(args.grid_n),
               "--step-s",str(args.step_s),"--reopen-ratio",str(args.reopen_ratio),
               "--pair-spacing-m",str(cfg.region_side_m/3),
               "--reverse-spacing-m",str(args.reverse_spacing_m),
               "--output",str(args.output/"dense"),"--comparison-child"]
        if args.export_network: child.append("--export-network")
        if args.all_closed: child.append("--all-closed")
        main(child)


if __name__ == "__main__":
    main()
