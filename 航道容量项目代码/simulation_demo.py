"""生成外边界随机到达的吞吐实验及自动调整单组预览。"""
import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
from time import perf_counter

from airway import Config, build_network
from airway.simulation import DroneSimulation
from throughput_plot import make_throughput_plot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arrival-alpha', type=float, default=0.5,
                        help='入口指数率lambda=alpha*v/安全距离；0停止生成')
    parser.add_argument('--window-s', type=float, default=60)
    parser.add_argument('--seconds', type=int, default=900)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--sample-s', type=int, default=5)
    parser.add_argument('--control-s', type=float, default=15)
    parser.add_argument('--min-speed-mps', type=float, default=10)
    parser.add_argument('--recovery-ratio', type=float, default=.6)
    parser.add_argument('--recovery-step-mps', type=float, default=2)
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    args = parser.parse_args()
    if args.seconds < 1 or args.sample_s < 1:
        parser.error('仿真时长和采样间隔必须为正整数')
    started = perf_counter()
    print('构建网络…', flush=True)
    net = build_network(Config(throughput_window_s=args.window_s,min_speed_mps=args.min_speed_mps,
                               speed_recovery_ratio=args.recovery_ratio,
                               speed_recovery_step_mps=args.recovery_step_mps))
    figure_svg, curve_rows = make_throughput_plot(net.throughput_model, net.config)
    pipe_ids, segment_ids = list(net.pipes), list(net.segments)
    pipe_index = {pid:i for i,pid in enumerate(pipe_ids)}
    segment_index = {sid:i for i,sid in enumerate(segment_ids)}
    group_started = perf_counter()
    net.initialize()
    print('初始化自动调整实验…', flush=True)
    sim = DroneSimulation(net,seed=args.seed,adaptive=True,control_interval_s=args.control_s,
                          arrival_alpha=args.arrival_alpha)
    frames = []
    event_index = 0
    def capture():
        nonlocal event_index
        sim.refresh_prediction()
        sim.record_positions()
        metrics = sim.metrics()
        assert metrics['population'] == sim.entered-sim.completed
        assert metrics['generated'] == sim.entered+metrics['outside_waiting']
        assert metrics['requests'] == metrics['generated']
        counts = sim.counts()
        assert sum(counts.values()) == metrics['population']
        events = [asdict(e) for e in sim.events[event_index:]]
        event_index = len(sim.events)
        frames.append(dict(metrics=metrics,
            drones=[[pipe_index[d.pipe_id], round(d.distance_m,2), d.id, int(d.waiting)]
                    for d in sim.drones.values()],
            loads=[[segment_index[sid],s.passed,net.segment_throughput(sid),s.demand,s.count]
                   for sid,s in net.segments.items()
                   if net.pipes[s.pipe_id].is_open or s.passed or s.demand or s.count],
            open=[pipe_index[p.id] for p in net.pipes.values() if p.is_open],
            speeds=[[pipe_index[p.id],round(p.speed_mps,3)] for p in net.pipes.values() if p.is_open],
            entry_rates=[[pipe_index[pid],params['lambda0_per_s'],params['lambda_per_s'],params['headway_s']]
                         for pid in sorted(sim.initial_pipe_ids) for params in [sim.entry_parameters(pid)]],
            throughputs={segment_index[sid]:net.segment_throughput(sid) for sid,s in net.segments.items()
                        if net.pipes[s.pipe_id].is_open},
            blocked=[segment_index[sid] for sid in sorted(sim.blocked_segments)],
            regions={rid:[net.region_totals(rid)[0],net.region_totals(rid)[1],sum(net.segments[s].demand for s in r.segment_ids)]
                     for rid,r in net.regions.items()},
            events=events))
    capture()
    for t in range(1,args.seconds+1):
        sim.step(1)
        if t % args.sample_s == 0 or t == args.seconds:
            capture()
        if t % 120 == 0:
            print('调整', t, sim.metrics(), flush=True)
    result = dict(frames=frames, final=sim.metrics(),
        runtime_s=round(perf_counter()-group_started,3),
        births={e['id']:[e['time_s'],pipe_index[e['pipe_id']]] for e in sim.entry_log},
        exits={e['id']:[e['time_s'],pipe_index[e['pipe_id']]] for e in sim.exit_log},
        entry_lambdas=sim.entry_lambdas,
        admissions=sim.entry_log,
        arrivals=sim.arrival_log,
        tracks={did:[[round(t,6),pipe_index[p],round(x,4),w] for t,p,x,w in sorted(points,key=lambda v:v[0])]
                for did,points in sim.tracks.items()})
    assert all(e['pipe_id'] in sim.initial_pipe_ids for e in sim.entry_log)
    assert all(e['destination_pipe_id'] in sim.initial_pipe_ids for e in sim.arrival_log)
    assert all(e['destination_pipe_id'] in sim.destination_options[e['pipe_id']] for e in sim.arrival_log)
    payload = dict(config=asdict(net.config), throughput_model=net.throughput_model.metadata(),
                   control_basis='actual', routing_policy='avoid_overloaded_segments',
                   speed_control_policy='integer_throughput_gain',
                   routing_algorithm='reverse_dijkstra_integer_um',
                   external_demand='current_speed_scaled_initial_entries',
                   task_generation='sample_initial_reachable_exits',
                   initial_destination_options=sim.destination_options,
                   experiment_mode='adaptive_only',
                   experiment={k:v for k,v in vars(args).items() if k!='output'},
                   
                   pipes=[dict(id=p.id,a=p.start,b=p.end,layer=p.layer_id,pair=p.pair_id) for p in net.pipes.values()],
                   segments=[dict(id=s.id,pipe=pipe_index[s.pipe_id],region=s.region_id,a=s.start,b=s.end) for s in net.segments.values()],
                   stations=[asdict(s) for s in sim.stations], adaptive=result)
    args.output.mkdir(parents=True,exist_ok=True)
    html=Path(__file__).with_name('simulation_viewer.html').read_text(encoding='utf-8')
    figure_inline = figure_svg[figure_svg.index('<svg'):]
    (args.output/'simulation.html').write_text(html.replace('__THROUGHPUT_FIGURE__',figure_inline)
        .replace('__SIMULATION_DATA__',json.dumps(payload,ensure_ascii=False,separators=(',',':'))),encoding='utf-8')
    (args.output/'speed_throughput.svg').write_text(figure_svg,encoding='utf-8')
    with (args.output/'speed_throughput.csv').open('w',encoding='utf-8',newline='') as curve_file:
        writer=csv.DictWriter(curve_file,fieldnames=list(curve_rows[0]))
        writer.writeheader()
        writer.writerows(curve_rows)
    report=dict(experiment=payload['experiment'], throughput_model=payload['throughput_model'],
                control_basis=payload['control_basis'], runtime_s=round(perf_counter()-started,2),
                speed_control_policy=payload['speed_control_policy'],
                routing_algorithm=payload['routing_algorithm'],
                experiment_mode=payload['experiment_mode'],
                group_runtime_s={'adaptive':result['runtime_s']},
                adaptive=result['final'])
    (args.output/'simulation_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (args.output/'simulation_events.json').write_text(json.dumps([e for f in result['frames'] for e in f['events']],ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False),flush=True)
    print((args.output/'simulation.html').resolve())


if __name__=='__main__':
    main()
