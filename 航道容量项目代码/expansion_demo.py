"""复制结构快照演示，单独展示一个初始间隙的三层二叉扩容。"""
from dataclasses import asdict
import json
from pathlib import Path

from airway import Config, build_network


def main():
    net = build_network(Config())
    net.initialize()
    lid = 'R1-1/L1'
    layout = net.layouts[lid]
    left, right = layout.initial_pair_ids[:2]
    root = next(pid for pid,n in net.split_nodes.items()
                if n.left_pair_id == left and n.right_pair_id == right)
    def in_tree(pid):
        while pid is not None:
            if pid == root:
                return True
            pid = net.split_nodes[pid].parent_pair_id
        return False
    focus = net.pairs[left].pipe_ids[0]+'@R1-1'
    frames, nodes = [], []
    def capture(label, events):
        frames.append(dict(label=label,time=len(nodes),restored=False,
            speeds={pid:p.speed_mps for pid,p in net.pipes.items()},
            open_pipes=sorted(pid for pid,p in net.pipes.items() if p.is_open),
            flows={},throughputs={sid:net.segment_throughput(sid) for sid in net.segments},
            spacings={pid:net.throughput_model.safe_spacing(p.speed_mps) for pid,p in net.pipes.items()},
            layouts={lid:dict(k=l.extra_pairs,selected=l.selected_pair_ids[:],spacing_m=l.spacing_m)
                     for lid,l in net.layouts.items()},
            blocked_targets=[],region_totals={rid:net.region_totals(rid) for rid in net.regions},events=events))
    capture('初始两对',[])
    for step in range(1,8):
        candidates=[pid for pid in net.expansion_candidates(lid) if in_tree(pid)]
        pid=min(candidates,key=lambda p:(net.split_nodes[p].depth,p))
        node=net.split_nodes[pid]
        before=set(frames[-1]['open_pipes'])
        changes=net.apply_layout(lid,step,selected_pair_ids=layout.selected_pair_ids+[pid])
        assert changes['opened']==[pid] and not changes['closed']
        assert before < {pid for pid,p in net.pipes.items() if p.is_open}
        nodes.append(dict(asdict(node),step=step,offset=net.pair_offset(pid)))
        capture(f'第{step}次 · 第{node.depth+1}层',
                [dict(time_s=step,region_id='R1-1',segment_id='',action='pairs_expanded',
                      details=dict(pair_id=pid,depth=node.depth,bounds=[node.left_pair_id,node.right_pair_id],**changes))])
    assert [n['depth'] for n in nodes]==[0,1,1,2,2,2,2]
    assert len(layout.selected_pair_ids)==9
    payload=dict(config=dict(asdict(net.config),pair_spacing_m=net.config.pair_spacing_m),
        comparison_url='network.html',comparison_label='返回完整结构演示',focus=focus,focus_pipe=net.segments[focus].pipe_id,
        layers=[asdict(l) for l in net.layers.values()],
        pipes={pid:dict(layer=p.layer_id,number=p.number,pair=p.pair_id,direction=p.direction) for pid,p in net.pipes.items()},
        segments={sid:dict(pipe=s.pipe_id,region=s.region_id,length_m=s.length_m,a=s.start,b=s.end) for sid,s in net.segments.items()},
        ramps=[],frames=frames,tree=nodes,bounds=[net.pair_offset(left),net.pair_offset(right)])
    out=Path(__file__).with_name('outputs');out.mkdir(exist_ok=True)
    template=Path(__file__).with_name('expansion_viewer.html').read_text(encoding='utf-8')
    (out/'expansion.html').write_text(template.replace('__NETWORK_DATA__',json.dumps(payload,ensure_ascii=False,separators=(',',':'))),encoding='utf-8')
    (out/'expansion_steps.json').write_text(json.dumps(nodes,ensure_ascii=False,indent=2),encoding='utf-8')
    print('Verified: 7 added pairs; depths 0,1,1,2,2,2,2; existing pipes preserved.')
    print(out/'expansion.html')


if __name__=='__main__':main()
