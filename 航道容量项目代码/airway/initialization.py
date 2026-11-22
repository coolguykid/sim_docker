"""以左上角参考区域为锚点，沿每层法向全域等间距初始化。"""
from math import ceil, floor, sin, cos, radians


def initial_targets(network, layer_id):
    layout = network.layouts[f"R{network.config.grid_n}-1/L{layer_id+1}"]
    spacing = network.config.pair_spacing_m
    theta = radians(network.layers[layer_id].angle_deg)
    projection_half = network.config.region_side_m*(abs(sin(theta))+abs(cos(theta)))/2
    anchor = layout.center_offset_m-projection_half+spacing/2
    pairs = [p.id for p in network.pairs.values()
             if network.pipes[p.pipe_ids[0]].layer_id == layer_id]
    offsets = [network.pair_offset(pid) for pid in pairs]
    # 只延续到候选管道对中心的覆盖范围；不将域外目标压到边缘。
    return [anchor+j*spacing for j in range(
        ceil((min(offsets)-anchor)/spacing-1e-10),
        floor((max(offsets)-anchor)/spacing+1e-10)+1)]


def select_initial_pairs(network, layer_id):
    pairs = [p.id for p in network.pairs.values()
             if network.pipes[p.pipe_ids[0]].layer_id == layer_id]
    offsets = {pid: network.pair_offset(pid) for pid in pairs}
    chosen = [min(pairs, key=lambda pid: (round(abs(offsets[pid]-target), 8), pid))
              for target in initial_targets(network, layer_id)]
    if len(set(chosen)) != len(chosen):
        raise ValueError("候选管道密度不足：不同等间距目标吸附到同一管道对")
    return chosen
