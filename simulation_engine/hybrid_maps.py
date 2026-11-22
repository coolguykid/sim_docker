"""Map facade combining center-DT global routing with one live regional map."""
from __future__ import annotations

import math

from uav_sim.airspace.partition_graph import PartitionMaps


def guidance_transitions(graph):
    """按端点的区域和最近高度层识别可加权的有向边。

    输入
        graph：内部路网对象。
    输出
        边 ID 到源、目标单元的映射。

    原有说明
        Map directed edges whose endpoints belong to different guidance cells.
    """
    layers = sorted(tuple(row) for row in graph.metadata.get("guidance_layers", ()))
    if not layers:
        # Older serialized database graphs retain layer IDs on their boundary records.
        boundaries = graph.metadata.get("boundary_transitions", {})
        layers = sorted({
            (graph.nodes[node_id].position[2], int(item[layer_key]))
            for edge_id, item in boundaries.items() if edge_id in graph.edges
            for node_id, layer_key in (
                (graph.edges[edge_id].source, "source_layer"),
                (graph.edges[edge_id].target, "target_layer"),
            )
        })
    if not layers:
        layers = [(height, index) for index, height in enumerate(
            sorted({node.position[2] for node in graph.nodes.values()}), start=1)]
    cells = {
        node_id: (int(node.region_id), min(
            layers, key=lambda row: abs(float(row[0]) - node.position[2]))[1])
        for node_id, node in graph.nodes.items()
    }
    return {
        edge_id: (cells[edge.source], cells[edge.target])
        for edge_id, edge in graph.edges.items()
        if cells[edge.source] != cells[edge.target]
        and (edge.ramp_id is None or graph.ramp_states.get(
            "R{}:{}".format(edge.region_id, edge.ramp_id),
            graph.ramp_states.get(edge.ramp_id, True)))
    }


def guidance_penalties(graph, transfers, weight_seconds=15.0):
    """对已启用源单元按导引比例计算边的附加寻路代价。

    输入
        graph：内部路网对象。
        transfers：按源和目标区域层定义的导引比例。
        weight_seconds：导引附加代价的权重，单位秒。
    输出
        边 ID 到惩罚秒数的映射。
    """
    if weight_seconds <= 0:
        return {}
    policy = {
        ((int(row["source_region_id"]), int(row["source_layer"])),
         (int(row["target_region_id"]), int(row["target_layer"]))): float(row["proportion"])
        for row in transfers
    }
    guided_sources = {source for source, _ in policy}
    result = {}
    for edge_id, (source, target) in guidance_transitions(graph).items():
        if source not in guided_sources:
            continue
        probability = policy.get((source, target), 0.0)
        if probability < 1.0:
            result[edge_id] = -float(weight_seconds) * math.log(max(probability, 1e-6))
    return result

class HybridPartitionMaps:
    """Exact interface consumed by delivered centralized_regional backend."""

    def __init__(self, global_graph, local_graph, transfers, revision=1,
                 weight_seconds=15.0):
        """从全局图和本地图构造应用同一版导引的寻路视图。

        输入
            global_graph：覆盖参与区域的全局路网。
            local_graph：当前区域的有效路网。
            transfers：按源和目标区域层定义的导引比例。
            revision：用于使寻路代价缓存失效的导引版本。
            weight_seconds：导引附加代价的权重，单位秒。
        输出
            None。
        """
        revision = int(revision)
        if revision <= 0:
            raise ValueError("guidance revision must be positive")
        penalties = guidance_penalties(global_graph, transfers, weight_seconds)
        self.global_maps = PartitionMaps(global_graph).with_edge_penalties(
            penalties, revision)
        local_penalties = {edge_id: value for edge_id, value in penalties.items()
                           if edge_id in local_graph.edges}
        self.local_maps = PartitionMaps(local_graph).with_edge_penalties(
            local_penalties, revision)
        self.graph = global_graph
        self.cost_revision = revision
        self.local_region_id = next(iter(local_graph.regions))

    def region(self, region_id):
        """读取当前服务所负责区域的详细路网。

        输入
            region_id：区域编号。
        输出
            本区详细路网，非本区时抛出 KeyError。
        """
        if int(region_id) != int(self.local_region_id):
            raise KeyError(region_id)
        return self.local_maps.region(region_id)

    def summary(self):
        """生成或复用加权后的全局跨区摘要。

        输入
            无显式参数。
        输出
            PortalSummary。
        """
        return self.global_maps.summary()

    def destination(self, goal_node_id):
        """将目标节点转换成包含跨区入境代价的目的地描述。

        输入
            goal_node_id：目标内部节点 ID。
        输出
            Destination。
        """
        return self.global_maps.destination(goal_node_id)
