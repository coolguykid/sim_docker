"""Adapters between regional database DTOs and the delivered partition model."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

from uav_sim.airspace.partition_contracts import PartitionGraph, RoadEdge, RoadNode


def regional_node_id(external_id, region_id):
    """给数据库节点 ID 加上区域命名空间。

    输入
        external_id：数据库节点编号。
        region_id：区域编号。
    输出
        内部节点 ID 字符串。
    """
    return "N:{}:R{}".format(int(external_id), int(region_id))


def pipe_edge_id(region_id, pipe_id):
    """把数据库航道编号转成带区域前缀的图边 ID。

    输入
        region_id：区域编号。
        pipe_id：数据库航道编号。
    输出
        航道图边 ID。
    """
    return "P:R{}:{}".format(int(region_id), int(pipe_id))


def ramp_edge_id(region_id, ramp_id):
    """把数据库坡道编号转成带区域前缀的图边 ID。

    输入
        region_id：区域编号。
        ramp_id：数据库坡道编号。
    输出
        坡道图边 ID。
    """
    return "R:R{}:{}".format(int(region_id), int(ramp_id))


@dataclass(frozen=True, order=True)
class EdgeRef:
    edge_type: str
    edge_id: int


class StatisticsEdge:
    def __init__(self, road_edge, capacity=50):
        """将路网边的几何、限速和容量封装为统计对象。

        输入
            road_edge：内部路网边。
            capacity：道路边容量。
        输出
            None。
        """
        self.length = float(road_edge.length_m)
        self.num_lanes = int(road_edge.lane_count)
        self.max_speed = float(road_edge.speed_limit_mps)
        self.max_capacity = int(capacity or 50)


class StatisticsEnvironment:
    def __init__(self, edges):
        """保存供统计器查询的道路边映射。

        输入
            edges：待索引的道路边集合。
        输出
            None。
        """
        self._edges = dict(edges)
        self.edges = tuple(sorted(self._edges))

    def edge(self, ref):
        """按数据库边引用查找统计边。

        输入
            ref：数据库边引用。
        输出
            匹配的统计边。
        """
        return self._edges[ref]


def _point(value):
    """将坐标分量转换为浮点数。

    输入
        value：待转换的三维坐标序列。
    输出
        浮点坐标元组或列表，以所在模块约定为准。
    """
    return tuple(float(part) for part in value)


def _lane_offsets(lanes, start, end, lane_count):
    """根据车道端点和航道中心线计算车道偏移。

    输入
        lanes：参与偏移计算的车道几何记录。
        start：起点坐标或帧开始时刻，依所在函数而定。
        end：终点坐标或帧结束时刻，依所在函数而定。
        lane_count：使用的车道数。
    输出
        起点和终点偏移元组。
    """
    ordered = sorted(lanes, key=lambda row: int(row.get("lane_id", 0)))
    starts = [tuple(a-b for a, b in zip(_point(row["start_pt"]), start)) for row in ordered]
    ends = [tuple(a-b for a, b in zip(_point(row["end_pt"]), end)) for row in ordered]
    while len(starts) < lane_count:
        starts.append((0., 0., 0.))
        ends.append((0., 0., 0.))
    return tuple(starts[:lane_count]), tuple(ends[:lane_count])


def edge_ref(edge):
    """Return the database facility associated with a movement edge."""
    if edge.kind == "FLY" and edge.pipe_id is not None:
        return EdgeRef("pipe", edge.pipe_id)
    if edge.kind == "RAMP" and edge.ramp_id is not None:
        return EdgeRef("ramp", edge.ramp_id)
    raise KeyError(edge.edge_id)


def _add_outbound_boundaries(region_id, raw_nodes, nodes, edges):
    """Build directed exits from the region's own database node rows."""
    transitions = {}
    for external_id, row in raw_nodes.items():
        if int(row.get("boundary_type", row.get("boundary_role", 0))) != 2:
            continue
        memberships = {int(value) for value in row["region_ids"]}
        if region_id not in memberships or len(memberships) != 2:
            continue
        target_region, = memberships - {region_id}
        layer = int(row["layer_id"])
        source = regional_node_id(external_id, region_id)
        target = regional_node_id(external_id, target_region)
        point = nodes[source].position
        nodes[target] = RoadNode(target, target_region, point)
        name = "B:N{}:R{}L{}:R{}L{}".format(
            external_id, region_id, layer, target_region, layer)
        edges[name] = RoadEdge(name, source, target, region_id, "BOUNDARY",
                               0.0, 15.0, (point, point), lane_count=1)
        transitions[name] = {
            "shared_node_id": external_id,
            "source_region": region_id, "target_region": target_region,
            "source_layer": layer, "target_layer": layer,
        }
    return transitions


def build_region_topology(region_id, version, network, include_lanes=True):
    """将有效区域数据库拓扑建成导引版区域图和索引。

    输入
        region_id：区域编号。
        version：路网拓扑版本号。
        network：区域网络及设施记录。
        include_lanes：是否计算并保留车道偏移。
    输出
        原生图、统计环境、节点数据库 ID 映射和边层号映射。

    原有说明
        Build one current regional physical graph from its database snapshot.
    """
    region_id = int(region_id)
    raw_nodes = {int(row["node_id"]): dict(row) for row in network["nodes"]}
    nodes, external_ids = {}, {}
    for external_id, row in raw_nodes.items():
        name = regional_node_id(external_id, region_id)
        nodes[name] = RoadNode(name, region_id, _point(row["position"]))
        external_ids[name] = external_id
    lanes_by_pipe = {}
    for row in network.get("lanes", ()):
        lanes_by_pipe.setdefault(int(row["pipe_id"]), []).append(dict(row))
    edges, stats, layers = {}, {}, {}
    for row in network["pipes"]:
        if not bool(row.get("is_open", True)):
            continue
        pid = int(row["pipe_id"])
        source = regional_node_id(row["start_node_id"], region_id)
        target = regional_node_id(row["end_node_id"], region_id)
        start, end = nodes[source].position, nodes[target].position
        count = max(1, int(row.get("lane_number", 1)))
        offsets = (_lane_offsets(lanes_by_pipe.get(pid, ()), start, end, count)
                   if include_lanes else ((), ()))
        name = pipe_edge_id(region_id, pid)
        edge = RoadEdge(name, source, target, region_id, "FLY",
                        float(row.get("length") or math.dist(start, end)),
                        float(row.get("speed_limit") or 15.), (start, end),
                        pipe_id=pid, lane_count=count,
                        offsets_start=offsets[0], offsets_end=offsets[1])
        edges[name] = edge
        ref = EdgeRef("pipe", pid)
        stats[ref] = StatisticsEdge(edge, row.get("max_capacity", 50))
        layers[name] = int(row.get("layer_id") or 0)
    for row in network["ramps"]:
        if not bool(row.get("is_open", True)):
            continue
        rid = int(row["ramp_id"])
        source = regional_node_id(row["start_node_id"], region_id)
        target = regional_node_id(row["end_node_id"], region_id)
        start, end = nodes[source].position, nodes[target].position
        name = ramp_edge_id(region_id, rid)
        edge = RoadEdge(name, source, target, region_id, "RAMP",
                        float(row.get("length") or math.dist(start, end)), 15.,
                        tuple(_point(p) for p in (row.get("ramp_path") or (start, end))),
                        ramp_id=rid, lane_count=max(1, int(row.get("lane_number", 1))))
        edges[name] = edge
        ref = EdgeRef("ramp", rid)
        stats[ref] = StatisticsEdge(edge)
        layers[name] = int(raw_nodes[int(row["start_node_id"])].get("layer_id") or 0)
    transitions = _add_outbound_boundaries(region_id, raw_nodes, nodes, edges)
    graph = PartitionGraph(nodes, edges, {region_id: {"region_id": region_id}},
                           {"database_adapter": "region-dt-topology-v2",
                            "boundary_transitions": transitions}, int(version),
                           {int(row["ramp_id"]): bool(row.get("is_open", True))
                            for row in network["ramps"]})
    return graph, StatisticsEnvironment(stats), external_ids, layers


def build_global_partition_graph(networks, version):
    """汇总各区域拓扑及目标节点存在于汇总范围内的出境边。

    输入
        networks：按区域 ID 索引的区域网络集合。
        version：路网拓扑版本号。
    输出
        全局 PartitionGraph。

    原有说明
        Compile the temporary center-DT global routing graph from all regions.
    """
    topologies = {int(rid): build_region_topology(rid, version, network, False)
                  for rid, network in networks.items()}
    nodes, edges, regions, ramp_states, transitions = {}, {}, {}, {}, {}
    for rid, (graph, _, external_ids, _) in topologies.items():
        nodes.update((name, graph.nodes[name]) for name in external_ids)
        regions[rid] = {"region_id": rid}
        ramp_states.update({"R{}:{}".format(rid, key): value
                            for key, value in graph.ramp_states.items()})
    for graph, _, _, _ in topologies.values():
        for edge in graph.edges.values():
            if edge.target not in nodes:
                continue
            edges[edge.edge_id] = edge
            if edge.kind == "BOUNDARY":
                transitions[edge.edge_id] = graph.metadata["boundary_transitions"][edge.edge_id]
    guidance_layers = {}
    for network in networks.values():
        layer_rows = network.get("layers") or [
            {"layer_id": row["layer_id"], "height": row["position"][2]}
            for row in network["nodes"] if row.get("layer_id") is not None
        ]
        for row in layer_rows:
            height, layer_id = float(row["height"]), int(row["layer_id"])
            if guidance_layers.setdefault(height, layer_id) != layer_id:
                raise ValueError("inconsistent database layer IDs at height {}".format(height))
    return PartitionGraph(nodes, edges, regions,
                          {"database_adapter": "center-dt-test-v1",
                           "boundary_transitions": transitions,
                           "guidance_layers": sorted(guidance_layers.items())},
                          int(version), ramp_states)


def overlay_local_region(global_graph, local_graph, region_id, node_external_ids):
    """将当前尝试的本地有效路网覆盖进全局图。

    输入
        global_graph：覆盖参与区域的全局路网。
        local_graph：当前区域有效图。
        region_id：当前区域编号。
        node_external_ids：实际数据库节点的索引。
    输出
        更新后的全局图。

    """
    obsolete = [
        edge_id
        for edge_id, edge in global_graph.edges.items()
        if int(edge.region_id) == region_id
    ]
    transitions = global_graph.metadata.setdefault("boundary_transitions", {})
    for edge_id in obsolete:
        del global_graph.edges[edge_id]
        transitions.pop(edge_id, None)
    global_graph.ramp_states.update({
        f"R{region_id}:{ramp_id}": enabled
        for ramp_id, enabled in local_graph.ramp_states.items()
    })
    global_graph.nodes.update((name, local_graph.nodes[name])
                              for name in node_external_ids)
    for edge in local_graph.edges.values():
        if edge.target not in global_graph.nodes:
            continue
        global_graph.edges[edge.edge_id] = edge
        if edge.kind == "BOUNDARY":
            transitions[edge.edge_id] = local_graph.metadata["boundary_transitions"][edge.edge_id]
    # Native PartitionMaps reads numeric ramp IDs. Apply the regional state
    # keys here, where identical ramp IDs in different regions are distinct.
    closed_ramps = [
        edge.edge_id for edge in global_graph.edges.values()
        if edge.ramp_id is not None
        and not global_graph.ramp_states.get(f"R{edge.region_id}:{edge.ramp_id}", True)
    ]
    for edge_id in closed_ramps:
        del global_graph.edges[edge_id]
        transitions.pop(edge_id, None)
    return global_graph


def graph_to_dict(graph):
    """把内部路网打包成可以放入 JSON 请求的结构。

    输入
        graph：内部路网对象。
    输出
        序列化路网字典。
    """
    return {
        "version": graph.version,
        "regions": graph.regions,
        "nodes": [asdict(node) for node in graph.nodes.values()],
        "edges": [asdict(edge) for edge in graph.edges.values()],
        "metadata": graph.metadata,
        "ramp_states": graph.ramp_states,
    }


def graph_from_dict(value):
    """从请求中的序列化路网恢复节点、边与元数据。

    输入
        value：序列化路网字典。
    输出
        PartitionGraph。
    """
    nodes = {row["node_id"]: RoadNode(**row) for row in value["nodes"]}
    edges = {}
    for raw in value["edges"]:
        row = dict(raw)
        for name in ("points", "offsets_start", "offsets_end"):
            row[name] = tuple(tuple(float(v) for v in point) for point in row.get(name, ()))
        edges[row["edge_id"]] = RoadEdge(**row)
    return PartitionGraph(nodes, edges,
                          {int(k): v for k, v in value["regions"].items()},
                          dict(value.get("metadata") or {}), int(value["version"]),
                          dict(value.get("ramp_states") or {}))


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
    penalties = {}
    for edge_id, (source, target) in guidance_transitions(graph).items():
        if source in guided_sources:
            probability = policy.get((source, target), 0.0)
            if probability < 1.0:
                penalties[edge_id] = -float(weight_seconds) * math.log(max(probability, 1e-6))
    return penalties
