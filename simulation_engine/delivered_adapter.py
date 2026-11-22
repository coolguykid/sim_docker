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


class DeliveredTopology:
    def __init__(self, region_id, version, graph, edge_refs, statistics_env,
                 node_external_ids, layer_by_edge):
        """保存区域图、数据库边引用、统计环境和层号映射，并构建查询索引。

        输入
            region_id：区域编号。
            version：路网拓扑版本号。
            graph：内部路网对象。
            edge_refs：图边 ID 到数据库边引用的映射。
            statistics_env：统计器使用的道路边查询环境。
            node_external_ids：内部节点 ID 到数据库节点 ID 的映射。
            layer_by_edge：内部边 ID 到层号的映射。
        输出
            None。
        """
        self.region_id, self.version = int(region_id), int(version)
        self.graph, self.edge_refs = graph, edge_refs
        self.statistics_env = statistics_env
        self.node_external_ids, self.layer_by_edge = node_external_ids, layer_by_edge
        self.rebuild_indexes()

    def rebuild_indexes(self):
        """依据当前图边刷新节点的出边索引。

        输入
            无显式参数。
        输出
            None。
        """
        self.outgoing = {}
        for edge in self.graph.edges.values():
            self.outgoing.setdefault(edge.source, []).append(edge.edge_id)

    def edge_ref(self, edge_id):
        """将内部边 ID 转换成数据库边引用。

        输入
            edge_id：内部图边编号。
        输出
            数据库边引用。
        """
        return self.edge_refs[str(edge_id)]


def build_delivered_topology(region_id, version, network, include_lanes=True):
    """将有效区域数据库拓扑建成导引版区域图和索引。

    输入
        region_id：区域编号。
        version：路网拓扑版本号。
        network：区域网络及设施记录。
        include_lanes：是否计算并保留车道偏移。
    输出
        DeliveredTopology。

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
    edges, refs, stats, layers = {}, {}, {}, {}
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
        refs[name], stats[ref] = ref, StatisticsEdge(edge, row.get("max_capacity", 50))
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
        refs[name], stats[ref] = ref, StatisticsEdge(edge)
        layers[name] = int(raw_nodes[int(row["start_node_id"])].get("layer_id") or 0)
    graph = PartitionGraph(nodes, edges, {region_id: {"region_id": region_id}},
                           {"database_adapter": "region-dt-delivery-v2",
                            "boundary_transitions": {}}, int(version),
                           {int(row["ramp_id"]): bool(row.get("is_open", True))
                            for row in network["ramps"]})
    return DeliveredTopology(region_id, version, graph, refs,
                             StatisticsEnvironment(stats), external_ids, layers)


def build_global_partition_graph(networks, version):
    """拼接区域拓扑，按共享节点生成有向跨区连接。

    输入
        networks：按区域 ID 索引的区域网络集合。
        version：路网拓扑版本号。
    输出
        全局 PartitionGraph。

    原有说明
        Compile the temporary center-DT global routing graph from all regions.
    """
    topologies = {int(rid): build_delivered_topology(rid, version, network, False)
                  for rid, network in networks.items()}
    nodes, edges, regions, ramp_states, transitions = {}, {}, {}, {}, {}
    shared = {}
    for rid, topology in topologies.items():
        nodes.update(topology.graph.nodes)
        edges.update(topology.graph.edges)
        regions[rid] = {"region_id": rid}
        ramp_states.update({"R{}:{}".format(rid, key): value
                            for key, value in topology.graph.ramp_states.items()})
        for row in networks[rid]["nodes"]:
            memberships = tuple(int(v) for v in (row.get("region_ids") or (rid,)))
            if len(memberships) > 1:
                shared.setdefault(int(row["node_id"]), set()).update(memberships)

    incoming, outgoing = {}, {}
    for rid, topology in topologies.items():
        for edge in topology.graph.edges.values():
            incoming.setdefault((rid, edge.target), []).append(edge)
            outgoing.setdefault((rid, edge.source), []).append(edge)
    for external, memberships in shared.items():
        for source_region in sorted(memberships):
            source_node = regional_node_id(external, source_region)
            for target_region in sorted(memberships):
                if source_region == target_region:
                    continue
                target_node = regional_node_id(external, target_region)
                if source_node not in nodes or target_node not in nodes:
                    continue
                for source_edge in incoming.get((source_region, source_node), ()):
                    for target_edge in outgoing.get((target_region, target_node), ()):
                        sl = topologies[source_region].layer_by_edge.get(source_edge.edge_id, 0)
                        tl = topologies[target_region].layer_by_edge.get(target_edge.edge_id, 0)
                        name = "B:N{}:R{}L{}:R{}L{}".format(
                            external, source_region, sl, target_region, tl)
                        if name in edges:
                            continue
                        point = nodes[source_node].position
                        edges[name] = RoadEdge(
                            name, source_node, target_node, source_region, "BOUNDARY",
                            0., min(source_edge.speed_limit_mps, target_edge.speed_limit_mps),
                            (point, point), lane_count=min(source_edge.lane_count,
                                                          target_edge.lane_count))
                        transitions[name] = {
                            "shared_node_id": external,
                            "source_region": source_region, "target_region": target_region,
                            "source_layer": sl, "target_layer": tl,
                        }
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


def attach_global_boundaries(local_topology, global_graph):
    """把本区官方出境边和目标节点附加到本地拓扑并刷新索引。

    输入
        local_topology：当前区域拓扑及索引。
        global_graph：覆盖参与区域的全局路网。
    输出
        更新后的本地拓扑。

    原有说明
        Add only this region's official outbound BOUNDARY edges to its live map.
    """
    rid = local_topology.region_id
    for edge in global_graph.edges.values():
        if edge.kind != "BOUNDARY" or edge.region_id != rid:
            continue
        if edge.source not in local_topology.graph.nodes:
            continue
        local_topology.graph.nodes[edge.target] = global_graph.nodes[edge.target]
        local_topology.graph.edges[edge.edge_id] = edge
    local_topology.graph.metadata["boundary_transitions"] = dict(
        global_graph.metadata.get("boundary_transitions", {}))
    local_topology.rebuild_indexes()
    return local_topology


def overlay_local_region(global_graph, local_topology):
    """将当前尝试的本地有效路网覆盖进全局图。

    输入
        global_graph：覆盖参与区域的全局路网。
        local_topology：当前区域拓扑及索引。
    输出
        更新后的全局图。

    原有说明
        Replace one summary region with this attempt's effective live network.
        
        The center graph may be five minutes old, while network adjustment changes
        pipe/ramp availability or speed for the next attempt.  The official portal
        summary must therefore use the current attempt's local costs for this
        region, without replacing any other region or the center-owned boundaries.
    """
    region_id = int(local_topology.region_id)
    obsolete = [
        edge_id
        for edge_id, edge in global_graph.edges.items()
        if int(edge.region_id) == region_id and edge.kind != "BOUNDARY"
    ]
    for edge_id in obsolete:
        del global_graph.edges[edge_id]
    global_graph.nodes.update(local_topology.graph.nodes)
    global_graph.edges.update(local_topology.graph.edges)
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
