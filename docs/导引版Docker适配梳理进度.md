# 导引版 Docker 适配梳理进度

## 路网和寻路

- `PartitionGraph` 是导引版的路网结构，可表示本区图或全局图。`RegionMap` 是单区寻路视图，`adjacency` 按起点保存 `(终点, 代价, 边 ID)`。
- 每轮从本区数据库初始快照叠加累计调整，生成有效快照。适配器的 `build_region_topology()` 将节点、开放管道、开放匝道和航道转换为本区图。`_lane_offsets()` 将航道起止坐标转换为相对管道中心线的偏移，供飞机定位。
- 本区出口节点由 `boundary_type=2` 标识。恰好由两区共享时，用 `region_ids` 确定对侧区域，用 `layer_id` 确定两侧层号，直接生成零长度有向 `BOUNDARY` 边。全局图汇总区域之间的连接。
- 本区 `RegionMap` 直接从本轮区域图构造。全局图提供 `PortalSummary` 和 `Destination`，本轮有效本区图也会覆盖进当前会话的全局图副本，让门户代价反映本轮状态。寻路视图使用一致的拓扑版本与代价修订号，CSV 保留本区数据库快照版本。
- 导引 P 的稀疏转移条目由 `guidance_transitions()` 和 `guidance_penalties()` 转换为边的附加代价，再交给原生 `PartitionMaps.with_edge_penalties()`。这两个函数与路网输入转换同在 Docker 适配器中。

[路网输入适配器](../simulation_engine/routing_planning_input_adapter.py) · [区域会话](../simulation_engine/local_session.py) · [导引版原生寻路](../无人机_大都会36区_导引版/src/uav_sim/airspace/partition_router.py)

## 无人机状态和推进

- 一架 UAV 持有一个当前的原生 `RoadState`。其中 `route` 保存要走的边序列，`edge_id`、`progress`、`lane_id`、`position`、`speed` 描述当前运动状态。`PreparedMotion.state` 是候选状态，提交后才成为当前状态。
- 原生分区模式将 `Destination` 存入 `RoadState.goal`。Docker 寻路也使用原生 `Destination`，但执行时由 `_set_goal()` 提供 `node_id`、`lane_id`、`position`。跨区航段先以本区出口为执行目标，到达后由目标区域接手并重新规划。
- `DeliveredUAV` 保存 `RoadState`、区域图引用、加减速上限和起飞帧，负责提交候选状态，并通过 `current_edge_ref` 向统计与 CSV 提供数据库设施引用。批量候选运动由区域会话调用原生 `RegionalControl` 计算。
- 区域会话直接持有原生图、统计环境、数据库节点 ID 映射及边层号映射。初始任务没有 `current_edge` 时，从本区 `RegionMap.adjacency` 选初始边。入境移交也查该邻接表，排除跨区边并优先匹配目标层。`edge_ref()` 从原生 `RoadEdge` 派生数据库设施引用。

[原生 RoadState 与推进](../无人机_大都会36区_导引版/src/uav_sim/entities/partition_motion.py) · [Docker 无人机封装](../simulation_engine/delivered_uav.py) · [区域会话](../simulation_engine/local_session.py)

## 当前进度

- 区域构图入口已改名为 `build_region_topology()`，模块为 `routing_planning_input_adapter.py`。已移除 `HybridPartitionMaps`、`DeliveredTopology` 和重复的 `outgoing` 索引。适配改动集中在 Docker 侧。
- 最新一次接入验证为 7 项测试通过，覆盖本区构图、导引代价、区内规划、跨区出口选择和目标区接续。指定航道上的精确中途目标仍需单独验证。上述源码整理后尚未重建运行中的 Docker 容器。

[接入测试](../test/test_delivered_engine_smoke.py) · [分区路网结构详解](导引版分区路网数据结构与示例.md) · [区域推演引擎接口详解](区域推演引擎源码接口与数据结构.md)
