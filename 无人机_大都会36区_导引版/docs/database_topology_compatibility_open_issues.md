# 数据库拓扑兼容性待确认事项

本文记录当前 PostgreSQL 适配层无法仅凭《数据库设计文档》最终确定的业务语义。代码已经提供临时兼容策略，但这些策略不应被视为数据库合同的最终定义。

| # | 待确认事项 | 当前临时策略 | 主要风险 | 建议数据库字段/约束 | 状态 |
|---|---|---|---|---|---|
| 1 | 匝道入口、出口管道字段的最终 SQL 列名 | 接受 `source_pipe_id/r_in_pipe_id/in_pipe_id/pipe_a_id` 和对应出口别名；缺失时直接报错 | 列名变更会导致拓扑无法加载；不能用最近管道静默猜测 | 固定为 `source_pipe_id`、`target_pipe_id`，加外键 | 待数据库负责人确认 |
| 2 | `Pipe_width` 是整管宽度还是单车道宽度 | 配置允许时临时用 `Pipe_width / Lane_number` 推导 `lane_width`，并记录诊断 | 车道横向位置和避障距离可能偏差 | 在 `lane_info` 明确 `lane_width`，或定义 `Pipe_width` 的业务口径 | 待确认 |
| 3 | `layer_info.Angle` 与当前生成模式角度集合不同 | 数据库模式以 `Start_pt -> End_pt` 坐标计算真实方向，`Angle` 只作为校验信息 | 仅按 Angle 建图会产生错误方向 | 明确 Angle 坐标系、零度方向和旋转正方向 | 待确认 |
| 4 | 缺少正式单调 `topology_version/ramp_state_version/source_version` | 拓扑和匝道状态暂按记录内容、更新时间和数量生成稳定摘要版本 | 同一业务版本可能产生不同摘要，或更新顺序无法表达 | 增加单调 bigint 版本并在同一事务内发布 | 待新增字段 |
| 5 | `ramp_info` 只有单个 `Region_id` | 从入口、出口管道的 `Region_id` 推导跨区域关系 | 数据库 Region_id 与两端管道不一致时无法判断权威值 | 增加 `source_region_id/target_region_id` 或规定必须由管道推导 | 待确认 |
| 6 | `ramp_info.type` 只区分层内/层间 | `type=1` 解释为层间；`type=0` 再根据两端管道区域判断普通层内或跨区域同层 | 类型统计和 portal 构建可能与业务定义不一致 | 扩展枚举：`intra_layer/inter_layer/cross_region_intra_layer` | 待确认 |
| 7 | `uav_data.Speed` 为 `Real[]` | 当前算法仍使用沿当前有向管道方向的标量 `speed_mps` | 三维速度数组直接代入会改变匝道时间、制动距离和缓存键 | 明确数组坐标系、单位，并提供沿管道标量速度或投影规则 | 待确认 |
| 8 | UAV 三维 `Position/Source/Destination` 无法完整表达路由状态 | 运行态继续维护内部 `pipe_id/lane_id/pos_1d/phase/dest_pipe_id` | 交叉管道和匝道附近仅靠坐标会出现归属歧义 | 增加当前子管道、车道、一维位置、任务阶段和目标子管道字段 | 待新增字段 |
| 9 | `Is_open + Update_time` 不能表达未来生效和失效窗口 | 轮询结果默认从当前回合立即生效，`effective_until_turn=None` | 无法准确执行预约关闭、临时管制和回滚 | 增加 `source_version/effective_from_turn/effective_until_turn` | 待新增字段 |
| 10 | 节点 ID 是否跨中心库、区域库全局唯一 | 当前按数据库返回值直接建立共享节点索引 | 不同区域复用同一数字 ID 会错误拼接子管道 | 使用全局节点 ID，或采用 `(region_id,node_id)` 复合键 | 待确认 |
| 11 | 同一共享节点的多入多出分支是否全部允许通行 | 坐标一致且区域集合相交时，为所有 `end -> start` 组合生成有向 `CONTINUE` 边 | 可能开放业务上禁止的转向 | 增加允许转向关系表或节点转向约束 | 待确认 |
| 12 | 数据库 `Length` 与坐标计算值冲突时谁是权威 | 严格模式直接拒绝；非严格模式记录差异，运行仍用坐标预计算长度 | 路径时间、视频几何和数据库报表可能不一致 | 增加长度误差容限和权威来源约定 | 待确认 |

## 已实现且不再属于缺口的能力

- 数据库每条 `pipe_info` 作为有向子管道，内部映射为连续整数 ID。
- 共享节点通过有向 `CONTINUE` 边拼成完整图，不进入 UAV 匝道状态。
- 数据库每条 Ramp 按 `R_in -> R_out` 解释为独立单向实体，两个方向可独立开关。
- 匝道长度在加载时由三维端点计算一次；Dijkstra 和实际切换均使用 `length / UAV speed`。
- `D_safe` 已进入规则/RL 避障特征、服务器安全盾和碰撞检测。
- PostgreSQL 拓扑读取使用同一个 `REPEATABLE READ, READ ONLY` 事务；高频循环不会逐 UAV 查询数据库。
