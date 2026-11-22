# 算法 [1] 数据库接入

GUI 的“空域数据源”支持 PostgreSQL 与程序生成空域。数据库连接信息从环境变量读取：

```powershell
$env:UAV_SIM_DB_HOST = "127.0.0.1"
$env:UAV_SIM_DB_NAME = "uav_airspace"
$env:UAV_SIM_DB_USER = "readonly_user"
$env:UAV_SIM_DB_PASSWORD = "连接密码"
python main.py
```

读取 `region_info/layer_info/pipe_info/lane_info/ramp_info`，通过 `DatabaseTopologyAdapter` 转为运行时有向图。数据验证失败时默认不回退到程序生成拓扑。

需要连通管道的一维坐标、方向、层高、长度、车道数量和限速，以及匝道入口/出口、长度、方向及启用状态。规则避障需要同管道同车道中其他无人机的位置、速度和安全间距。

外部匝道状态通过 `PostgresRampStateProvider`、`RampStatePoller` 和 `AirspaceRegistry.apply_external_ramp_states()` 更新；版本变化后会失效寻路缓存并重新规划。数据库语义的待确认事项保存在 `database_topology_compatibility_open_issues.md`。

配置样例见 `configs/default.yaml`；依赖为 `psycopg[binary]`。数据库查询和适配实现位于 `src/uav_sim/integrations/` 与 `src/uav_sim/airspace/database_topology_adapter.py`。
