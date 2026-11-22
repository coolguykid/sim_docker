# 35区域分布式区域 DT

本目录部署35个相互独立的区域 DT 容器和一个跨区域推演协调器。当前
`region_data` 提供区域1至35的数据；区域36尚未接入。

- 区域 DT 的 `workflow.py` 是业务流程核心：接收本区导引后加载本区路网和 UAV
  快照，启动本区推演，保存逐帧 CSV，计算本区统计，判断拥堵，按需调用航路
  调整，推演通过后提交路网变化并下发导引。
- 协调器只是分布式推演工具：统一仿真步、形成同步屏障并转发跨区域移交事件。
  它不加载业务数据，不解释导引，也不判断拥堵或修改路网。

```powershell
cd D:\shuo\sim_docker
docker compose up -d --build
docker compose ps
```

协调器健康检查地址为 `http://127.0.0.1:18080/health`。区域1至区域35
服务端口为 `18101` 至 `18135`，对应 PostgreSQL 宿主机端口为
`15431` 至 `15465`。

提交交付版完整 `centralized_regional` 跨区冒烟测试。该脚本临时模拟中心 DT，
默认从 35 个区域容器的 `/api/v1/topology` 汇总全局图，再向区域1和区域2的
`/api/v1/guidance` 提交同一次导引；导引不是提交给协调器：

```powershell
C:\Users\coolguykid\anaconda3\envs\traffic\python.exe `
  scripts\submit_guidance_demo.py `
  --topology-source docker `
  --run-id workflow_demo_001 `
  --duration 10
```

查询区域 1 的业务工作流：

```powershell
Invoke-RestMethod `
  http://127.0.0.1:18101/api/v1/simulations/workflow_demo_001/attempts/0 |
  ConvertTo-Json -Depth 20
```

查询协调器中该次推演尝试的同步状态：

```powershell
Invoke-RestMethod `
  http://127.0.0.1:18080/api/v1/runs/workflow_demo_001/attempts/0 |
  ConvertTo-Json -Depth 20
```

各区域的输出分别位于：

```text
output/region_XX/<run_id>/attempt_000/uav_states.csv
output/region_XX/<run_id>/attempt_000/edge_statistics.csv
```

完整职责、执行链、源码结构、接口以及已验证结果见
`docs/区域DT业务工作流与跨区协调器职责_2026-11-05.md`。

三个核心模块逐文件、逐函数的输入输出和调用关系见
`docs/三模块源码函数接口与调用关系_2026-11-05.md`。
