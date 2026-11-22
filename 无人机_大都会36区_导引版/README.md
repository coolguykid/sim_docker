# 无人机空域仿真 大都会 36 区导引版

本目录从 `D:\无人机_大都会36区_第一版增强` 复制并增加区域流量导引。保留参数面板、双层 Dijkstra、三种寻路运行方式、统一秒制时间和集中安全预约。上一个版本的测试报告保存在 `docs/verification_metropolis36/`。本版入口和新增检查见 [区域流量导引](docs/区域流量导引.md)。

原有功能的使用说明见 [新版使用说明](docs/新版使用说明.md)。第一版增强目录保留历史规模实验输出。

[双层寻路实现与风险说明](docs/双层寻路实现与风险说明.md)用通俗语言解释运行机制和模型限制；[实际测试报告](docs/实际测试报告.md)只列真实完成的结果。

## 启动

在项目目录打开 PowerShell：

```powershell
conda activate hxy2
python main.py
```

也可用明确路径运行：

```powershell
Set-Location -LiteralPath 'D:\无人机_大都会36区_导引版'
D:\Anaconda3\envs\hxy2\python.exe main.py
```

新环境安装：

```powershell
python -m pip install -r requirements.txt
python -m pip install -e .
python main.py
```

支持 Python 3.9 及以上。算法不使用神经网络、训练脚本或模型权重，无需安装 PyTorch。

## 保留能力

- 单图 Dijkstra、36 区域两级 Dijkstra、单向匝道和区域穿越。
- 规则换道、限速、加减速约束和服务器安全预测。
- PyQt5 参数界面、标准/都市/36 区域空域、接载点及终点配置。
- 串行仿真、Ray 集中寻路、服务器权威控制和本地规划。
- PostgreSQL 空域读取及外部匝道状态同步。
- 轨迹绘图、逐回合日志、计时、JSON 帧导出及可选 KL/通信指标。

大都会增强模式中的动态道路按密度生成，并在本次运行中保持开启。旧空域模式及数据库接口继续保留在原模块中。默认整次仿真共用 10 个第六层随机内部终点，中间节点数量为 0。

## 目录

- `src/uav_sim/algorithms/`：算法 [1] 入口、规则避障、安全约束及指标工具。
- `src/uav_sim/airspace/`：空域、路网与两级 Dijkstra。
- `src/uav_sim/distributed/`：Ray Actor、集中寻路及规则控制。
- `src/uav_sim/entities/`：无人机状态推进。
- `src/uav_sim/ui/`：参数界面。
- `src/uav_sim/integrations/`：数据库接入。
- `tests/`：保留能力的回归测试及单算法验收检查。
- `docs/verification/`：原文件校验清单及验证记录。

运行输出自动写入 `artifacts/runs/<run_id>/algorithm_1/`。此目录不附带旧项目的历史运行结果。

## 验证

```powershell
python -m pytest -q
```

