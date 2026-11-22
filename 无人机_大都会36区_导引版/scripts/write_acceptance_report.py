"""Summarize actual artifacts; missing or failed cases remain clearly marked."""
import argparse
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
MODES={'centralized_full':'集中完整寻路','centralized_regional':'服务器逐区域寻路','onboard_regional':'机载逐区域寻路（Ray）'}


def fmt(value):
    if value is None:
        return '未取得'
    return str(round(value,3)) if isinstance(value,float) else str(value)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args()
    environment_path=args.root/'execution_environment.json'
    environment=json.loads(environment_path.read_text(encoding='utf-8-sig')) if environment_path.is_file() else {}
    case_overrides=environment.get('case_overrides',{})
    lines=['# 新版本实际测试报告','',
           '代码位置：`D:\\无人机_大都会36区_第一版增强`。基准：`D:\\无人机_仅算法1`。',
           '', '## 万架对照的测试口径','',
           '每组最多发起 10000 个任务，在飞与地面待放行合计不超过 10000。起飞每批最多 1000 架，正式测试采用面板默认的 100 个物理秒补机周期，具体参数以各组 config.json 为准。使用同一个 30 km × 30 km 大都会、每层 50 管道、20 车道、随机种子 42、整次共用 10 个第六层内部终点，中间节点为 0。',
           '', '原计划的万架对照组采用统一静态路网：关闭动态匝道，使用 3100 条静态匝道。GUI 默认低密度动态匝道还会增加 800 条道路，并在本次运行中保持开启；因此这些万架结果不能当成 GUI 所有道路密度、临时封路或任意参数组合的验证。下文小规模六组则覆盖了默认低密度动态匝道路网。',
           '', '每组均独立启动、建图、规划并执行连续安全检查，不复用其他组的航迹或安全证书。为减少总等待时间，部分组在同一台电脑上并行运行；墙钟耗时包含建图及资源竞争，不作为独占电脑时的性能排名。',
           '', '正式有限队列试验的新增任务时段为 4000 秒、累计任务上限 10000、排空时限 120000 秒。GUI 默认新增时段和排空时限各为 18000 秒，且会持续补机；这些运行时限不同，因此不能把长排空试验中的全部到达，解读成 GUI 默认限时内也一定全部到达。超时仍在途或地面待放行的任务会如实计入未完成。',
           '', '累计任务数与同时在飞峰值不同。严格“一万架同时在飞”条件只有实际峰值达到 10000 才算满足；未达到时明确写为未达到。地面等待不会计作在飞，未到达任务不会计作成功。',
           '', '三种模式都使用中央避障的完整参考航程安全预约；第二、三种导航仍在实际过区时计算并校验本区路径。第三种运行使用真实 Ray 工作进程。该模式不取消中央安全服务器。',
           '', '原计划机载万架参数为 1000 架/进程（10 个进程），按用户更新要求未执行完整负载测试。本次实际机载验证使用 120 架、30 架/进程、4 个真实 Ray 工作进程。原面板保留的 300 架/进程在目标一万架时会产生 34 个进程，该组合也未完整验证。',
           '', '## 已完成的万架测试','',
           '| 导航方式 | 步长秒 | 生成/到达 | 在飞峰值 | 未完成 | 安全违规/碰撞 | 墙钟秒（含建图） | 状态 |',
           '|---|---:|---:|---:|---:|---:|---:|---|']
    details=[]
    results={}
    if environment.get('full_load_scope_curtailed'):
        lines[3:3]=['','本次按用户最新要求完成代码核查与三模式 × 两种步长的小规模验证；已跑完的三组万架结果保留，剩余万架长测已停止。完整万架任务到达不等于一万架同时在飞，后者目前未验证通过。','']
    for mode,label in MODES.items():
        for dt in ('1','0.1'):
            case_name=mode+'_'+dt+'s'
            path=args.root/case_overrides.get(case_name,case_name)/'benchmark_result.json'
            if not path.is_file():
                status='按最新要求改用小规模验证；未完成万架长测' if environment.get('full_load_scope_curtailed') else '尚未完成'
                lines.append('| '+label+' | '+dt+' | — | — | — | — | — | '+status+' |')
                continue
            d=json.loads(path.read_text(encoding='utf-8-sig'))
            results[(mode,dt)]=(d,path.parent)
            status='任务全部到达' if d['success'] else '失败或未全部到达'
            if d['peak_active']<10000:
                status+='；并发未达一万'
            else:
                status+='；并发达到一万'
            lines.append('| '+' | '.join([label,dt,str(d['generated'])+'/'+str(d['arrived']),str(d['peak_active']),str(d['unfinished']),
                fmt(d['safety_violations'])+'/'+fmt(d['hard_collisions']),fmt(d.get('total_wall_seconds_including_setup')),status])+' |')
            sample=d.get('independent_sampled_safety') or {}
            details.extend(['',f'### {label}，{dt} 秒','',
                f"- 平均实际飞行时间 {fmt(d.get('mean_travel_seconds'))} 秒；平均地面等待 {fmt(d.get('mean_ground_wait_seconds'))} 秒；最后真实到达时刻 {fmt(d.get('last_arrival_time_seconds'))} 秒。",
                f"- 最大终点坐标误差 {fmt(d.get('max_goal_position_error_m'))} 米；连续检查相位对 {d.get('continuous_phase_pairs_checked')}；独立 10 秒抽查 {sample.get('sample_count')} 次，发现违规对 {sample.get('unsafe_pairs')}。",
                f"- Ray 实际启用：{d['routing'].get('use_ray')}；工作进程 PID：{d['routing'].get('worker_pids')}；实际过区导航校验 {d.get('journey_routing',{}).get('verified_regional_entry_plans')} 次。",
                f"- Python 主进程峰值内存 {fmt(d.get('memory',{}).get('driver_peak_working_set_mib'))} MiB。此数不包含 Ray 子进程，不作为整机内存总量。",
                '- 原始结果：`'+str(path)+'`。'+(' 失败原因：'+d['failed'] if d.get('failed') else '')])
    lines.extend(details)
    lines.extend(['','## 步长一致性',''])
    parity=[]
    for mode,label in MODES.items():
        pair=[results.get((mode,dt)) for dt in ('1','0.1')]
        if not all(pair):
            status='完整万架对照按最新要求停止；小规模步长对照见下文。' if environment.get('full_load_scope_curtailed') else '两组尚未完成，暂不作结论。'
            lines.append('- '+label+'：'+status)
            continue
        records=[]
        for _,directory in pair:
            records.append({row['uav_id']:row for row in
                            (json.loads(line) for line in (directory/'arrivals.ndjson').read_text(encoding='utf-8').splitlines() if line)})
        same_ids=records[0].keys()==records[1].keys()
        maximum=max((abs(records[0][uid]['arrival_time_seconds']-records[1][uid]['arrival_time_seconds'])
                     for uid in records[0].keys()&records[1].keys()),default=None)
        parity.append(dict(mode=mode,same_completed_ids=same_ids,max_arrival_time_delta_seconds=maximum))
        lines.append(f'- {label}：完成任务编号集合相同={same_ids}；逐架真实到达时间最大差={fmt(maximum)} 秒。比较的是物理到达时间，末尾输出网格可相差不足一个时隙。')
    small_root=Path(environment['small_matrix_root']) if environment.get('small_matrix_root') else None
    matrix_path=small_root/'matrix_summary.json' if small_root else None
    if matrix_path and matrix_path.is_file():
        matrix=json.loads(matrix_path.read_text(encoding='utf-8'))
        lines.extend(['','## 按最新要求完成的小规模六组验证','',
            '用户已明确同意不再完整跑完剩余万架测试，改为代码检查和小规模验证。下列六组每组 120 架，分四批、每批 30 架，在 0、100、200、300 个物理秒提交任务；整次共用 10 个随机终点、无中间节点。',
            '', '仍使用完整 30 km × 30 km 六层大都会、每层 50 管道、每管道 20 车道、36 个管理区域和 1000 个入口槽位；开启 GUI 默认低密度动态匝道，并在本次运行中保持道路开放。小规模只是减少任务数，并未换成简化路网。机载模式每组使用 4 个真实 Ray 工作进程，每个逻辑无人机保有自己的规划状态。',
            '', '| 导航方式 | 步长秒 | 到达/总数 | 安全违规/碰撞 | 实际过区规划校验 | 机载地图范围抽查 | 墙钟秒（含建图） |',
            '|---|---:|---:|---:|---:|---:|---:|'])
        for row in matrix['cases']:
            mode,dt=row['case'].rsplit('_',1)
            lines.append('| '+' | '.join([MODES[mode],dt[:-1],str(row['arrived'])+'/120',
                str(row['safety_violations'])+'/'+str(row['hard_collisions']),str(row['regional_plans']),
                str(row['scope_checks']),fmt(row['wall_seconds'])])+' |')
        arrival=max(row['max_arrival_delta_seconds'] for row in matrix['parity'])
        spawn=max(row['max_spawn_delta_seconds'] for row in matrix['parity'])
        scope_count=sum(row['scope_checks'] for row in matrix['cases'])
        lines.extend(['',
            f"逐架比较六组结果：真实到达时间最大差 {fmt(arrival)} 秒，真实起飞时间最大差 {fmt(spawn)} 秒，实际经过的区域列表一致。四批任务提交时刻也逐架核对一致，不只比较最终统计数字。",
            '', f"独立整图 Dijkstra 与双层 Dijkstra 对照 {matrix['oracle']['checked_pairs']} 组（50 个底层管道入口 × 10 个终点），最大代价差 {matrix['oracle']['max_cost_error_seconds']} 秒。六组路网结构指纹一致。",
            '', f"机载模式共完成 {scope_count} 次地图范围抽查，均只持有该逻辑无人机当前区域的详细图；抽查还检查了已到达无人机的规划器被清理。所有六组另做了独立三维位置近邻安全抽查。",
            '', '原始结果：`'+str(matrix_path)+'`；每组完整记录见同目录内的 `verification_result.json`、`summary.json`、`arrivals.ndjson`。'])
    lines.extend(['','## 代码逻辑核查','',
        '- 分区在同一个大都会路网上切分，未复制 36 张完整路网。区域连接摘要与当前区域详细图分开，机载规划器进入新区时替换当前图，版本和时隙不匹配时拒绝继续。',
        '- 区域和区域内均使用 Dijkstra；第二、三种模式在实际进入新区时重新计算路径，并与中央安全预约核对。中央服务器仍持有全局参考，这是导航分布式与避障集中式的组合。',
        '- 时钟以十分之一秒的整数计数；连续运动中的起飞、加减速、边界和到达事件使用物理秒。0.1 秒与 1 秒改变输出同步间隔，没有用一秒位移作为零点一秒执行量。',
        '- 每批上限、在飞与等待合计上限及任务总数守恒均有运行检查；只有终点编号和三维坐标同时正确才计入到达。等待和超时任务不会被计成成功。',
        '- 避障在执行前检查整段连续轨迹；短边剩余时间、跨时隙最近距离、速度连续性、错误版本和真实过区重算已由自动测试覆盖。最新全量回归为 554 项通过，随后 74 个冻结源码文件校验未变化。'])
    reference=environment.get('pre_freeze_reference_case')
    if reference:
        reference_path=args.root/reference['case']/'benchmark_result.json'
        if reference_path.is_file():
            old=json.loads(reference_path.read_text(encoding='utf-8-sig'))
            lines.extend(['','## 冻结前的同参数参考运行','',
                f"集中式 1 秒的较早运行已完成 {old['arrived']}/{old['generated']} 个任务，安全违规 {old['safety_violations']}、碰撞 {old['hard_collisions']}，真实在飞峰值 {old['peak_active']}。该进程启动后曾合入等价内存优化及一个默认不启用的选项，因此保留为参考，并另用冻结的最终代码补跑；上方六组表不把这条旧记录算作最终版本完成。",
                '原始记录：`'+str(reference_path)+'`。'])
    if environment_path.is_file():
        environment=json.loads(environment_path.read_text(encoding='utf-8-sig'))
        pauses=environment.get('resource_pauses',[])
        if pauses:
            lines.extend(['','## 电脑资源暂停记录','',
                '下列暂停用于控制同机内存占用，仿真状态和物理时间均保留。表中的墙钟耗时包含暂停，不能直接当成算法的独占计算耗时。'])
            for pause in pauses:
                duration=fmt(pause.get('seconds'))+' 秒' if pause.get('ended_utc') else '尚未恢复'
                lines.append('- `'+pause['case']+'`：'+duration+'；开始 UTC '+pause['started_utc']+'；结束 UTC '+str(pause.get('ended_utc') or '等待中')+'。')
    attempts=environment.get('interrupted_attempts',[])
    if attempts:
        lines.extend(['','## 未计入完成的中断尝试','',
            '下列记录保留用于说明测试过程，不算成功，也不把被主动中断时的未完成任务算作算法到达失败。后续以同参数完整重跑的结果为准。'])
        for attempt in attempts:
            reason='按用户更新后的测试范围停止长测' if attempt.get('reason_code')=='user_scope_change' else '同机并行时内存余量不足，主动停止'
            lines.append('- `'+attempt['case']+'`：'+reason+'；记录目录 `'+str(args.root/attempt['archived_as'])+'`。未计作完成。')
    lines.extend(['','## 其他已完成验证','',
        '- 原目录完整性结果见 `docs/verification_metropolis36/baseline_integrity.json`；最新全量自动测试日志见 `docs/verification_metropolis36/pytest_final_current_20261102.txt`。这些验证与六组万架负载结果分别记录。',
        '- 1000 架的独立回归结果见 `artifacts/verification/nativebinary1000/benchmark_result.json`；真实 Ray 小规模、两种步长的完整大都会路网运行结果见 `docs/verification_metropolis36/real_ray_metropolis_smoke.json`。小规模通过不代表万架已经通过。'])
    lines.extend(['','## 限制与阅读方法','',
        '- 连续安全检查覆盖整个已预约运动段；10 秒位置抽查是额外独立验证，不能替代连续检查。',
        '- 避障可能让无人机在地面等候。安全优先并不等于保证任意负载都能同时飞满一万架，也没有证明全体任务的总到达时间最优。',
        '- 目前折线转角、区域边界停稳后再继续，未建模真实无人机转弯半径、姿态、风、定位噪声、通信丢失或飞行中的道路突发关闭；不能把本仿真结果当成实飞保证。',
        '- 本报告只承认原始结果文件中实际完成的测试。历史开发失败和中断尝试保留在 verification 目录，不作为验收通过记录。',
        '- Unity 接口提供下一时隙位置和场景信息；万架冷启动预约可能耗时较长，本版本不承诺万架实时刷新达到每秒一帧。'])
    if matrix_path and matrix_path.is_file():
        lines.append('- 小规模 Ray 日志出现内部监控代理连接告警；本次导航、地图范围、到达和安全数据均已实际取得，判定不依赖该内部监控代理。尚未完整验证机载模式的万架性能。')
    (ROOT/'docs'/'实际测试报告.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    (args.root/'time_grid_parity.json').write_text(json.dumps(parity,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(completed_cases=len(results),expected_cases=6,report=str(ROOT/'docs'/'实际测试报告.md'))))


if __name__=='__main__':
    main()
