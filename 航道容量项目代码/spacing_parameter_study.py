"""只扫描现有安全距离公式的参数，不修改航路网默认模型或加入波动项。"""
from dataclasses import replace
import csv
import json
from math import ceil, floor, sqrt
from pathlib import Path

from airway import ThroughputModel


ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT/'outputs'/'spacing_parameter_study'
WINDOW = 60
INITIAL_SPEED = 20
LOWER, UPPER = 10, 50
EPS = 1e-9


def evaluate(model, name, kind):
    a = model.vehicle_length_m+model.main_spacing_factor*model.base_spacing_m
    b = model.main_spacing_factor*model.reaction_time_s
    c = model.main_spacing_factor*model.braking_gap_factor/(2*model.deceleration_mps2)
    q20 = model.throughput(INITIAL_SPEED,WINDOW)
    q18, q16 = (model.throughput(v,WINDOW) for v in (18,16))
    best_speed = model.best_speed(LOWER,UPPER)
    peak = model.throughput(best_speed,WINDOW)
    # 按现有控制器，刚达到/超过理论通过量的首个整数实测值。
    observed = ceil(q20-EPS)
    proposed = model.speed_for_demand(observed*1.01,WINDOW,LOWER,UPPER)
    proposed_q = model.throughput(proposed,WINDOW)
    accepted = (abs(proposed-INITIAL_SPEED)>1e-10 and proposed_q>q20+EPS
                and floor(proposed_q+EPS)>floor(q20+EPS))
    row = dict(name=name,kind=kind,**model.metadata(),A=a,B=b,C=c,
               distance20_m=model.safety_distance(20),q20=q20,q18=q18,q16=q16,
               gain18_percent=100*(q18/q20-1),gain16_percent=100*(q16/q20-1),
               whole20=floor(q20+EPS),whole18=floor(q18+EPS),
               unconstrained_peak_speed=sqrt(a/c),best_speed=best_speed,peak_q=peak,
               first_overload=observed,proposed_speed=proposed,proposed_q=proposed_q,
               accepted=accepted,applied_speed=proposed if accepted else INITIAL_SPEED,
               proposed_time_percent=100*(INITIAL_SPEED/proposed-1),
               applied_time_percent=100*(INITIAL_SPEED/(proposed if accepted else INITIAL_SPEED)-1))
    for v in (10,14,16,18,20,30,50):
        assert abs(model.throughput(v,WINDOW)-WINDOW*v/(a+b*v+c*v*v))<1e-10
    for v in (10,12,14,16,18):
        assert model.throughput(v,WINDOW)/q20 <= INITIAL_SPEED/v+1e-10
    return row


def main():
    OUTPUT.mkdir(parents=True,exist_ok=True)
    base = ThroughputModel()
    selected = [
        ('当前参数',base),
        ('基础距离10 m',replace(base,base_spacing_m=10)),
        ('基础距离20 m、反应时间0.6 s',replace(base,base_spacing_m=20,reaction_time_s=.6)),
        ('基础距离15 m、反应时间0.3 s',replace(base,base_spacing_m=15,reaction_time_s=.3)),
        ('仅减速度1 m/s²',replace(base,deceleration_mps2=1)),
    ]
    rows = [evaluate(m,name,'selected') for name,m in selected]
    ranges = dict(base_spacing_m=[5,10,15,20,25,30,40,60,90],
                  reaction_time_s=[.1,.3,.6,.9,1.2,1.5,2,3],
                  deceleration_mps2=[.5,.8,1,1.2,1.5,2,3,4,8],
                  braking_gap_factor=[.3,.45,.6,.75,.9,1.2],
                  main_spacing_factor=[1,1.2,1.3,1.4,1.6,2])
    for parameter,values in ranges.items():
        for value in values:
            rows.append(evaluate(replace(base,**{parameter:value}),f'{parameter}={value}','one_parameter'))
    for s0 in (5,10,12,15,20,25,30):
        for reaction in (.1,.3,.6,.9,1.2):
            for decel in (.8,1,1.2,1.5,2,3):
                model = replace(base,base_spacing_m=s0,reaction_time_s=reaction,deceleration_mps2=decel)
                rows.append(evaluate(model,f's0={s0},T={reaction},a={decel}','combined'))
    with (OUTPUT/'sensitivity.csv').open('w',encoding='utf-8-sig',newline='') as target:
        writer=csv.DictWriter(target,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)
    summary=dict(window_s=WINDOW,initial_speed_mps=INITIAL_SPEED,speed_range=[LOWER,UPPER],
                 cases=len(rows),selected=rows[:len(selected)],
                 gain_upper_bound={'20_to_18_percent':100*(20/18-1),'20_to_16_percent':25,
                                   '20_to_18_with_peak_at_least_10_percent':100*((100/20+20)/(100/18+18)-1)},
                 scope='公式参数敏感性分析；不代表机型校准，不修改默认参数，不运行候选网络仿真')
    (OUTPUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    plot(selected)
    report = [
        '# 安全距离参数探索（2026-10-15）','',
        '本次只改变原公式参数，固定20 m/s初速、60秒窗口、机长5 m、允许速度10–50 m/s；未加入天气或波动项，也未将候选设为默认。',
        '', '公式可写为 `d(v)=A+Bv+Cv²`，`Q(v)=60/(A/v+B+Cv)`。',
        '其中 `A=机长+放大系数×基础距离`，`B=放大系数×反应时间`，`C=放大系数×制动项系数/(2×减速度)`；无速度上下限时，峰值速度为 `sqrt(A/C)`。',
        '', '## 候选曲线', '',
        '| 参数 | 20 m/s吞吐 | 18 m/s吞吐 | 增益 | 整数能力20→18 | 允许速度内峰值速度 |',
        '| --- | ---: | ---: | ---: | --- | ---: |']
    for row in summary['selected']:
        report.append(f"| {row['name']} | {row['q20']:.3f} | {row['q18']:.3f} | {row['gain18_percent']:.2f}% | {row['whole20']}→{row['whole18']} | {row['best_speed']:.2f} |")
    report += ['', '以上吞吐单位均为架次/60秒，20→18 m/s的同距离纯飞行时间统一增加11.11%。基础距离10 m的理论峰值为9.35 m/s，受当前下限约束只能取10 m/s。',
               '', '## 刚超载时，现有控制器会怎么选', '',
               '假设当前刚达到首个超载整数，继续沿用控制器1%余量，并新增整数吞吐必须增长的检查：', '',
               '| 参数 | 首个超载实测值 | 候选速度 | 候选吞吐 | 是否接受 | 执行后纯飞行时间变化 |',
               '| --- | ---: | ---: | ---: | --- | ---: |']
    for row in summary['selected']:
        report.append(f"| {row['name']} | {row['first_overload']} | {row['proposed_speed']:.2f} | {row['proposed_q']:.3f} | {'接受' if row['accepted'] else '跳过'} | {row['applied_time_percent']:+.2f}% |")
    report += ['', '这是单管道公式工况，不是完整网络仿真结果；严重超载时仍可能选择较低的峰值速度，整数检查本身不能保证总任务时间缩短。',
               '', '## 结论与适用范围', '',
               '- 原参数6.40→6.63不能增加整数吞吐，新规则会跳过该调速，继续处理匝道和扩容；恢复提速不受此整数门槛限制。',
               '- 仅调整参数即可找到有整数增益的候选，基础距离20 m、反应时间0.6 s适合作为下一轮候选对比；这些数值尚未按真实机型校准。',
               '- 减少基础距离或反应时间改变了安全假设，需要由设备、定位及控制能力确定其合理取值；不能只根据曲线好看来认定安全。',
               '- 单纯减小减速度虽然提高相对降速增益，却会降低绝对吞吐，甚至仍跨不过整数门槛；不宜把它作为默认调参方向。',
               '- 对非负A、B、C及降速v≤20，恒有 `Q(v)/Q(20)≤20/v`。所以20→18的吞吐增益上限为11.11%，只改这类二次公式无法让小幅降速获得任意大的吞吐增益；这不否定减少排队后可能获得的总旅时收益。',
               '- 若还要求理论峰值速度不低于10 m/s，20→18的连续吞吐增益上限收紧为6.13%；要达到10%则须 `C≥A/18+B/2`，理论峰位至多4.24 m/s，落在当前允许速度范围外。',
               '- 扫描只研究理论曲线与单次整数门槛，不包含候选网络运行；改变安全间距也会同步改变入口λ，完整实验需分别记录生成、排队和离网量。',
               '', f'共扫描{len(rows)}组；单参数范围和组合完整数值见 `sensitivity.csv`，汇总见 `summary.json`。', '',
               '![参数曲线与收益代价](parameter_comparison.png)', '']
    (OUTPUT/'report.md').write_text('\n'.join(report),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False))


def plot(selected):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
    colors=['#173a51','#d77e30','#147888','#795cb1','#b15c5c']
    speeds=[5+i*.05 for i in range(501)]
    with plt.rc_context({'font.size':10,'svg.fonttype':'path'}):
        fig,axes=plt.subplots(1,2,figsize=(12.6,4.5),layout='constrained')
        axes[0].axvspan(5,10,color='#e7e9ec',alpha=.7)
        axes[0].axvline(20,color='#9aabb6',ls='--',lw=1)
        for (name,model),color in zip(selected,colors):
            axes[0].plot(speeds,[model.throughput(v,WINDOW) for v in speeds],color=color,label=name,lw=2)
            axes[0].scatter([18,20],[model.throughput(v,WINDOW) for v in (18,20)],color=color,s=17,zorder=3)
            relevant=[20-i*.01 for i in range(572)]
            costs=[100*(20/v-1) for v in relevant]
            gains=[100*(model.throughput(v,WINDOW)/model.throughput(20,WINDOW)-1) for v in relevant]
            axes[1].plot(costs,gains,color=color,lw=2)
        axes[0].set(xlim=(5,30),ylim=(0,16))
        axes[0].set_title('只调整原公式参数的吞吐曲线',fontproperties=font)
        axes[0].set_xlabel('速度（m/s；灰区低于当前允许下限）',fontproperties=font)
        axes[0].set_ylabel('可支持吞吐量（架次/60秒）',fontproperties=font)
        axes[0].legend(prop=font,frameon=False,loc='upper right',fontsize=9)
        axes[1].plot([0,40],[0,40],color='#9aabb6',ls='--',lw=1)
        axes[1].axvline(100*(20/18-1),color='#c3ccd2',ls=':',lw=1)
        axes[1].set(xlim=(0,40),ylim=(0,40))
        axes[1].set_title('吞吐增益与纯飞行时间代价',fontproperties=font)
        axes[1].set_xlabel('同距离纯飞行时间增加（%）',fontproperties=font)
        axes[1].set_ylabel('相对20 m/s吞吐增益（%）',fontproperties=font)
        axes[1].text(19,35,'虚线：二次模型的数学上界',fontproperties=font,color='#647886',fontsize=9)
        for axis in axes:
            axis.grid(alpha=.2);axis.spines[['top','right']].set_visible(False)
        fig.savefig(OUTPUT/'parameter_comparison.png',dpi=160)
        fig.savefig(OUTPUT/'parameter_comparison.svg',metadata={'Date':None})
        plt.close(fig)


if __name__=='__main__':
    main()
