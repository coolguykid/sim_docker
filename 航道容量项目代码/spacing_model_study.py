"""比较安全距离模型形式；仅生成研究图表，不改变仿真默认模型。"""
from dataclasses import dataclass
import csv
import json
from math import floor, isclose, sqrt
from pathlib import Path

from airway import ThroughputModel


ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "outputs" / "spacing_model_study"
WINDOW, REFERENCE_SPEED = 60.0, 20.0
LOWER, UPPER = 10.0, 50.0
DEFAULT = ThroughputModel()
A = DEFAULT.vehicle_length_m + DEFAULT.main_spacing_factor * DEFAULT.base_spacing_m
B = DEFAULT.main_spacing_factor * DEFAULT.reaction_time_s
R = (DEFAULT.main_spacing_factor * DEFAULT.braking_gap_factor
     / (2 * DEFAULT.deceleration_mps2) * REFERENCE_SPEED**2)


@dataclass(frozen=True)
class Candidate:
    name: str
    kind: str
    power: int = 2
    risk_share: float = 0.0
    threshold: float = 16.0
    added_margin: float = 0.0

    def spacing(self, v):
        if self.kind == "power":
            return A + B*v + R*(v/REFERENCE_SPEED)**self.power
        if self.kind == "threshold":
            return A + B*v + R*((1-self.risk_share)*(v/REFERENCE_SPEED)**2
                   + self.risk_share*(max(v-self.threshold, 0)
                                      / (REFERENCE_SPEED-self.threshold))**2)
        return DEFAULT.safety_distance(v) + self.added_margin*max(v-self.threshold, 0)**2

    def analytical_peak(self):
        if self.kind == "power":
            return REFERENCE_SPEED*(A/((self.power-1)*R))**(1/self.power)
        c = R/REFERENCE_SPEED**2
        gamma = self.added_margin
        if self.kind == "threshold":
            c *= 1-self.risk_share
            gamma = self.risk_share*R/(REFERENCE_SPEED-self.threshold)**2
        # 两个分段的驻点和分界点独立比较，避免仅复用生产求解器验证自身。
        low = min(self.threshold, max(LOWER, sqrt(A/c)))
        high = min(UPPER, max(self.threshold,
                             sqrt((A+gamma*self.threshold**2)/(c+gamma))))
        return max((low, high, self.threshold), key=lambda v: v/self.spacing(v))


CANDIDATES = [
    Candidate("当前二次模型", "power"),
    Candidate("三次项（同一起点）", "power", power=3),
    Candidate("四次项（同一起点）", "power", power=4),
    Candidate("阈值模型 ρ=0.25", "threshold", risk_share=0.25),
    Candidate("阈值模型 ρ=0.50", "threshold", risk_share=0.5),
    Candidate("原模型直接加裕度", "additive", added_margin=2.0),
]


def evaluate(candidate):
    model = ThroughputModel(spacing_function=candidate.spacing)
    q20 = model.throughput(REFERENCE_SPEED, WINDOW)
    best = model.best_speed(LOWER, UPPER)
    reference_peak = candidate.analytical_peak()
    assert abs(best-reference_peak) < 1e-4
    assert isclose(model.throughput(best, WINDOW),
                   WINDOW*reference_peak/candidate.spacing(reference_peak), abs_tol=1e-8)
    if candidate.kind != "additive":
        assert isclose(candidate.spacing(20), DEFAULT.safety_distance(20), abs_tol=1e-10)
    for i in range(1001):
        speed = LOWER + (UPPER-LOWER)*i/1000
        assert candidate.spacing(speed) > 0
        assert isclose(model.throughput(speed, WINDOW),
                       WINDOW*speed/candidate.spacing(speed), abs_tol=1e-10)
        if i:
            assert candidate.spacing(speed) > candidate.spacing(previous_speed)
        previous_speed = speed
    # 固定同一个7架/60秒观测值，体现整数门槛；加裕度组起始吞吐较低。
    proposed = model.speed_for_demand(7*1.01, WINDOW, LOWER, UPPER)
    proposed_q = model.throughput(proposed, WINDOW)
    accepted = (proposed_q > q20+1e-9
                and floor(proposed_q+1e-9) > floor(q20+1e-9))
    row = dict(name=candidate.name, kind=candidate.kind, power=candidate.power,
               risk_share=candidate.risk_share, threshold=candidate.threshold,
               added_margin=candidate.added_margin, d20=candidate.spacing(20), q20=q20,
               best_speed=best, peak_q=model.throughput(best, WINDOW),
               proposed_speed=proposed, proposed_q=proposed_q,
               accepted=accepted, target_met=proposed_q >= 7.07-1e-9,
               applied_time_percent=100*(20/proposed-1) if accepted else 0.0)
    for v in (18, 16):
        q = model.throughput(v, WINDOW)
        row.update({f"d{v}": candidate.spacing(v), f"q{v}": q,
                    f"gain{v}_percent": 100*(q/q20-1), f"whole{v}": floor(q+1e-9)})
    return row


def plot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.sans-serif": ["Microsoft YaHei", "DejaVu Sans"],
                         "axes.unicode_minus": False, "font.size": 11,
                         "svg.fonttype": "none"})
    fig, (left, right) = plt.subplots(1, 2, figsize=(13.6, 5.3))
    fig.suptitle("改变安全距离模型：统一20 m/s时的安全距离与吞吐", fontsize=17, y=.97)
    speeds = [10+i*.025 for i in range(601)]
    colors = ("#64748b", "#2563eb", "#d97706")
    for index, color in zip((0, 2, 4), colors):
        candidate = CANDIDATES[index]
        q = lambda v: WINDOW*v/candidate.spacing(v)
        left.plot(speeds, [q(v) for v in speeds], label=candidate.name,
                  color=color, linewidth=2.4)
        left.scatter((16, 18, 20), [q(v) for v in (16, 18, 20)],
                     color=color, s=25, zorder=3)
        right.plot(speeds, [candidate.spacing(v) for v in speeds],
                   label=candidate.name, color=color, linewidth=2.4)
        right.scatter((16, 18, 20), [candidate.spacing(v) for v in (16, 18, 20)],
                      color=color, s=25, zorder=3)
    left.axvline(16, color="#d97706", linewidth=1, linestyle=":", alpha=.7)
    left.set(xlabel="管道速度（m/s）", ylabel="理论可支持通过量（架次 / 60秒）",
             xlim=(10, 25), ylim=(3, 9.3))
    left.legend(loc="lower left", fontsize=10, framealpha=.95)
    left.grid(alpha=.18)
    right.axvline(16, color="#d97706", linewidth=1, linestyle=":", alpha=.7)
    right.set(xlabel="管道速度 v（m/s）", ylabel="安全距离 d(v)（m）",
              xlim=(10, 25), ylim=(0, 450))
    right.legend(loc="upper left", fontsize=10, framealpha=.95)
    right.grid(alpha=.18)
    fig.text(.5, .025,
             "共同起点：d(20)=187.4 m，Q(20)=6.403架次/60秒；候选为未校准假设，尚未运行完整网络仿真。",
             ha="center", fontsize=10, color="#475569")
    fig.subplots_adjust(left=.065, right=.98, bottom=.16, top=.85, wspace=.23)
    fig.savefig(OUTPUT/"model_comparison.png", dpi=160)
    fig.savefig(OUTPUT/"model_comparison.svg")
    plt.close(fig)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    rows = [evaluate(candidate) for candidate in CANDIDATES]
    with (OUTPUT/"comparison.csv").open("w", encoding="utf-8-sig", newline="") as target:
        writer = csv.DictWriter(target, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = dict(window_s=WINDOW, reference_speed_mps=REFERENCE_SPEED,
                   speed_range=[LOWER, UPPER], A=A, B=B, reference_risk_budget_m=R,
                   candidates=rows, production_model_changed=False,
                   full_network_simulation_run=False,
                   validation="6个模型与分段/幂函数解析峰值及6006个采样点核对通过")
    (OUTPUT/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    plot()
    report = [
        "# 安全距离模型形式探索（2026-10-20）", "",
        "本次仅进行公式计算和现有速度求解器核对，不替换默认模型、不更新网页回放，也不声称真实离网效率已改善。", "",
        "## 比较条件", "",
        "固定60秒窗口，允许速度10–50 m/s；除最后的加裕度反例外，各模型均满足d(20)=187.4 m、Q(20)=6.403415架次/60秒。",
        "原模型为d(v)=53+1.92v+96(v/20)²，Q(v)=60v/d(v)。", "",
        "| 模型 | Q(20) | Q(18) | 20→18增益 | Q(16) | 20→16增益 | 允许速度内峰值 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in rows:
        report.append(f"| {row['name']} | {row['q20']:.3f} | {row['q18']:.3f} | {row['gain18_percent']:.2f}% | "
                      f"{row['q16']:.3f} | {row['gain16_percent']:.2f}% | "
                      f"{row['peak_q']:.3f} @ {row['best_speed']:.2f} m/s |")
    report += ["", "20→18 m/s的同距离纯飞行时间增加11.11%，20→16增加25%；不含排队、绕路和匝道等待。", "",
        "## 候选的含义与限制", "",
        "1. 幂函数：d_p(v)=53+1.92v+96(v/20)^p，p=3或4。改写高速误差/响应裕度的增长规律；不是把标准制动距离直接说成v⁴，也没有实测数据支持这里的指数。",
        "2. 阈值：d(v)=53+1.92v+96[(1−ρ)(v/20)²+ρ(max(v−16,0)/4)²]。ρ控制20 m/s的96 m速度相关安全预算中多少分配给超过16 m/s后才出现的额外风险。阈值处距离和一阶导数连续，参数可独立调整。",
        "   ρ=0.5表示重新分解安全预算：保留一半原二次项，另一半改成阈值项；低速安全距离因此比原式短，需重新校准。若原制动项是不可削减的硬约束，就不能直接如此替换。",
        "3. 加裕度反例：d=d_old+2max(v−16,0)²。Q(20)降至5.469，18 m/s的相对提升13.93%，但低速峰值仍为原来的6.628；不能把这种相对增益等同于提高原有最大能力。", "",
        "## 对现有整数调速规则的影响", "",
        "固定观测为7架/60秒，仍使用1%余量，目标为7.07；选可满足需求的最快速度，不可满足时选理论峰值。", "",
        "| 模型 | 候选速度 | 候选Q | 整数门槛通过 | 达到7.07 | 执行后纯飞行时间变化 |",
        "| --- | ---: | ---: | --- | --- | ---: |",
    ]
    for row in rows:
        report.append(f"| {row['name']} | {row['proposed_speed']:.3f} | {row['proposed_q']:.3f} | "
                      f"{'是' if row['accepted'] else '否'} | {'是' if row['target_met'] else '否'} | "
                      f"{row['applied_time_percent']:+.2f}% |")
    report += ["", "Q=60/h表示持续饱和时的平均能力，非每个60秒滑动窗口的硬上限；例如h=9.37秒时，某个60秒窗口出现7次合法放行仍可能符合最小间隔。整数门槛是当前调速收益规则，不应被实现为每窗口的硬配额。", "",
        "## 建议与后续验证", "",
        "优先把阈值模型作为可选择的研究候选，先用ρ=0.25、0.5两档做单管道测试，再决定是否采用；不应为突出调速效果无限提高曲率，避免把扩容研究压缩为仅需调速的情形。",
        "当前内部运动没有跟驰和匝道合流安全放行约束。若要观察真实排队消散，应让距离/时间间隔约束实际合流或放行；只改理论Q会同时影响着色、控制和入口频率，不能据此认定累计离网改善。",
        "若新增内部服务队列，已放行量会受限流影响，触发调整时应同时考虑已经发生的到达请求、当前队列及队列增长；这些属于当前观测，不需要引入未来预测。",
        "当前λ=αλ₀随速度变化；候选将提高降速后的请求生成频率，网络实验需同时报告生成数、排队、离网比例和任务总耗时。", "",
        "## 参考与假设边界", "",
        "NASA的UTM试验以空间缓冲处理风和导航偏差，支持讨论不确定度裕度，但不提供这里的16 m/s阈值、ρ或四次指数：",
        "https://www.nasa.gov/centers-and-facilities/ames/first-steps-toward-drone-traffic-management/", "",
        "无人机流体排队研究将实际流出率限制为发送需求与链路服务率的较小值，并考虑合流和下游状态，可作为后续动态模型依据：",
        "https://arxiv.org/abs/1904.12988", "",
        "以上具体候选公式和数值由本项目假设给出，非引用论文的已标定模型。",
    ]
    (OUTPUT/"report.md").write_text("\n".join(report)+"\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
