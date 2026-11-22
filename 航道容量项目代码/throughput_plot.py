"""从控制器同一模型绘制MATLAB参考曲线，输出可离线展示的SVG。"""
from dataclasses import replace
from io import StringIO
from pathlib import Path

from airway import Config, ThroughputModel


def make_throughput_plot(model: ThroughputModel, config: Config) -> tuple[str, list[dict]]:
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from matplotlib.font_manager import FontProperties

    font_path = Path('C:/Windows/Fonts/msyh.ttc')
    font = FontProperties(fname=str(font_path)) if font_path.exists() else FontProperties()
    speeds = [10+i*.1 for i in range(401)]
    rows = []
    with plt.rc_context({'svg.fonttype': 'path', 'font.size': 10}):
        fig, ax = plt.subplots(figsize=(10, 3.6), layout='constrained')
        colors = ['#147888', '#d77e30', '#688b44', '#785f9f']
        variants = ([model] if model.spacing_function is not None else
                    [replace(model,base_spacing_m=s0) for s0 in sorted({model.base_spacing_m,30.0,60.0,90.0})])
        for i, alternative in enumerate(variants):
            s0 = alternative.base_spacing_m
            active = s0 == model.base_spacing_m
            label = ('当前自定义安全距离模型' if model.spacing_function is not None else
                     f's0 = {s0:g} m'+('（当前模型）' if active else ''))
            ax.plot(speeds, [alternative.throughput(v, config.throughput_window_s) for v in speeds],
                    color=colors[i], linewidth=2.4 if active else 1.7, label=label)
            samples = list(range(10, 51, 2))
            ax.scatter(samples, [alternative.throughput(v, config.throughput_window_s) for v in samples],
                       color=colors[i], s=10, zorder=3)
            for v in samples:
                rows.append(dict(s0=s0 if model.spacing_function is None else None, v0=v,
                                 safeGap=alternative.safety_gap(v) if model.spacing_function is None else None,
                                 mainSpacing=alternative.main_spacing_factor*alternative.safety_gap(v) if model.spacing_function is None else None,
                                 safetyDistance=alternative.safety_distance(v),
                                 throughputVehPerSec=alternative.throughput(v, 1),
                                 throughputPerWindow=alternative.throughput(v, config.throughput_window_s)))
        best = model.best_speed(config.min_speed_mps, config.max_speed_mps)
        peak = model.throughput(best, config.throughput_window_s)
        ax.scatter([best], [peak], color='#147888', s=42, zorder=4)
        ax.annotate(f'当前模型峰值：{best:.1f} m/s，{peak:.2f} 架次/窗口', (best, peak),
                    xytext=(best+5, peak+.35), fontproperties=font, color='#147888',
                    arrowprops=dict(arrowstyle='-', color='#147888'))
        ax.axvline(config.initial_speed_mps, color='#a8b6bc', linestyle='--', linewidth=1)
        ax.set_xlabel('管道速度（m/s）', fontproperties=font)
        ax.set_ylabel(f'可支持通过量（架次/{config.throughput_window_s:g}秒）', fontproperties=font)
        ax.set_xlim(10, 50)
        ax.set_ylim(bottom=0)
        ax.grid(alpha=.2)
        ax.spines[['top', 'right']].set_visible(False)
        ax.legend(prop=font, frameon=False, loc='upper right')
        output = StringIO()
        fig.savefig(output, format='svg', metadata={'Date': None})
        plt.close(fig)
    return output.getvalue(), rows


if __name__ == '__main__':
    output = Path('outputs')
    output.mkdir(exist_ok=True)
    svg, _ = make_throughput_plot(ThroughputModel(), Config())
    (output/'speed_throughput.svg').write_text(svg, encoding='utf-8')
