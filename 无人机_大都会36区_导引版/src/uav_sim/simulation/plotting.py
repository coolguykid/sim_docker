"""
plotting.py — 结果输出与绘图
"""
import numpy as np
import matplotlib.pyplot as plt

ALGO_STYLES = {
    "1": {
        "color": "red",
        "linewidth": 2.5,
        "linestyle": "--",
        "label_en": "Pure Dijkstra + Rule Avoidance",
    }
}
ALTITUDE_STYLES = {
    100: ("#A5D6A7", "Layer 1 (100m, 0°)"),
    200: ("#80DEEA", "Layer 2 (200m, 30°)"),
    300: ("#90CAF9", "Layer 3 (300m, 60°)"),
    400: ("#CE93D8", "Layer 4 (400m, 90°)"),
    500: ("#B0BEC5", "Layer 5 (500m, 120°)"),
    600: ("#FFCC80", "Layer 6 (600m, 150°)"),
}
UAV_TRAJECTORY_STYLES = [
    {"color": "#1f77b4", "linestyle": "-", "marker": "o"},
    {"color": "#ff7f0e", "linestyle": "--", "marker": "s"},
    {"color": "#2ca02c", "linestyle": "-.", "marker": "^"},
    {"color": "#d62728", "linestyle": ":", "marker": "D"},
    {"color": "#9467bd", "linestyle": "-", "marker": "v"},
    {"color": "#8c564b", "linestyle": "--", "marker": "P"},
    {"color": "#e377c2", "linestyle": "-.", "marker": "X"},
    {"color": "#7f7f7f", "linestyle": ":", "marker": "*"},
]


def _draw_airspace_background(ax, env):
    drawn_altitudes = set()
    for pipe in env.pipelines:
        p_alt = pipe.height
        color, layer_name = ALTITUDE_STYLES.get(p_alt, ("gray", f"Unknown ({p_alt}m)"))
        label_str = layer_name if p_alt not in drawn_altitudes else None
        ax.plot(
            [pipe.start_pt[0], pipe.end_pt[0]],
            [pipe.start_pt[1], pipe.end_pt[1]],
            [pipe.start_pt[2], pipe.end_pt[2]],
            color=color,
            linestyle=":",
            linewidth=2.0,
            alpha=0.3,
            label=label_str,
        )
        drawn_altitudes.add(p_alt)


def _set_3d_airspace_axes(ax, env):
    ax.set_xlim(env.x_min, env.x_max)
    ax.set_ylim(env.y_min, env.y_max)
    ax.set_zlim(env.z_min, env.z_max)
    ax.set_xlabel("Spatial X (m)", fontsize=16, fontweight="bold", labelpad=15)
    ax.set_ylabel("Spatial Y (m)", fontsize=16, fontweight="bold", labelpad=15)
    ax.set_zlabel("Altitude Z (m)", fontsize=16, fontweight="bold", labelpad=15)
    ax.tick_params(axis="both", which="major", labelsize=14)
    ax.view_init(elev=25, azim=-45)


def _plot_uav_key_points(ax, env, uavs, uid, point_color=None, include_labels=True):
    tracked_uav_obj = uavs[uid]
    start_pipe = env.pipelines[tracked_uav_obj.start_pipe_id]
    start_3d = start_pipe.start_pt
    pickup_pt_obj = tracked_uav_obj.pickup_pt
    pickup_3d = None
    if pickup_pt_obj is not None:
        pickup_pipe = env.pipelines[pickup_pt_obj.pipe_id]
        pickup_3d = pickup_pipe.start_pt + pickup_pipe.direction * pickup_pt_obj.pos_1d
    dest_pipe = env.pipelines[tracked_uav_obj.final_dest_pipe_id]
    dest_3d = dest_pipe.end_pt
    road=getattr(tracked_uav_obj,'_road_state',None)
    if road is not None:
        dest_3d=np.asarray(road.goal.position)
        start_3d=start_pipe.lanes[tracked_uav_obj.start_lane_id].start_pt
    if point_color is None:
        ax.scatter(
            start_3d[0],
            start_3d[1],
            start_3d[2],
            c="black",
            marker="^",
            s=300,
            label="Departure Point" if include_labels else None,
        )
        if pickup_3d is not None:
            ax.scatter(
                pickup_3d[0],
                pickup_3d[1],
                pickup_3d[2],
                c="purple",
                marker="*",
                s=450,
                label="Emergency Hub (Pickup)" if include_labels else None,
            )
        ax.scatter(
            dest_3d[0],
            dest_3d[1],
            dest_3d[2],
            c="red",
            marker="X",
            s=300,
            label="Final Destination" if include_labels else None,
        )
        return
    ax.scatter(
        start_3d[0],
        start_3d[1],
        start_3d[2],
        c=point_color,
        marker="^",
        s=120,
        edgecolors="black",
        label=f"UAV {uid} Departure" if include_labels else None,
    )
    if pickup_3d is not None:
        ax.scatter(
            pickup_3d[0],
            pickup_3d[1],
            pickup_3d[2],
            c=point_color,
            marker="*",
            s=180,
            edgecolors="black",
            label=f"UAV {uid} Pickup" if include_labels else None,
        )
    ax.scatter(
        dest_3d[0],
        dest_3d[1],
        dest_3d[2],
        c=point_color,
        marker="X",
        s=140,
        edgecolors="black",
        label=f"UAV {uid} Destination" if include_labels else None,
    )


def _plot_multi_uav_trajectory_overlay(results_dict, env, uavs, target_uav_ids):
    valid_uav_ids = [uid for uid in target_uav_ids if 0 <= uid < len(uavs)]
    if not valid_uav_ids:
        print("未找到可叠加绘制的 UAV 航迹，跳过多 UAV 同图。")
        return
    print(f"正在生成多 UAV 同图航迹对比图 (IDs: {valid_uav_ids})...")
    fig = plt.figure(figsize=(18, 12))
    ax = fig.add_subplot(111, projection="3d")
    for style_idx, uid in enumerate(valid_uav_ids):
        uav_style = UAV_TRAJECTORY_STYLES[style_idx % len(UAV_TRAJECTORY_STYLES)]
        for algo_id in sorted(results_dict.keys()):
            r = results_dict[algo_id]
            algo_style = ALGO_STYLES.get(
                algo_id,
                {
                    "color": "gray",
                    "linewidth": 2,
                    "linestyle": "-",
                    "label_en": algo_id,
                },
            )
            traj = np.array(r.get("trajectories", {}).get(uid, []))
            if len(traj) == 0:
                continue
            marker_interval = max(1, len(traj) // 8)
            ax.plot(
                traj[:, 0],
                traj[:, 1],
                traj[:, 2],
                label=f"UAV {uid} - {algo_id}. {algo_style['label_en']}",
                color=uav_style["color"],
                linewidth=algo_style.get("linewidth", 2.5),
                linestyle=uav_style["linestyle"],
                marker=uav_style["marker"],
                markersize=5,
                markevery=marker_interval,
                alpha=0.9,
            )
        _plot_uav_key_points(
            ax, env, uavs, uid, point_color=uav_style["color"], include_labels=False
        )
    _draw_airspace_background(ax, env)
    _set_3d_airspace_axes(ax, env)
    ax.set_title(
        "3D Multi-UAV Flight Trajectories Overlay",
        fontsize=24,
        fontweight="bold",
        pad=30,
    )
    ax.legend(fontsize=12, loc="center left", bbox_to_anchor=(1.05, 0.5))
    plt.tight_layout()
    plt.show()


def print_comparison_report(results_dict):
    """打印仿真报告表格"""
    has_lb = any(
        (r.get("metrics", {}).get("load_balance") for r in results_dict.values())
    )
    if has_lb:
        print("\n\n" + "=" * 150)
        print(f"{'算法 [1] 仿真报告':^140}")
        print("=" * 150)
        print(
            f"{'算法策略':<35} | {'最晚清空耗时':<12} | {'平均 KL 散度':<14} | {'近距越界总数':<12} | {'碰撞次数':<8} | {'未抵达':<8} | {'负载CV':<10} | {'峰值负载'}"
        )
        print("-" * 150)
        for algo_id in sorted(results_dict.keys()):
            r = results_dict[algo_id]
            lb = r.get("metrics", {}).get("load_balance", {})
            hard_collisions = r.get("metrics", {}).get("hard_collisions", 0)
            if lb:
                last_lb = list(lb.values())[-1]
                cv_str = f"{last_lb['pipe_load_cv']:.4f}"
                peak_str = str(last_lb["max_pipe_load"])
            else:
                cv_str = "-"
                peak_str = "-"
            print(
                f"{algo_id}. {r['name_cn']:<33} | {r['time']:<14} | {r['avg_kl']:<16.4f} | {r['violations']:<14} | {hard_collisions:<10} | {r['missed']:<8} | {cv_str:<10} | {peak_str}"
            )
        print("=" * 150)
    else:
        print("\n\n" + "=" * 120)
        print(f"{'算法 [1] 仿真报告':^110}")
        print("=" * 120)
        print(
            f"{'算法策略':<35} | {'最晚清空耗时':<12} | {'平均 KL 散度':<14} | {'近距越界总数':<12} | {'碰撞次数':<8} | {'未抵达'}"
        )
        print("-" * 120)
        for algo_id in sorted(results_dict.keys()):
            r = results_dict[algo_id]
            hard_collisions = r.get("metrics", {}).get("hard_collisions", 0)
            print(
                f"{algo_id}. {r['name_cn']:<33} | {r['time']:<14} | {r['avg_kl']:<16.4f} | {r['violations']:<14} | {hard_collisions:<10} | {r['missed']}"
            )
        print("=" * 120)


def plot_kl_divergence(results_dict):
    """画图1: 选中算法的 KL 散度对比曲线"""
    plt.figure(figsize=(14, 8))
    for algo_id in sorted(results_dict.keys()):
        r = results_dict[algo_id]
        style = ALGO_STYLES.get(
            algo_id,
            {"color": "gray", "linewidth": 2, "linestyle": "-", "label_en": algo_id},
        )
        plt.plot(
            r["kl_history"],
            label=f"{algo_id}. {style['label_en']}",
            color=style["color"],
            linewidth=style["linewidth"],
            linestyle=style["linestyle"],
        )
    plt.xlabel("Simulation Turn (Seconds)", fontsize=18, fontweight="bold")
    plt.ylabel("KL Divergence (Supply vs Demand)", fontsize=18, fontweight="bold")
    plt.title("KL Divergence Comparison", fontsize=22, fontweight="bold", pad=15)
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    plt.grid(True, linestyle=":", alpha=0.8, linewidth=1.5)
    plt.legend(fontsize=16, loc="upper right")
    plt.tight_layout()
    plt.show()


def plot_beamforming_gain(results_dict):
    """绘制启用通信指标后的增益曲线。"""
    dynamic_algos = [
        k for k in sorted(results_dict.keys()) if results_dict[k].get("gain_history")
    ]
    if not dynamic_algos:
        print("未启用通信增益指标，跳过通信增益图。")
        return
    plt.figure(figsize=(12, 6))
    all_gains = []
    for algo_id in dynamic_algos:
        r = results_dict[algo_id]
        style = ALGO_STYLES.get(
            algo_id,
            {"color": "gray", "linewidth": 2, "linestyle": "-", "label_en": algo_id},
        )
        g_dyn = r["gain_history"]
        t_dyn = r["time"]
        valid_turns = min(len(g_dyn), t_dyn + 10)
        plt.plot(
            g_dyn[:valid_turns],
            label=f"{algo_id}. {style['label_en']}",
            color=style["color"],
            linewidth=style["linewidth"],
            linestyle=style["linestyle"],
        )
        all_gains.extend(g_dyn[:valid_turns])
    plt.xlabel("Simulation Turn (Seconds)", fontsize=18, fontweight="bold")
    plt.ylabel("Average Channel Gain (Watts)", fontsize=18, fontweight="bold")
    plt.title(
        f"Average Beamforming Gain per Active Ramp (Massive MIMO M={8 * 8}, P_max=40W)",
        fontsize=20,
        fontweight="bold",
        pad=15,
    )
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    if all_gains:
        min_gain = min(all_gains)
        max_gain = max(all_gains)
        plt.ylim(min_gain * 0.95, max_gain * 1.05)
    plt.grid(True, linestyle=":", alpha=0.8, linewidth=1.5)
    plt.legend(fontsize=16)
    plt.tight_layout()
    plt.show()


def plot_3d_trajectories(
    results_dict, env, uavs, target_uav_ids, plot_multi_uav_same_figure=False
):
    """画图3: 多架无人机3D航迹对比图"""
    for uid in target_uav_ids:
        print(f"正在生成目标无人机 (ID: {uid}) 的 3D 航迹对比图...")
        fig = plt.figure(figsize=(16, 12))
        ax = fig.add_subplot(111, projection="3d")
        for algo_id in sorted(results_dict.keys()):
            r = results_dict[algo_id]
            style = ALGO_STYLES.get(
                algo_id,
                {
                    "color": "gray",
                    "linewidth": 2,
                    "linestyle": "-",
                    "label_en": algo_id,
                },
            )
            traj = np.array(r["trajectories"].get(uid, []))
            if len(traj) > 0:
                ax.plot(
                    traj[:, 0],
                    traj[:, 1],
                    traj[:, 2],
                    label=f"{algo_id}. {style['label_en']}",
                    color=style["color"],
                    linewidth=style["linewidth"],
                    linestyle=style["linestyle"],
                    alpha=0.9,
                )
        _plot_uav_key_points(ax, env, uavs, uid)
        _draw_airspace_background(ax, env)
        _set_3d_airspace_axes(ax, env)
        ax.set_title(
            f"3D Flight Trajectory (UAV ID: {uid})",
            fontsize=24,
            fontweight="bold",
            pad=30,
        )
        ax.legend(fontsize=15, loc="center left", bbox_to_anchor=(1.05, 0.5))
        plt.tight_layout()
        plt.show()
    if plot_multi_uav_same_figure:
        _plot_multi_uav_trajectory_overlay(results_dict, env, uavs, target_uav_ids)
