"""
kl_divergence.py — 基于高斯源强度的 KL 散度计算
"""
import numpy as np
from uav_sim.entities.uav import get_uav_3d_pos


def compute_kl_divergence(uavs, pickup_points, env, sigma=None):
    """
    基于高斯源强度的 KL 散度计算（运力异构版本）。

    需求分布 P：5个运力点，每个建模为强度=pt.remaining 的高斯源。
    供给分布 Q：飞行中的无人机，建模为强度=u.gaussian_intensity 的高斯源。
    KL(P‖Q) → 0 表示无人机的供给分布与运力需求分布在空间上趋于吻合。
    """
    if sigma is None:
        sigma = max(500.0, getattr(env, "xy_span", 5000.0) / 10.0)
    xs = np.linspace(env.x_min, env.x_max, 10)
    ys = np.linspace(env.y_min, env.y_max, 10)
    zs = np.array([100, 200, 300, 400, 500, 600])
    grid_x, grid_y, grid_z = np.meshgrid(xs, ys, zs, indexing='ij')
    grid_pts = np.stack([grid_x.flatten(), grid_y.flatten(), grid_z.flatten()], axis=1)

    P = np.zeros(len(grid_pts))
    Q = np.zeros(len(grid_pts))

    # P：运力点高斯源
    for pt in pickup_points:
        if pt.remaining > 0:
            pipe = env.pipelines[pt.pipe_id]
            pt_3d = pipe.start_pt + pipe.direction * pt.pos_1d
            dist_sq = np.sum((grid_pts - pt_3d) ** 2, axis=1)
            P += pt.remaining * np.exp(-dist_sq / (2 * sigma ** 2))

    # Q：飞行中无人机高斯源
    for u in uavs:
        if u.state not in ["ARRIVED", "WAITING"] and u.gaussian_intensity > 0:
            u_3d = get_uav_3d_pos(u, env)
            dist_sq = np.sum((grid_pts - u_3d) ** 2, axis=1)
            Q += u.gaussian_intensity * np.exp(-dist_sq / (2 * sigma ** 2))

    P_sum = np.sum(P)
    Q_sum = np.sum(Q)

    if P_sum < 1e-6:
        return 0.0

    P_norm = P / P_sum
    Q_norm = Q / Q_sum if Q_sum > 1e-6 else np.ones_like(Q) / Q.size

    eps = 1e-9
    P_norm = np.clip(P_norm, eps, 1.0)
    Q_norm = np.clip(Q_norm, eps, 1.0)

    return np.sum(P_norm * np.log(P_norm / Q_norm))

