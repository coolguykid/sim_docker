"""
beamforming.py — 波束赋形/通信增益计算（纯物理公式，无外部模块依赖）
"""
import numpy as np


def generate_steering_vector(azimuth, elevation, M_x=5, M_y=4, wavelength=0.1, d_spacing=0.05):
    """生成 2D 平面天线阵列 (UPA) 的复数导向矢量"""
    M = M_x * M_y
    steering_vector = np.zeros(M, dtype=complex)
    k = 2 * np.pi / wavelength
    idx = 0
    for m in range(M_x):
        for n in range(M_y):
            phase = k * d_spacing * (m * np.sin(elevation) * np.cos(azimuth) + n * np.sin(elevation) * np.sin(azimuth))
            steering_vector[idx] = np.exp(1j * phase)
            idx += 1
    return steering_vector.reshape(-1, 1)


def calculate_beamforming_gain(azimuth, elevation, allocated_power, M_x=5, M_y=4):
    """基于方向匹配策略，计算物理波束赋形增益 |a^H * w|^2"""
    if allocated_power <= 0:
        return 0.0
    a = generate_steering_vector(azimuth, elevation, M_x, M_y)
    a_norm = np.linalg.norm(a)
    w = np.sqrt(allocated_power) * (a / a_norm)
    a_H = a.conj().T
    complex_gain = np.dot(a_H, w)[0, 0]
    return np.abs(complex_gain) ** 2
