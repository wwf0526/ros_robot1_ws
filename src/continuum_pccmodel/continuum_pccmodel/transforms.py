"""
transforms.py

这个文件用于存放连续体机器人 PCC 正运动学中常用的位姿变换工具。

主要功能：
1. 生成 4x4 齐次变换矩阵
2. 将旋转矩阵转换为 ZYX 欧拉角
3. 检查旋转矩阵是否有效

为什么要单独建这个文件？
因为后续 forward_kinematics.py、inverse_kinematics.py、MPC、RL 都可能需要用到这些函数。
把这些基础函数独立出来，后续代码会更清楚。
"""

import math
import numpy as np


def make_translation_z(z_m: float) -> np.ndarray:
    """
    生成沿 z 轴方向平移的 4x4 齐次变换矩阵。

    参数：
        z_m:
            沿 z 轴平移的距离，单位 m。

    返回：
        T:
            4x4 齐次变换矩阵。

    使用场景：
        当一段连续体的三根绳收缩量相同，
        认为该段不发生弯曲，只发生轴向长度变化。
    """

    # 创建 4x4 单位矩阵
    T = np.eye(4, dtype=float)

    # 齐次变换矩阵最后一列的前三项表示平移
    T[2, 3] = float(z_m)

    return T


def rotm_to_euler_zyx(R: np.ndarray):
    """
    将 3x3 旋转矩阵转换为 ZYX 欧拉角。

    返回顺序：
        yaw, pitch, roll

    单位：
        rad

    解释：
        ZYX 欧拉角对应：
        R = Rz(yaw) * Ry(pitch) * Rx(roll)

    注意：
        这里使用标准 ZYX 提取方式。
        sy = sqrt(R[0,0]^2 + R[1,0]^2)
        不能使用 sqrt(R[2,0]^2 + R[2,1]^2)，否则小弯曲时可能错误得到约 45°。
    """

    R = np.array(R, dtype=float)

    if R.shape != (3, 3):
        raise ValueError("rotm_to_euler_zyx() 输入必须是 3x3 旋转矩阵")

    # 标准 ZYX 欧拉角判断项
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        yaw = math.atan2(R[1, 0], R[0, 0])
        pitch = math.atan2(-R[2, 0], sy)
        roll = math.atan2(R[2, 1], R[2, 2])
    else:
        yaw = math.atan2(-R[0, 1], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        roll = 0.0

    return float(yaw), float(pitch), float(roll)


def is_rotation_matrix(R: np.ndarray, tol: float = 1e-6) -> bool:
    """
    检查一个矩阵是否接近有效旋转矩阵。

    有效旋转矩阵需要满足：
        1. R 是 3x3 矩阵
        2. R.T @ R ≈ I
        3. det(R) ≈ 1

    参数：
        R:
            待检查矩阵。

        tol:
            数值误差容差。

    返回：
        True:
            矩阵接近有效旋转矩阵。

        False:
            矩阵不是有效旋转矩阵。
    """

    R = np.array(R, dtype=float)

    if R.shape != (3, 3):
        return False

    # 单位矩阵
    I = np.eye(3)

    # 检查正交性：R.T @ R 应该等于 I
    orthogonal_error = np.linalg.norm(R.T @ R - I)

    # 检查行列式：det(R) 应该等于 1
    det_error = abs(np.linalg.det(R) - 1.0)

    return bool(orthogonal_error < tol and det_error < tol)


def make_transform(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    """
    根据旋转矩阵 R 和位置向量 p 生成 4x4 齐次变换矩阵。

    参数：
        R:
            3x3 旋转矩阵。

        p:
            3维位置向量，单位 m。

    返回：
        T:
            4x4 齐次变换矩阵。
    """

    R = np.array(R, dtype=float)
    p = np.array(p, dtype=float).reshape(3)

    if R.shape != (3, 3):
        raise ValueError("R 必须是 3x3 矩阵")

    T = np.eye(4, dtype=float)

    # 左上角 3x3 是旋转矩阵
    T[:3, :3] = R

    # 右上角 3x1 是位置向量
    T[:3, 3] = p

    return T
