"""
inverse_kinematics.py

PCC 连续体机器人逆运动学模型。

核心作用：
1. 给定目标末端位置或目标末端姿态；
2. 通过优化方法求解每段三根绳的长度变化 DL；
3. 内部反复调用 forward_kinematics.py；
4. 使模型末端位姿尽量接近目标位姿。

对应 MATLAB 中的 fmincon 思路：
    target
    ↓
    优化变量 DL
    ↓
    正运动学 FK
    ↓
    计算误差
    ↓
    fmincon / minimize 求最优 DL

注意：
- 这里的逆运动学用于后续 MPC / RL 的参考轨迹、目标初始化或离线求解；
- 实时控制时不一定每个周期都调用 IK；
- 第一版先使用 scipy.optimize.minimize 的 SLSQP 方法，对应 MATLAB fmincon 的 SQP 思路。
"""

from typing import Any, Dict, List, Optional

import numpy as np
from scipy.optimize import minimize

from continuum_pccmodel.geometry import PccGeometry
from continuum_pccmodel.forward_kinematics import forward_kinematics_from_dl


def solve_inverse_kinematics(
    geometry: PccGeometry,
    phase_offsets_rad: List[float],
    target_position_m: Optional[List[float]] = None,
    target_euler_rad: Optional[List[float]] = None,
    num_sections: int = 2,
    dl_lower_m: float = 0.0,
    dl_upper_m: float = 0.12,
    max_iter: int = 200,
    position_weight: float = 1.0,
    orientation_weight: float = 1.0,
) -> Dict[str, Any]:
    """
    求解 PCC 连续体机器人的逆运动学。

    参数：
        geometry:
            PCC 几何参数。

        phase_offsets_rad:
            每段弯曲方向相位偏置，单位 rad。
            例如两段连续体：
                [0.0, 1.0471975512]

        target_position_m:
            目标末端位置，单位 m。
            格式：
                [x, y, z]

        target_euler_rad:
            目标末端姿态，单位 rad。
            格式：
                [yaw, pitch, roll]

        num_sections:
            连续体段数。
            当前你的结构是两段，所以默认 2。

        dl_lower_m:
            每根绳长度变化下限，单位 m。
            第一版默认 0，表示只允许收缩。

        dl_upper_m:
            每根绳长度变化上限，单位 m。
            第一版默认 0.12 m，对应 MATLAB 文件里的 ub = 0.12。

        max_iter:
            优化最大迭代次数。

        position_weight:
            位置误差权重。

        orientation_weight:
            姿态误差权重。

    返回：
        result_dict:
            success:
                优化是否成功。

            dl_sections_m:
                每段三根绳长度变化，单位 m。

            dl_sections_mm:
                每段三根绳长度变化，单位 mm。

            cost:
                最终目标函数值。

            fk:
                用最优 DL 重新计算的正运动学结果。
    """

    # =========================================================
    # 1. 输入检查
    # =========================================================

    if target_position_m is None and target_euler_rad is None:
        raise ValueError("target_position_m 和 target_euler_rad 至少需要提供一个")

    if len(phase_offsets_rad) != num_sections:
        raise ValueError("phase_offsets_rad 的长度必须等于 num_sections")

    if dl_upper_m <= dl_lower_m:
        raise ValueError("dl_upper_m 必须大于 dl_lower_m")

    # 目标位置
    if target_position_m is not None:
        target_position_m = np.array(target_position_m, dtype=float).reshape(3)

    # 目标姿态
    if target_euler_rad is not None:
        target_euler_rad = np.array(target_euler_rad, dtype=float).reshape(3)

    # =========================================================
    # 2. 定义优化变量
    # =========================================================

    # 两段连续体，每段三根绳：
    # x = [dl11, dl12, dl13, dl21, dl22, dl23]
    variable_count = num_sections * 3

    # 初始猜测值。
    # 第一版从 0 开始，表示所有绳都不收缩。
    x0 = np.zeros(variable_count, dtype=float)

    # 每根绳 DL 的上下限。
    bounds = [
        (float(dl_lower_m), float(dl_upper_m))
        for _ in range(variable_count)
    ]

    # =========================================================
    # 3. 定义目标函数
    # =========================================================

    def objective(x):
        """
        优化目标函数。

        输入：
            x:
                展平后的 DL。

        输出：
            cost:
                当前 DL 对应的末端位姿误差。
        """

        # 将一维变量转成 num_sections × 3
        dl_sections_m = np.array(x, dtype=float).reshape(num_sections, 3).tolist()

        # 调用正运动学
        fk = forward_kinematics_from_dl(
            dl_sections_m=dl_sections_m,
            geometry=geometry,
            phase_offsets_rad=phase_offsets_rad,
        )

        cost = 0.0

        # --------------------------
        # 位置误差
        # --------------------------
        if target_position_m is not None:
            current_position = np.array(
                [
                    fk["end_px"],
                    fk["end_py"],
                    fk["end_pz"],
                ],
                dtype=float,
            )

            position_error = current_position - target_position_m
            cost += float(position_weight * (position_error @ position_error))

        # --------------------------
        # 姿态误差
        # --------------------------
        if target_euler_rad is not None:
            current_euler = np.array(
                [
                    fk["end_yaw"],
                    fk["end_pitch"],
                    fk["end_roll"],
                ],
                dtype=float,
            )

            euler_error = current_euler - target_euler_rad
            cost += float(orientation_weight * (euler_error @ euler_error))

        return cost

    # =========================================================
    # 4. 调用优化器
    # =========================================================

    opt_result = minimize(
        objective,
        x0,
        method="SLSQP",
        bounds=bounds,
        options={
            "maxiter": int(max_iter),
            "ftol": 1e-9,
            "disp": False,
        },
    )

    # =========================================================
    # 5. 整理输出结果
    # =========================================================

    dl_sections_m = np.array(opt_result.x, dtype=float).reshape(num_sections, 3)
    dl_sections_mm = dl_sections_m * 1000.0

    fk_best = forward_kinematics_from_dl(
        dl_sections_m=dl_sections_m.tolist(),
        geometry=geometry,
        phase_offsets_rad=phase_offsets_rad,
    )

    result_dict = {
        "success": bool(opt_result.success),
        "message": str(opt_result.message),
        "cost": float(opt_result.fun),

        "dl_sections_m": dl_sections_m,
        "dl_sections_mm": dl_sections_mm,

        "fk": fk_best,
    }

    return result_dict
