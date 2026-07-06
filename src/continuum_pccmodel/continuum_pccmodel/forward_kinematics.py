"""
forward_kinematics.py

PCC 连续体机器人正运动学模型。

核心作用：
1. 输入每段三根等效驱动绳的长度变化 DL；
2. 计算每段弯曲角 theta；
3. 计算每段弯曲方向 phi；
4. 计算每段曲率 kappa；
5. 计算每段齐次变换矩阵 T；
6. 多段连续体串联得到末端位姿 Tend。

单位约定：
- 模型内部长度统一用 m；
- 模型内部角度统一用 rad；
- 如果 ROS 中输入是 mm，使用 forward_kinematics_from_dl_mm() 自动转换。

说明：
- kappa 曲率只是中间变量；
- 后续 MPC / RL 更推荐使用 roll、pitch、yaw、px、py、pz 作为状态。
"""

import math
from typing import Any, Dict, List

import numpy as np

from continuum_pccmodel.geometry import PccGeometry
from continuum_pccmodel.transforms import make_translation_z, rotm_to_euler_zyx


def section_transform_from_dl(
    dl_m: List[float],
    geometry: PccGeometry,
    phase_offset_rad: float = 0.0,
):
    """
    根据单段连续体的三根绳长变化，计算该段 PCC 变换矩阵。

    参数：
        dl_m:
            当前段三根等效驱动绳的长度变化，单位 m。
            约定：
                dl > 0 表示该根绳收缩；
                dl < 0 表示该根绳放长。

        geometry:
            PCC 几何参数，从 robot_calibration.yaml 中读取。

        phase_offset_rad:
            当前段弯曲方向的相位偏置，单位 rad。
            例如：
                第一段 phase_offset_rad = 0；
                第二段 phase_offset_rad = pi / 3。

    返回：
        T:
            当前段 4x4 齐次变换矩阵。

        info:
            当前段中间变量，包括 theta、phi、kappa、arc_length 等。
    """

    # =========================================================
    # 1. 读取三根绳长变化
    # =========================================================
    if len(dl_m) != 3:
        raise ValueError("单段 PCC 模型必须输入三根等效驱动绳的 DL")

    dl1 = float(dl_m[0])
    dl2 = float(dl_m[1])
    dl3 = float(dl_m[2])

    # =========================================================
    # 2. 读取几何参数
    # =========================================================

    # 每段有效弯曲长度：
    # section_length_m = intervals_per_section × disk_pitch_m
    section_length = float(geometry.section_length_m)

    # 三根等效驱动绳到中心的几何距离。
    # 对应原 MATLAB 模型中的 d。
    tendon_d = float(geometry.tendon_geometry_d_m)

    # 有效间隔数。
    # 对应原 MATLAB 模型中的 n。
    interval_count = float(geometry.intervals_per_section)

    if section_length <= 0.0:
        raise ValueError("section_length_m 必须大于 0")

    if tendon_d <= 0.0:
        raise ValueError("tendon_geometry_d_m 必须大于 0")

    if interval_count <= 0.0:
        raise ValueError("intervals_per_section 必须大于 0")

    # =========================================================
    # 3. 计算三根绳变形后的实际长度
    # =========================================================

    # 原 MATLAB 中：
    # l1 = nL - DL1
    # l2 = nL - DL2
    # l3 = nL - DL3
    #
    # 这里用 section_length 替代 nL。
    l1 = section_length - dl1
    l2 = section_length - dl2
    l3 = section_length - dl3

    if l1 <= 0.0 or l2 <= 0.0 or l3 <= 0.0:
        raise ValueError(
            "计算后的绳长出现非正值，请检查 DL 单位、section_length_m 或几何参数"
        )

    # 三根绳平均长度，可近似理解为中心线长度。
    center_length = (l1 + l2 + l3) / 3.0

    # =========================================================
    # 4. 判断是否为纯轴向伸缩
    # =========================================================

    # 如果三根绳变化量几乎相同：
    # 该段不弯曲，只沿 z 方向产生轴向长度变化。
        # 即使 IK 测试时 equal_dl_tol_m 设置为 0，
    # 三根 DL 完全相等时也必须进入纯平移分支。
    # 否则零弯曲会错误进入 PCC 弧长公式，导致长度变成一半。
    tol = max(float(geometry.equal_dl_tol_m), 1e-12)

    dl_spread = max(dl1, dl2, dl3) - min(dl1, dl2, dl3)

    same_dl = dl_spread <= tol

    if same_dl:
        T = make_translation_z(center_length)

        info = {
            "theta": 0.0,
            "phi": float(phase_offset_rad),
            "kappa": 0.0,
            "arc_length": float(center_length),
            "center_length": float(center_length),
        }

        return T, info

    # =========================================================
    # 5. 计算 PCC 中间变量
    # =========================================================

    # l_sum 对应 MATLAB 中的 lll。
    l_sum = l1 + l2 + l3

    # nn 对应 MATLAB 中：
    # nn = l1^2 + l2^2 + l3^2 - l1*l2 - l1*l3 - l2*l3
    nn = (
        l1 * l1
        + l2 * l2
        + l3 * l3
        - l1 * l2
        - l1 * l3
        - l2 * l3
    )

    # 防止浮点误差导致 sqrt 负数。
    nnn = math.sqrt(max(nn, 0.0))

    # 如果 nnn 很小，说明弯曲极弱，按纯平移处理。
    if nnn < 1e-9:
        T = make_translation_z(center_length)

        info = {
            "theta": 0.0,
            "phi": float(phase_offset_rad),
            "kappa": 0.0,
            "arc_length": float(center_length),
            "center_length": float(center_length),
        }

        return T, info

    # =========================================================
    # 6. 计算弧长 arc_length
    # =========================================================

    # 原 MATLAB：
    # lq = n*d*lll*asin(nnn/(3*n*d))/(2*nnn)
    #
    # 这里：
    # n = interval_count
    # d = tendon_d
    ratio = nnn / (3.0 * interval_count * tendon_d)

    # asin 输入必须在 [-1, 1]。
    ratio = max(-1.0, min(1.0, ratio))

    arc_length = (
        interval_count
        * tendon_d
        * l_sum
        * math.asin(ratio)
        / (2.0 * nnn)
    )

    # =========================================================
    # 7. 计算弯曲方向 phi
    # =========================================================

    # 原 MATLAB：
    # phiq = atan( sqrt(3)*(l2+l3-2*l1) / (3*(l2-l3)) )
    #
    # 这里使用 atan2，更稳定。
    numerator = math.sqrt(3.0) * (l2 + l3 - 2.0 * l1)
    denominator = 3.0 * (l2 - l3)

    phi = math.atan2(numerator, denominator)

    # 加上段间相位偏置。
    phi = phi + float(phase_offset_rad)

    # =========================================================
    # 8. 计算曲率 kappa
    # =========================================================

    # 原 MATLAB：
    # kq = 2*nnn/(d*lll)
    kappa = 2.0 * nnn / (tendon_d * l_sum)

    # =========================================================
    # 9. 计算弯曲角 theta
    # =========================================================

    # 原 MATLAB：
    # thta = lq * kq
    theta = arc_length * kappa

    # =========================================================
    # 10. 构造单段 PCC 齐次变换矩阵
    # =========================================================

    if abs(kappa) < 1e-12:
        T = make_translation_z(arc_length)
    else:
        cphi = math.cos(phi)
        sphi = math.sin(phi)

        ctheta = math.cos(theta)
        stheta = math.sin(theta)

        # 标准 PCC 常曲率齐次变换矩阵。
        #
        # 使用形式：
        #     R = Rz(phi) * Ry(theta) * Rz(-phi)
        #
        # 这样做的关键好处：
        #     当 theta = 0 时，R = I
        #
        # 也就是说：
        #     不弯曲时，连续体不会因为 phi 或 phase_offset 产生额外 yaw。
        #
        # 这比直接使用 MATLAB 中原始矩阵更适合后续 MPC / RL 中的姿态控制。

        T = np.array(
            [
                [
                    cphi * cphi * ctheta + sphi * sphi,
                    cphi * sphi * (ctheta - 1.0),
                    cphi * stheta,
                    cphi * (1.0 - ctheta) / kappa,
                ],
                [
                    cphi * sphi * (ctheta - 1.0),
                    sphi * sphi * ctheta + cphi * cphi,
                    sphi * stheta,
                    sphi * (1.0 - ctheta) / kappa,
                ],
                [
                    -cphi * stheta,
                    -sphi * stheta,
                    ctheta,
                    stheta / kappa,
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ],
            ],
            dtype=float,
        )

    # =========================================================
    # 11. 保存中间变量
    # =========================================================

    info = {
        "theta": float(theta),
        "phi": float(phi),
        "kappa": float(kappa),
        "arc_length": float(arc_length),
        "center_length": float(center_length),
    }

    return T, info


def forward_kinematics_from_dl(
    dl_sections_m: List[List[float]],
    geometry: PccGeometry,
    phase_offsets_rad: List[float],
) -> Dict[str, Any]:
    """
    多段连续体 PCC 正运动学。

    参数：
        dl_sections_m:
            每一段的三根绳长变化，单位 m。

            两段连续体示例：
            [
                [dl11, dl12, dl13],
                [dl21, dl22, dl23],
            ]

        geometry:
            PCC 几何参数。

        phase_offsets_rad:
            每段弯曲方向相位偏置，单位 rad。

    返回：
        result:
            包含每段位姿和整体末端位姿。
    """

    # =========================================================
    # 1. 输入检查
    # =========================================================

    if len(dl_sections_m) == 0:
        raise ValueError("dl_sections_m 不能为空")

    if len(dl_sections_m) != len(phase_offsets_rad):
        raise ValueError("dl_sections_m 和 phase_offsets_rad 的段数必须一致")

    for dl in dl_sections_m:
        if len(dl) != 3:
            raise ValueError("每一段必须包含三根等效驱动绳")

    # =========================================================
    # 2. 初始化总变换矩阵
    # =========================================================

    # 初始总变换是单位矩阵。
    Tend = np.eye(4, dtype=float)

    # 用于保存每一段末端位姿。
    section_poses = []

    # =========================================================
    # 3. 逐段计算并累乘
    # =========================================================

    for i, dl_m in enumerate(dl_sections_m):
        phase_offset = float(phase_offsets_rad[i])

        # 计算当前段局部变换。
        T_i, info_i = section_transform_from_dl(
            dl_m=dl_m,
            geometry=geometry,
            phase_offset_rad=phase_offset,
        )

        # 串联结构：
        # 总变换 = 上一段总变换 × 当前段局部变换。
        Tend = Tend @ T_i

        # 提取当前段末端位置和姿态。
        R = Tend[:3, :3]
        p = Tend[:3, 3]

        yaw, pitch, roll = rotm_to_euler_zyx(R)

        section_poses.append(
            {
                "section_index": i + 1,

                # 当前段末端齐次变换矩阵。
                "T": Tend.copy(),

                # 当前段末端位置，单位 m。
                "px": float(p[0]),
                "py": float(p[1]),
                "pz": float(p[2]),

                # 当前段末端姿态，单位 rad。
                "yaw": float(yaw),
                "pitch": float(pitch),
                "roll": float(roll),

                # 当前段 PCC 中间变量。
                "theta": float(info_i["theta"]),
                "phi": float(info_i["phi"]),
                "kappa": float(info_i["kappa"]),
                "arc_length": float(info_i["arc_length"]),
            }
        )

    # =========================================================
    # 4. 提取整体末端位姿
    # =========================================================

    end_R = Tend[:3, :3]
    end_p = Tend[:3, 3]

    end_yaw, end_pitch, end_roll = rotm_to_euler_zyx(end_R)

    result = {
        "section_poses": section_poses,
        "T_end": Tend,

        # 末端位置，单位 m。
        "end_px": float(end_p[0]),
        "end_py": float(end_p[1]),
        "end_pz": float(end_p[2]),

        # 末端姿态，单位 rad。
        "end_yaw": float(end_yaw),
        "end_pitch": float(end_pitch),
        "end_roll": float(end_roll),
    }

    return result


def forward_kinematics_from_dl_mm(
    dl_sections_mm: List[List[float]],
    geometry: PccGeometry,
    phase_offsets_rad: List[float],
) -> Dict[str, Any]:
    """
    以 mm 为输入单位的正运动学接口。

    为什么需要这个函数？
        因为你当前 ROS 状态估计中 tendon_length_mm 使用的是 mm；
        但是 PCC 模型内部统一使用 m；
        所以这里专门做 mm → m 的单位转换。
    """

    # mm 转 m。
    dl_sections_m = (np.array(dl_sections_mm, dtype=float) / 1000.0).tolist()

    return forward_kinematics_from_dl(
        dl_sections_m=dl_sections_m,
        geometry=geometry,
        phase_offsets_rad=phase_offsets_rad,
    )
