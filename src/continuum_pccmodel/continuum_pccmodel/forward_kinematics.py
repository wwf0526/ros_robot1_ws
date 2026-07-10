"""
forward_kinematics.py

PCC 连续体机器人正运动学模型。

核心作用：
1. 输入每段三根等效驱动绳的长度变化 DL；
2. 使用每段真实安装后的初始线缆长度 L0；
3. 计算每段弯曲角 theta、弯曲方向 phi、曲率 kappa；
4. 计算每段齐次变换矩阵 T；
5. 多段串联得到末端位姿 Tend。

单位约定：
- 模型内部长度统一用 m；
- 模型内部角度统一用 rad；
- 如果 ROS 中输入是 mm，使用 forward_kinematics_from_dl_mm() 自动转换。
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

import numpy as np

from continuum_pccmodel.geometry import PccGeometry
from continuum_pccmodel.transforms import make_translation_z, rotm_to_euler_zyx


def _make_pcc_transform(kappa: float, theta: float, phi: float, arc_length: float) -> np.ndarray:
    """
    根据 PCC 参数构造单段齐次变换矩阵。

    姿态使用：
        R = Rz(phi) * Ry(theta) * Rz(-phi)

    好处：
        theta = 0 时，R = I，不会因为 phi 或 phase_offset 产生虚假 yaw。
    """

    if abs(kappa) < 1e-12 or abs(theta) < 1e-12:
        return make_translation_z(arc_length)

    cphi = math.cos(phi)
    sphi = math.sin(phi)
    ctheta = math.cos(theta)
    stheta = math.sin(theta)

    return np.array(
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
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def section_transform_from_dl(
    dl_m: List[float],
    geometry: PccGeometry,
    phase_offset_rad: float = 0.0,
    section_name: Optional[str] = None,
):
    """
    根据单段连续体三根绳长变化计算该段 PCC 变换矩阵。

    参数：
        dl_m:
            当前段三根等效驱动绳的长度变化，单位 m。
            约定 dl > 0 表示收绳，dl < 0 表示放绳。

        geometry:
            PCC 几何参数，由 robot_calibration.yaml 读取。

        phase_offset_rad:
            当前段弯曲方向相位偏置，单位 rad。
            该参数保留用于兼容旧调用；如果没有显式传入，
            也可由 geometry.sections[section_name].phase_offset_rad 提供。

        section_name:
            段名称，例如 "section2" 或 "section1"。
            新代码应传入该参数，以便使用每段真实安装尺寸。

    返回：
        T:
            当前段 4x4 齐次变换矩阵。

        info:
            当前段 PCC 中间变量。
    """

    if len(dl_m) != 3:
        raise ValueError("单段 PCC 模型必须输入三根等效驱动绳的 DL")

    dl1, dl2, dl3 = [float(x) for x in dl_m]

    # =========================================================
    # 1. 读取每段真实安装尺寸
    # =========================================================
    section_geometry = None
    if section_name is not None:
        section_geometry = geometry.sections.get(section_name)
        if section_geometry is None:
            raise KeyError(f"geometry.sections 中找不到 {section_name}")

    if section_geometry is not None:
        # 新逻辑：
        # L0 = 盘厚度 × 计入盘数量 + 实测盘间净间隙总和
        section_length = float(section_geometry.initial_tendon_length_m)
        tendon_d = float(section_geometry.tendon_radius_m)

        # 如果调用者没有显式给 phase_offset，则使用该段配置中的 phase_offset。
        # 注意：为了兼容旧接口，只有 phase_offset_rad 仍为默认 0.0 时才自动覆盖。
        if abs(float(phase_offset_rad)) < 1e-15:
            phase_offset_rad = float(section_geometry.phase_offset_rad)
    else:
        # 旧逻辑兜底：
        section_length = float(geometry.section_length_m)
        tendon_d = float(geometry.tendon_geometry_d_m)

    if section_length <= 0.0:
        raise ValueError("section_length 必须大于 0")

    if tendon_d <= 0.0:
        raise ValueError("tendon_d 必须大于 0")

    # =========================================================
    # 2. 计算三根线缆当前实际长度
    # =========================================================
    # L0 为该段真实装配后的初始线缆长度。
    # dl > 0 表示收绳，所以当前长度 = L0 - dl。
    l1 = section_length - dl1
    l2 = section_length - dl2
    l3 = section_length - dl3

    if l1 <= 0.0 or l2 <= 0.0 or l3 <= 0.0:
        raise ValueError(
            f"{section_name or 'section'} 计算后的绳长出现非正值："
            f"l1={l1:.6f}, l2={l2:.6f}, l3={l3:.6f}。"
            "请检查 DL 单位、真实初始长度或电机收绳符号。"
        )

    center_length = (l1 + l2 + l3) / 3.0
    l_sum = l1 + l2 + l3

    # =========================================================
    # 3. 判断是否为纯轴向伸缩
    # =========================================================
    tol = max(float(geometry.equal_dl_tol_m), 1e-12)
    dl_spread = max(dl1, dl2, dl3) - min(dl1, dl2, dl3)

    if dl_spread <= tol:
        T = make_translation_z(center_length)
        info = {
            "section_name": section_name,
            "L0": float(section_length),
            "l1": float(l1),
            "l2": float(l2),
            "l3": float(l3),
            "theta": 0.0,
            "phi": float(phase_offset_rad),
            "kappa": 0.0,
            "arc_length": float(center_length),
            "center_length": float(center_length),
            "tendon_d": float(tendon_d),
        }
        return T, info

    # =========================================================
    # 4. 计算 PCC 曲率和弯曲方向
    # =========================================================
    nn = (
        l1 * l1
        + l2 * l2
        + l3 * l3
        - l1 * l2
        - l1 * l3
        - l2 * l3
    )
    nnn = math.sqrt(max(nn, 0.0))

    if nnn < 1e-12:
        T = make_translation_z(center_length)
        info = {
            "section_name": section_name,
            "L0": float(section_length),
            "l1": float(l1),
            "l2": float(l2),
            "l3": float(l3),
            "theta": 0.0,
            "phi": float(phase_offset_rad),
            "kappa": 0.0,
            "arc_length": float(center_length),
            "center_length": float(center_length),
            "tendon_d": float(tendon_d),
        }
        return T, info

    numerator = math.sqrt(3.0) * (l2 + l3 - 2.0 * l1)
    denominator = 3.0 * (l2 - l3)
    phi = math.atan2(numerator, denominator) + float(phase_offset_rad)

    # Webster/Jones 三线缆 PCC 公式：
    # kappa = 2*sqrt(nn) / (d*(l1+l2+l3))
    kappa = 2.0 * nnn / (tendon_d * l_sum)

    # 关键修改：
    # 使用三根线缆平均长度作为当前中心线弧长，
    # 不再使用原 MATLAB 中带 interval_count 的 lq 公式，
    # 避免真实总长度接入后出现弧长被重复缩放。
    arc_length = center_length
    theta = kappa * arc_length

    T = _make_pcc_transform(kappa=kappa, theta=theta, phi=phi, arc_length=arc_length)

    info = {
        "section_name": section_name,
        "L0": float(section_length),
        "l1": float(l1),
        "l2": float(l2),
        "l3": float(l3),
        "theta": float(theta),
        "phi": float(phi),
        "kappa": float(kappa),
        "arc_length": float(arc_length),
        "center_length": float(center_length),
        "tendon_d": float(tendon_d),
    }

    return T, info


def forward_kinematics_from_dl(
    dl_sections_m: List[List[float]],
    geometry: PccGeometry,
    phase_offsets_rad: Optional[List[float]] = None,
    section_names: Optional[List[str]] = None,
) -> Dict[str, Any]:
    """
    多段连续体 PCC 正运动学。

    参数：
        dl_sections_m:
            每一段的三根绳长变化，单位 m。
            顺序应与 section_names 对应。
            如果 section_names 为空，则默认使用 geometry.chain_order。

        geometry:
            PCC 几何参数。

        phase_offsets_rad:
            每段弯曲方向相位偏置，单位 rad。
            可选；如果不传，则从 geometry.sections[section_name].phase_offset_rad 读取。

        section_names:
            每段名称，例如 ["section2", "section1"]。
            可选；如果不传，则使用 geometry.chain_order。

    返回：
        result:
            包含每段位姿和整体末端位姿。
    """

    if len(dl_sections_m) == 0:
        raise ValueError("dl_sections_m 不能为空")

    if section_names is None:
        section_names = list(geometry.chain_order[: len(dl_sections_m)])

    if len(section_names) != len(dl_sections_m):
        raise ValueError("section_names 和 dl_sections_m 的段数必须一致")

    if phase_offsets_rad is None:
        phase_offsets_rad = []
        for name in section_names:
            sec = geometry.sections.get(name)
            phase_offsets_rad.append(float(sec.phase_offset_rad) if sec else 0.0)

    if len(dl_sections_m) != len(phase_offsets_rad):
        raise ValueError("dl_sections_m 和 phase_offsets_rad 的段数必须一致")

    for dl in dl_sections_m:
        if len(dl) != 3:
            raise ValueError("每一段必须包含三根等效驱动绳")

    Tend = np.eye(4, dtype=float)
    section_poses = []
    section_infos = []

    for i, dl_m in enumerate(dl_sections_m):
        section_name = section_names[i]
        phase_offset = float(phase_offsets_rad[i])

        T_i, info_i = section_transform_from_dl(
            dl_m=dl_m,
            geometry=geometry,
            phase_offset_rad=phase_offset,
            section_name=section_name,
        )

        Tend = Tend @ T_i

        R = Tend[:3, :3]
        p = Tend[:3, 3]
        yaw, pitch, roll = rotm_to_euler_zyx(R)

        section_info = {
            "section_index": i + 1,
            "section_name": section_name,
            "T": Tend.copy(),
            "px": float(p[0]),
            "py": float(p[1]),
            "pz": float(p[2]),
            "yaw": float(yaw),
            "pitch": float(pitch),
            "roll": float(roll),
            "theta": float(info_i["theta"]),
            "phi": float(info_i["phi"]),
            "kappa": float(info_i["kappa"]),
            "arc_length": float(info_i["arc_length"]),
            "center_length": float(info_i["center_length"]),
            "L0": float(info_i["L0"]),
            "l1": float(info_i["l1"]),
            "l2": float(info_i["l2"]),
            "l3": float(info_i["l3"]),
            "tendon_d": float(info_i["tendon_d"]),
        }

        section_poses.append(section_info)
        section_infos.append(info_i)

    end_R = Tend[:3, :3]
    end_p = Tend[:3, 3]
    end_yaw, end_pitch, end_roll = rotm_to_euler_zyx(end_R)

    return {
        "section_names": section_names,
        "section_poses": section_poses,
        "section_infos": section_infos,
        "T_end": Tend,
        "end_px": float(end_p[0]),
        "end_py": float(end_p[1]),
        "end_pz": float(end_p[2]),
        "end_yaw": float(end_yaw),
        "end_pitch": float(end_pitch),
        "end_roll": float(end_roll),
    }


def forward_kinematics_from_dl_mm(
    dl_sections_mm: List[List[float]],
    geometry: PccGeometry,
    phase_offsets_rad: Optional[List[float]] = None,
    section_names: Optional[List[str]] = None,
) -> Dict[str, Any]:
    """
    以 mm 为输入单位的正运动学接口。

    ROS 状态估计中的 tendon_length_mm 通常使用 mm；
    PCC 模型内部统一使用 m，所以这里做 mm -> m 转换。
    """

    dl_sections_m = (np.array(dl_sections_mm, dtype=float) / 1000.0).tolist()

    return forward_kinematics_from_dl(
        dl_sections_m=dl_sections_m,
        geometry=geometry,
        phase_offsets_rad=phase_offsets_rad,
        section_names=section_names,
    )
