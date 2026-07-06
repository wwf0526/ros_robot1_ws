"""
geometry.py

这个文件专门负责 PCC 连续体机器人的几何参数管理。

为什么单独放一个文件？
因为后续 MPC / RL 都需要知道连续体构型，例如：
1. 每段有几个盘
2. 每个盘之间的等效长度
3. 绳到中心的距离
4. 每段使用哪几根绳
5. 第二段相对于第一段是否有安装相位偏置

这些参数不应该写死在代码里，
后续统一从 robot_calibration.yaml 中读取。
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class PccGeometry:
    """
    PCC 连续体模型的几何参数。

    单位约定：
    - 长度统一用 m
    - 角度统一用 rad

    注意：
    PCC建模真正使用的是 section_length_m，
    而不是简单的“盘个数 × 某个长度”。
    """

    # 六边形盘外接圆半径，单位 m。
    # 对于正六边形：外接圆半径 = 六边形边长。
    # 主要用于后续可视化，不直接决定弯曲长度。
    hex_radius_m: float = 0.0810

    # 六边形盘厚度，单位 m。
    disk_thickness_m: float = 0.006

    # 相邻两个盘之间的净间距，单位 m。
    disk_gap_m: float = 0.0525

    # 相邻两个盘中心面之间的距离，单位 m。
    # disk_pitch_m = disk_thickness_m + disk_gap_m
    disk_pitch_m: float = 0.0585

    # 每段有效间隔数。
    # 如果一段有6个盘，且包含首端盘和末端盘，则有效间隔数是5。
    intervals_per_section: int = 5

    # 三根等效驱动绳到中心的几何距离，单位 m。
    # 对应 MATLAB 模型中的 d。
    tendon_geometry_d_m: float = 0.050

    # 判断三根绳收缩量是否近似相等的容差。
    equal_dl_tol_m: float = 0

    @property
    def section_length_m(self) -> float:
        """
        每段连续体的有效弯曲长度，单位 m。

        计算方式：
            section_length_m = intervals_per_section × disk_pitch_m

        其中：
            disk_pitch_m = 盘厚度 + 盘间净距
        """

        return float(self.intervals_per_section * self.disk_pitch_m)


def load_pcc_geometry_from_cfg(cfg: Dict) -> PccGeometry:
    """
    从 robot_calibration.yaml 读取 PCC 几何参数。

    参数：
        cfg:
            yaml.safe_load() 读取出来的整个配置字典。

    返回：
        PccGeometry 对象。

    如果 yaml 文件里没有 pcc_model 字段，
    就使用 PccGeometry 中定义的默认值。
    """

    # 从总配置中读取 pcc_model 字段。
    # 如果没有该字段，则返回空字典。
    pcc_cfg = cfg.get("pcc_model", {})

    # 用 yaml 中的值覆盖默认值。
    # 如果某个字段 yaml 中没有写，就使用后面的默认值。
    geometry = PccGeometry(
        hex_radius_m=float(pcc_cfg.get("hex_radius_m", 0.0810)),

        disk_thickness_m=float(pcc_cfg.get("disk_thickness_m", 0.006)),
        disk_gap_m=float(pcc_cfg.get("disk_gap_m", 0.0525)),
        disk_pitch_m=float(pcc_cfg.get("disk_pitch_m", 0.0585)),

        intervals_per_section=int(pcc_cfg.get("intervals_per_section", 5)),

        tendon_geometry_d_m=float(pcc_cfg.get("tendon_geometry_d_m", 0.050)),

        equal_dl_tol_m=float(pcc_cfg.get("equal_dl_tol_m", 1e-7)),
    )

    return geometry


def get_section_model_cfg(
    cfg: Dict,
    section_name: str,
    default_tendon_ids: List[int],
) -> Tuple[List[int], List[float], float]:
    """
    读取某一段连续体的建模配置。

    参数：
        cfg:
            robot_calibration.yaml 读取出来的总配置字典。

        section_name:
            段名称，例如：
            "section1"
            "section2"

        default_tendon_ids:
            如果 yaml 中没有写该段 tendon_ids，
            就使用状态估计节点原来的分段绳索编号。

    返回：
        tendon_ids:
            当前段参与建模的三根等效绳编号。

        dl_signs:
            每根绳的长度变化符号。
            如果后续发现某根绳方向反了，只需要在 yaml 中把 1.0 改成 -1.0。

        phase_offset_rad:
            当前段的弯曲方向相位偏置，单位 rad。
            MATLAB 中第二段 phiq + pi/3，
            后续就在 yaml 中设置为 1.0471975512。
    """

    # 读取 pcc_model 配置
    pcc_cfg = cfg.get("pcc_model", {})

    # 读取 sections 配置
    sections_cfg = pcc_cfg.get("sections", {})

    # 读取具体某一段配置，例如 section1 或 section2
    sec_cfg = sections_cfg.get(section_name, {})

    # 读取该段使用的三根绳编号
    tendon_ids = sec_cfg.get("tendon_ids", default_tendon_ids)
    tendon_ids = [int(x) for x in tendon_ids]

    # 读取该段三根绳的符号修正
    dl_signs = sec_cfg.get("dl_signs", [1.0, 1.0, 1.0])
    dl_signs = [float(x) for x in dl_signs]

    # 读取该段相位偏置
    phase_offset_rad = float(sec_cfg.get("phase_offset_rad", 0.0))

    return tendon_ids, dl_signs, phase_offset_rad
