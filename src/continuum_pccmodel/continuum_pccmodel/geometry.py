"""
geometry.py

PCC 连续体机器人的几何参数管理。

主要作用：
1. 从 robot_calibration.yaml 读取全局 PCC 参数；
2. 从顶层 sections 读取每一段真实安装尺寸；
3. 根据“盘厚度 × 计入盘数量 + 实测盘间隙总和”计算每段 PCC 初始线缆长度；
4. 从 pcc_model.sections 读取 PCC 内部 DL 槽位对应的 tendon_ids、dl_signs、phase_offset_rad。

单位约定：
- YAML 中真实结构尺寸通常用 mm；
- 进入 PCC 模型后统一转换为 m；
- 角度统一使用 rad。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple


@dataclass
class SectionGeometry:
    """
    单段连续体真实安装几何参数。

    核心长度定义：
        L0 = disk_thickness_m * disk_count_for_tendon_length
             + sum(disk_gap_lengths_m)

    其中：
    - disk_count_for_tendon_length：该段线缆长度计算中计入的盘数量；
    - disk_gap_lengths_m：该段实测的每个盘间净间隙；
    - L0 会作为 PCC 计算中的该段初始线缆长度/初始中心线长度。
    """

    name: str

    disk_count_for_tendon_length: int
    gap_count: int
    disk_thickness_m: float
    disk_gap_lengths_m: List[float]

    tendon_radius_m: float
    hexagon_inscribed_radius_m: float = 0.0

    # PCC 内部建模参数：来自 pcc_model.sections
    model_tendon_ids: List[int] = field(default_factory=list)
    dl_signs: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    phase_offset_rad: float = 0.0

    @property
    def initial_tendon_length_m(self) -> float:
        """
        该段 PCC 计算使用的初始线缆长度 L0。

        L0 = 盘厚度 × 计入盘数量 + 盘间净间隙总和。
        """
        return float(
            self.disk_thickness_m * self.disk_count_for_tendon_length
            + sum(self.disk_gap_lengths_m)
        )

    @property
    def section_length_m(self) -> float:
        """
        兼容名称。

        在当前 PCC 假设下，该段初始中心线长度直接等于初始线缆长度。
        """
        return self.initial_tendon_length_m

    @property
    def equivalent_gap_m(self) -> float:
        """该段平均盘间净间隙，仅用于日志和检查。"""
        if self.gap_count <= 0:
            return 0.0
        return float(sum(self.disk_gap_lengths_m) / self.gap_count)


@dataclass
class PccGeometry:
    """
    PCC 连续体模型全局参数。

    注意：
    - 新 PCC 正运动学应优先使用 geometry.sections[section_name].section_length_m；
    - geometry.section_length_m 仅保留给旧代码兜底兼容。
    """

    hex_radius_m: float = 0.0810
    disk_thickness_m: float = 0.010
    disk_gap_m: float = 0.030
    disk_pitch_m: float = 0.040
    intervals_per_section: int = 5

    tendon_geometry_d_m: float = 0.055
    equal_dl_tol_m: float = 1.0e-7

    chain_order: List[str] = field(default_factory=lambda: ["section2", "section1"])
    sections: Dict[str, SectionGeometry] = field(default_factory=dict)

    @property
    def section_length_m(self) -> float:
        """
        旧代码兼容用默认段长。

        新代码不要优先使用这个，而要使用：
            geometry.sections["section1"].section_length_m
            geometry.sections["section2"].section_length_m
        """
        return float(self.intervals_per_section * self.disk_pitch_m)


def _as_float_list(values) -> List[float]:
    return [float(x) for x in values]


def _as_int_list(values) -> List[int]:
    return [int(x) for x in values]


def _load_one_section_geometry(
    cfg: Dict,
    section_name: str,
    pcc_cfg: Dict,
) -> SectionGeometry:
    """
    从 robot_calibration.yaml 中读取某一段真实安装尺寸。

    优先使用顶层 sections.sectionX.disk_gap_lengths_mm。
    如果没有，则兼容 disk_gap_mm 或 segment_lengths_mm。

    核心计算：
        L0 = disk_thickness * disk_count + sum(gap_lengths)
    """

    sections_cfg = cfg.get("sections", {})
    sec_cfg = sections_cfg.get(section_name, {})

    pcc_sections_cfg = pcc_cfg.get("sections", {})
    pcc_sec_cfg = pcc_sections_cfg.get(section_name, {})

    # 盘厚度：优先用顶层 sections 的实测值，单位 mm -> m
    disk_thickness_m = float(
        sec_cfg.get(
            "disk_thickness_mm",
            pcc_cfg.get("disk_thickness_m", 0.010) * 1000.0,
        )
    ) / 1000.0

    # 线缆长度计算中计入的盘数量
    disk_count = int(
        sec_cfg.get(
            "disk_count_for_tendon_length",
            sec_cfg.get("moving_disk_count", 5),
        )
    )

    # 盘间净间隙数量
    gap_count = int(
        sec_cfg.get(
            "gap_count",
            sec_cfg.get("segment_count", pcc_cfg.get("intervals_per_section", 5)),
        )
    )

    # 盘间净间隙：优先读取实测列表
    if "disk_gap_lengths_mm" in sec_cfg:
        disk_gap_lengths_m = [
            float(x) / 1000.0 for x in sec_cfg["disk_gap_lengths_mm"]
        ]

    elif "disk_gap_mm" in sec_cfg:
        gap_m = float(sec_cfg["disk_gap_mm"]) / 1000.0
        disk_gap_lengths_m = [gap_m for _ in range(gap_count)]

    elif "segment_lengths_mm" in sec_cfg:
        # 兼容旧写法：
        # 如果 segment_lengths_mm 表示“盘厚 + 盘间净间隙”，
        # 则 gap_i = segment_i - disk_thickness。
        disk_gap_lengths_m = []
        for x in sec_cfg["segment_lengths_mm"]:
            segment_m = float(x) / 1000.0
            gap_m = segment_m - disk_thickness_m
            if gap_m < 0.0:
                raise ValueError(
                    f"{section_name}: segment length {segment_m:.6f} m "
                    f"is smaller than disk thickness {disk_thickness_m:.6f} m"
                )
            disk_gap_lengths_m.append(gap_m)

    else:
        # 兜底：使用 pcc_model.disk_gap_m
        gap_m = float(pcc_cfg.get("disk_gap_m", 0.030))
        disk_gap_lengths_m = [gap_m for _ in range(gap_count)]

    if len(disk_gap_lengths_m) != gap_count:
        raise ValueError(
            f"{section_name}: disk_gap_lengths length "
            f"{len(disk_gap_lengths_m)} != gap_count {gap_count}"
        )

    # 线缆到中心轴距离：优先使用每段实测 tendon_radius_mm
    tendon_radius_m = float(
        sec_cfg.get(
            "tendon_radius_mm",
            pcc_cfg.get("tendon_geometry_d_m", 0.055) * 1000.0,
        )
    ) / 1000.0

    hexagon_inscribed_radius_m = float(
        sec_cfg.get("hexagon_inscribed_radius_mm", 0.0)
    ) / 1000.0

    # PCC 内部三根等效绳顺序、符号、相位偏置
    default_tendon_ids = sec_cfg.get("tendon_ids", [])
    model_tendon_ids = pcc_sec_cfg.get("tendon_ids", default_tendon_ids)
    model_tendon_ids = _as_int_list(model_tendon_ids)

    dl_signs = pcc_sec_cfg.get("dl_signs", [1.0, 1.0, 1.0])
    dl_signs = _as_float_list(dl_signs)

    phase_offset_rad = float(pcc_sec_cfg.get("phase_offset_rad", 0.0))

    if len(model_tendon_ids) not in (0, 3):
        raise ValueError(
            f"{section_name}: pcc_model.sections.{section_name}.tendon_ids "
            f"must contain 3 ids, got {model_tendon_ids}"
        )

    if len(dl_signs) != 3:
        raise ValueError(
            f"{section_name}: dl_signs must contain 3 values, got {dl_signs}"
        )

    return SectionGeometry(
        name=section_name,
        disk_count_for_tendon_length=disk_count,
        gap_count=gap_count,
        disk_thickness_m=disk_thickness_m,
        disk_gap_lengths_m=disk_gap_lengths_m,
        tendon_radius_m=tendon_radius_m,
        hexagon_inscribed_radius_m=hexagon_inscribed_radius_m,
        model_tendon_ids=model_tendon_ids,
        dl_signs=dl_signs,
        phase_offset_rad=phase_offset_rad,
    )


def load_pcc_geometry_from_cfg(cfg: Dict) -> PccGeometry:
    """
    从 robot_calibration.yaml 读取 PCC 几何参数。

    全局参数从 pcc_model 读取；
    每段真实线缆初始长度从顶层 sections 读取；
    每段 PCC DL 槽位映射从 pcc_model.sections 读取。
    """

    pcc_cfg = cfg.get("pcc_model", {})

    chain_order = [
        str(x) for x in pcc_cfg.get("chain_order", ["section2", "section1"])
    ]

    geometry = PccGeometry(
        hex_radius_m=float(pcc_cfg.get("hex_radius_m", 0.0810)),
        disk_thickness_m=float(pcc_cfg.get("disk_thickness_m", 0.010)),
        disk_gap_m=float(pcc_cfg.get("disk_gap_m", 0.030)),
        disk_pitch_m=float(pcc_cfg.get("disk_pitch_m", 0.040)),
        intervals_per_section=int(pcc_cfg.get("intervals_per_section", 5)),
        tendon_geometry_d_m=float(pcc_cfg.get("tendon_geometry_d_m", 0.055)),
        equal_dl_tol_m=float(pcc_cfg.get("equal_dl_tol_m", 1e-7)),
        chain_order=chain_order,
    )

    # 保证常用两段和 chain_order 中的段都会被加载
    section_names = []
    for name in ["section1", "section2", *chain_order]:
        if name not in section_names:
            section_names.append(name)

    geometry.sections = {
        name: _load_one_section_geometry(cfg, name, pcc_cfg)
        for name in section_names
    }

    return geometry


def get_section_model_cfg(
    cfg: Dict,
    section_name: str,
    default_tendon_ids: List[int],
) -> Tuple[List[int], List[float], float]:
    """
    读取某一段连续体的 PCC 建模配置。

    注意：
    这里读取的是 pcc_model.sections，而不是顶层 sections。
    顶层 sections 描述真实安装尺寸；
    pcc_model.sections 描述 PCC 内部 DL1/DL2/DL3 对应哪几根真实绳。
    """

    pcc_cfg = cfg.get("pcc_model", {})
    sections_cfg = pcc_cfg.get("sections", {})
    sec_cfg = sections_cfg.get(section_name, {})

    tendon_ids = sec_cfg.get("tendon_ids", default_tendon_ids)
    tendon_ids = [int(x) for x in tendon_ids]

    dl_signs = sec_cfg.get("dl_signs", [1.0, 1.0, 1.0])
    dl_signs = [float(x) for x in dl_signs]

    phase_offset_rad = float(sec_cfg.get("phase_offset_rad", 0.0))

    return tendon_ids, dl_signs, phase_offset_rad
