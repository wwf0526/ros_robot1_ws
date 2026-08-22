"""
generate_mujoco_model.py

功能：
    根据 robot_calibration.yaml 自动生成 MuJoCo MJCF XML 模型。

为什么需要这个文件？
    之前 MuJoCo XML 中每个盘间单元的长度是手写固定值，
    例如 pos="0 0 0.0585"。

    但真实连续体机器人安装后：
        section1 和 section2 的实际长度可能不同；
        靠近基部的 section2 可能因为受压而更短；
        每个盘间隙也可能不完全相同。

    所以应当从 robot_calibration.yaml 中读取真实尺寸，
    自动生成 MuJoCo XML，使 MuJoCo 显示模型和 PCC 计算模型一致。

核心生成规则：
    每个盘间单元长度 =
        disk_thickness_mm + 当前盘间净间隙 disk_gap_lengths_mm[i]

    例如：
        disk_thickness_mm = 10
        disk_gap_lengths_mm = [16, 17, 18, 19, 20]

    则 MuJoCo 中 section2 的单元长度为：
        [26, 27, 28, 29, 30] mm
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List
from xml.etree import ElementTree as ET

import yaml
import math

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

def add_tendon_markers(
    body,
    prefix,
    radius=0.045
):

    tendon_angles = {
        1: 300,
        2: 240,
        3: 180,
        4: 120,
        5: 60,
        6: 0,
    }


    for tendon_id, angle_deg in tendon_angles.items():

        angle = math.radians(angle_deg)


        x = radius * math.cos(angle)
        y = radius * math.sin(angle)


        ET.SubElement(
            body,
            "site",
            {
                "name": f"{prefix}_tendon_{tendon_id}",
                "pos": f"{x} {y} 0.006",
                "size": "0.006",
                "rgba": "1 0 0 1",
            },
        )
        

def _fmt(value: float) -> str:
    """
    将浮点数格式化成适合写入 XML 的字符串。

    例如：
        0.02600000000001 -> "0.026"
    """
    return f"{float(value):.9g}"


def _list_to_str(values: List[float]) -> str:
    """
    将数字列表转换成 MuJoCo XML 中常用的空格分隔形式。

    例如：
        [0, 0, 0.026] -> "0 0 0.026"
    """
    return " ".join(_fmt(v) for v in values)

def _add_coordinate_frame(worldbody: ET.Element):
    """
    添加 PCC / 机器人基坐标系显示

    坐标定义：
        X+ : 红色
        Y+ : 绿色
        Z+ : 蓝色

    说明：
        1. 坐标系原点位于连续体基座中心；
        2. 在各轴末端增加 site，名称分别为 X / Y / Z；
        3. MuJoCo viewer 中可开启 site label 查看字母标注。
    """

    # ==========================
    # X+ axis
    # cylinder 默认沿自身局部 z 轴
    # 要让它沿世界 x 轴，需要绕 y 轴旋转 +90°
    # ==========================
    x_body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "pcc_axis_x",
            "pos": "0.06 0 0",
            "quat": "0.70710678 0 0.70710678 0",
        },
    )

    ET.SubElement(
        x_body,
        "geom",
        {
            "name": "pcc_axis_x_geom",
            "type": "cylinder",
            "size": "0.005 0.06",
            "rgba": "1 0 0 1",
        },
    )

    ET.SubElement(
        worldbody,
        "site",
        {
            "name": "X",
            "pos": "0.135 0 0",
            "size": "0.008",
            "rgba": "1 0 0 1",
        },
    )

    # ==========================
    # Y+ axis
    # 沿世界 y 轴，需要绕 x 轴旋转 -90°
    # ==========================
    y_body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "pcc_axis_y",
            "pos": "0 0.06 0",
            "quat": "0.70710678 -0.70710678 0 0",
        },
    )

    ET.SubElement(
        y_body,
        "geom",
        {
            "name": "pcc_axis_y_geom",
            "type": "cylinder",
            "size": "0.005 0.06",
            "rgba": "0 1 0 1",
        },
    )

    ET.SubElement(
        worldbody,
        "site",
        {
            "name": "Y",
            "pos": "0 0.135 0",
            "size": "0.008",
            "rgba": "0 1 0 1",
        },
    )

    # ==========================
    # Z+ axis
    # cylinder 默认就是沿 z 轴
    # ==========================
    z_body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "pcc_axis_z",
            "pos": "0 0 0.075",
        },
    )

    ET.SubElement(
        z_body,
        "geom",
        {
            "name": "pcc_axis_z_geom",
            "type": "cylinder",
            "size": "0.005 0.075",
            "rgba": "0 0 1 1",
        },
    )

    ET.SubElement(
        worldbody,
        "site",
        {
            "name": "Z",
            "pos": "0 0 0.165",
            "size": "0.008",
            "rgba": "0 0 1 1",
        },
    )

def _get_section_segment_lengths_m(cfg: Dict, section_name: str) -> List[float]:
    """
    从 robot_calibration.yaml 中读取某一段连续体的真实单元长度。

    返回：
        segment_lengths_m:
            当前 section 中每个盘间单元的长度，单位 m。

    优先级：
        1. 优先使用 disk_gap_lengths_mm：
            segment_i = disk_thickness_mm + disk_gap_lengths_mm[i]

        2. 如果只有 disk_gap_mm：
            每个单元使用同一个间隙：
            segment_i = disk_thickness_mm + disk_gap_mm

        3. 如果只有 segment_lengths_mm：
            认为它已经是“盘厚 + 间隙”的等效单元长度，
            直接转换为 m 使用。

        4. 如果都没有：
            使用 pcc_model 中的 disk_pitch_m 兜底。
    """

    sections_cfg = cfg.get("sections", {})
    if section_name not in sections_cfg:
        raise KeyError(f"robot_calibration.yaml 中没有 sections.{section_name}")

    sec_cfg = sections_cfg[section_name]
    pcc_cfg = cfg.get("pcc_model", {})

    disk_thickness_mm = float(
        sec_cfg.get(
            "disk_thickness_mm",
            float(pcc_cfg.get("disk_thickness_m", 0.010)) * 1000.0,
        )
    )

    gap_count = int(
        sec_cfg.get(
            "gap_count",
            sec_cfg.get("segment_count", pcc_cfg.get("intervals_per_section", 5)),
        )
    )

    # 情况 1：推荐写法。每个盘间净间隙单独实测。
    if "disk_gap_lengths_mm" in sec_cfg:
        gap_lengths_mm = [float(x) for x in sec_cfg["disk_gap_lengths_mm"]]

        if len(gap_lengths_mm) != gap_count:
            raise ValueError(
                f"{section_name}: disk_gap_lengths_mm 数量 "
                f"{len(gap_lengths_mm)} != gap_count {gap_count}"
            )

        segment_lengths_mm = [
            disk_thickness_mm + gap_mm for gap_mm in gap_lengths_mm
        ]

    # 情况 2：所有盘间隙相同。
    elif "disk_gap_mm" in sec_cfg:
        gap_mm = float(sec_cfg["disk_gap_mm"])
        segment_lengths_mm = [
            disk_thickness_mm + gap_mm for _ in range(gap_count)
        ]

    # 情况 3：兼容旧写法。
    # 这里认为 segment_lengths_mm 已经是“盘厚 + 间隙”。
    elif "segment_lengths_mm" in sec_cfg:
        segment_lengths_mm = [float(x) for x in sec_cfg["segment_lengths_mm"]]

        if len(segment_lengths_mm) != gap_count:
            raise ValueError(
                f"{section_name}: segment_lengths_mm 数量 "
                f"{len(segment_lengths_mm)} != gap_count {gap_count}"
            )

    # 情况 4：兜底，使用理想 disk_pitch_m。
    else:
        disk_pitch_m = float(pcc_cfg.get("disk_pitch_m", 0.0585))
        return [disk_pitch_m for _ in range(gap_count)]

    return [x / 1000.0 for x in segment_lengths_mm]


def _add_joint_chain_unit(
    parent_body: ET.Element,
    prefix: str,
    index: int,
    unit_length_m: float,
    disk_rgba: str,
) -> ET.Element:
    """
    在当前 parent_body 下增加一个盘间单元。

    每个单元包含：
        1. 一个 body，位置沿 z 方向前进 unit_length_m；
        2. bend_x 关节；
        3. bend_y 关节；
        4. slide_z 关节；
        5. 六边形盘 mesh；
        6. 中心 backbone capsule。

    返回：
        新创建的 body。
        后续单元会继续挂在这个 body 下面，形成串联结构。
    """

    unit_name = f"{prefix}_u{index}"

    body = ET.SubElement(
        parent_body,
        "body",
        {
            "name": unit_name,
            "pos": _list_to_str([0.0, 0.0, unit_length_m]),
        },
    )

    # 绕 x 轴的等效弯曲关节。
    ET.SubElement(
        body,
        "joint",
        {
            "name": f"{unit_name}_bend_x",
            "type": "hinge",
            "axis": "1 0 0",
            "range": "-0.6 0.6",
        },
    )

    # 绕 y 轴的等效弯曲关节。
    ET.SubElement(
        body,
        "joint",
        {
            "name": f"{unit_name}_bend_y",
            "type": "hinge",
            "axis": "0 1 0",
            "range": "-0.6 0.6",
        },
    )

    # 沿 z 方向的等效轴向伸缩关节。
    ET.SubElement(
        body,
        "joint",
        {
            "name": f"{unit_name}_slide_z",
            "type": "slide",
            "axis": "0 0 1",
            "range": "-0.03 0.03",
        },
    )

    # 六边形盘外观。
    ET.SubElement(
        body,
        "geom",
        {
            "name": f"{unit_name}_disk",
            "type": "mesh",
            "mesh": "hex_disk",
            "rgba": disk_rgba,
            "euler": "0 0 11",
        },
    )

    # 中心线显示杆。
    # fromto 从上一个 body 坐标系方向指向当前盘。
    ET.SubElement(
        body,
        "geom",
        {
            "name": f"{unit_name}_backbone",
            "type": "capsule",
            "fromto": f"0 0 -{_fmt(unit_length_m)} 0 0 0",
            "size": "0.003",
            "rgba": "0.1 0.1 0.1 1",
        },
    )

    return body


def generate_mujoco_xml(
    cfg: Dict,
    output_xml: Path,
    model_name: str = "continuum_kinematic_generated",
    mesh_file: str = "../assets/hex_disk.stl",
) -> None:
    """
    根据配置字典生成 MuJoCo XML 文件。

    参数：
        cfg:
            robot_calibration.yaml 读取出的字典。

        output_xml:
            输出 XML 路径。

        model_name:
            MuJoCo 模型名称。

        mesh_file:
            六边形盘 STL 相对 XML 文件的路径。

            因为 XML 默认生成到：
                models/generated/

            而 STL 在：
                models/assets/

            所以相对路径是：
                ../assets/hex_disk.stl
    """

    pcc_cfg = cfg.get("pcc_model", {})
    chain_order = list(pcc_cfg.get("chain_order", ["section2", "section1"]))

    section_prefix = {
        "section2": "sec2",
        "section1": "sec1",
    }

    section_rgba = {
        "section2": "0.2 0.5 0.9 1",   # 蓝色：靠近基座
        "section1": "0.8 0.35 0.1 1",  # 橙色：靠近末端
    }

    # 创建根节点。
    mujoco = ET.Element("mujoco", {"model": model_name})

    # 角度统一使用 rad。
    ET.SubElement(mujoco, "compiler", {"angle": "radian"})

    # 这里仍然关闭重力，因为当前是运动学显示模型。
    ET.SubElement(
        mujoco,
        "option",
        {
            "timestep": "0.01",
            "gravity": "0 0 0",
        },
    )

    # 默认属性：
    # 1. joint 启用限位；
    # 2. damping/armature 设为 0，避免引入动力学阻尼；
    # 3. geom 不参与碰撞，仅显示。
    default = ET.SubElement(mujoco, "default")

    ET.SubElement(
        default,
        "joint",
        {
            "limited": "true",
            "damping": "0",
            "armature": "0",
        },
    )

    ET.SubElement(
        default,
        "geom",
        {
            "contype": "0",
            "conaffinity": "0",
            "density": "1000",
        },
    )

    # 资产：六边形盘 mesh。
    asset = ET.SubElement(mujoco, "asset")
    ET.SubElement(
        asset,
        "mesh",
        {
            "name": "hex_disk",
            "file": mesh_file,
        },
    )

    # 世界主体。
    worldbody = ET.SubElement(mujoco, "worldbody")

    _add_coordinate_frame(worldbody)

    ET.SubElement(
        worldbody,
        "light",
        {
            "name": "top_light",
            "pos": "0 0 2",
        },
    )

    # 底板。
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "base_plate",
            "type": "box",
            "pos": "0 0 -0.01",
            "size": "0.12 0.12 0.01",
            "rgba": "0.25 0.25 0.25 1",
        },
    )

    # 机器人基座。
    base_body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "base",
            "pos": "0 0 0",
        },
    )

    ET.SubElement(
        base_body,
        "site",
        {
            "name": "base_site",
            "pos": "0 0 0",
            "size": "0.006",
            "rgba": "1 0 0 1",
        },
    )

    # current_body 表示当前串联链末端。
    # section2 生成完后，section1 会接在 section2 后面。
    current_body = base_body

    total_length_m = 0.0

    for section_name in chain_order:
        if section_name not in section_prefix:
            raise ValueError(
                f"暂不支持 section_name={section_name}，"
                "当前仅支持 section2 和 section1"
            )

        prefix = section_prefix[section_name]
        rgba = section_rgba[section_name]
        segment_lengths_m = _get_section_segment_lengths_m(cfg, section_name)

        section_length_m = sum(segment_lengths_m)
        total_length_m += section_length_m

        print(
            f"[generate_mujoco_model] {section_name}: "
            f"segments_m={segment_lengths_m}, "
            f"section_length_m={section_length_m:.6f}"
        )

        for i, unit_length_m in enumerate(segment_lengths_m, start=1):
            current_body = _add_joint_chain_unit(
                parent_body=current_body,
                prefix=prefix,
                index=i,
                unit_length_m=unit_length_m,
                disk_rgba=rgba,
            )

    # 末端绿色标记点。
    ET.SubElement(
        current_body,
        "site",
        {
            "name": "end_site",
            "pos": "0 0 0",
            "size": "0.008",
            "rgba": "0 1 0 1",
        },
    )

    print(
        f"[generate_mujoco_model] total_length_m={total_length_m:.6f}"
    )

    # 美化缩进，方便人工查看 XML。
    ET.indent(mujoco, space="  ")

    tree = ET.ElementTree(mujoco)

    output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)

    print(f"[generate_mujoco_model] XML written to: {output_xml}")


class GenerateMujocoModelNode(Node):
    """
    ROS2 包装节点。

    这个节点只在生成 XML 时运行一次，不需要长期运行。
    """

    def __init__(self):
        super().__init__("generate_mujoco_model")

        bringup_share = Path(get_package_share_directory("robot_bringup"))
        mujoco_share = Path(
            get_package_share_directory("continuum_mujoco_sim")
        )
        generated_dir = Path.home() / ".ros" / "ros_robot1" / "mujoco"

        self.declare_parameter(
            "calibration_file",
            str(bringup_share / "config" / "robot_calibration.yaml"),
        )

        self.declare_parameter(
            "output_xml",
            str(generated_dir / "continuum_kinematic_generated.xml"),
        )

        self.declare_parameter(
            "model_name",
            "continuum_kinematic_generated",
        )

        self.declare_parameter(
            "mesh_file",
            str(mujoco_share / "models" / "assets" / "hex_disk.stl"),
        )

        calibration_file = Path(
            self.get_parameter("calibration_file").value
        ).expanduser()

        output_xml = Path(
            self.get_parameter("output_xml").value
        ).expanduser()

        model_name = str(self.get_parameter("model_name").value)
        mesh_file = str(self.get_parameter("mesh_file").value)

        self.get_logger().info(f"Loading calibration: {calibration_file}")

        with calibration_file.open("r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        generate_mujoco_xml(
            cfg=cfg,
            output_xml=output_xml,
            model_name=model_name,
            mesh_file=mesh_file,
        )

        self.get_logger().info("MuJoCo XML generation finished.")


def main(args=None):
    rclpy.init(args=args)

    node = GenerateMujocoModelNode()

    # 这个节点只负责生成文件，生成后立即退出。
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
