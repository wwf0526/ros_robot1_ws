"""
mujoco_pcc_viewer.py

功能说明
========
这个节点是连续体机器人 MuJoCo 可视化节点。

它不负责：
1. 电机通信；
2. IMU 读取；
3. PCC 正运动学计算；
4. 逆运动学求解；
5. MPC / RL 控制。

它只负责：
1. 加载 MuJoCo 模型 continuum_kinematic.xml；
2. 订阅状态估计节点发布的 /continuum/state；
3. 从 ContinuumState 中读取每段 PCC 构型参数：
   - theta：该段总弯曲角，单位 rad；
   - phi：该段弯曲方向，单位 rad；
   - arc_length：该段当前中心线弧长，单位 m；
   - l0：该段真实装配初始长度，单位 m；
4. 把每段 PCC 构型参数映射到 MuJoCo 中的虚拟关节：
   - bend_x；
   - bend_y；
   - slide_z；
5. 实时刷新 MuJoCo viewer。

系统数据流
==========
真实电机 / IMU
    ↓
continuum_state_estimator
    ↓ 发布 robot_interfaces/msg/ContinuumState
/continuum/state
    ↓
mujoco_pcc_viewer
    ↓
MuJoCo 可视化

注意
====
当前 MuJoCo 模型是“运动学显示模型”，不是高保真动力学模型。
本节点使用 mujoco.mj_forward() 根据 qpos 直接更新模型姿态，
不是使用 mujoco.mj_step() 做动力学积分。
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Dict

import mujoco
import mujoco.viewer
import numpy as np

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from robot_interfaces.msg import ContinuumState


class MujocoPccViewer(Node):
    """
    MuJoCo PCC 可视化节点。

    节点输入：
        /continuum/state，类型 robot_interfaces/msg/ContinuumState。

    节点输出：
        无 ROS 输出，只更新 MuJoCo viewer 中的模型姿态。

    关键约定：
        真实机器人结构顺序为：
            基座 -> section2 -> section1 -> 末端

        因此 MuJoCo 模型中的 body / joint 命名也使用：
            sec2_u1 ... sec2_u5
            sec1_u1 ... sec1_u5
    """

    def __init__(self):
        super().__init__("mujoco_pcc_viewer")

        # =====================================================
        # 1. 声明 ROS2 参数
        # =====================================================
        # 状态估计节点发布的完整连续体状态话题。
        # 后续 MPC、RL、MuJoCo 都可以订阅这个统一状态源。
        self.declare_parameter("state_topic", "/continuum/state")

        # MuJoCo 模型文件名称。
        # 默认从 continuum_mujoco_sim/models/continuum_kinematic.xml 加载。
        self.declare_parameter("model_xml", "continuum_kinematic.xml")

        # 每段在 MuJoCo XML 中离散为多少个盘间单元。
        # 你当前模型是每段 5 个单元：sec2_u1~sec2_u5, sec1_u1~sec1_u5。
        self.declare_parameter("intervals_per_section", 5)

        # 当前 MuJoCo XML 中每段的“名义显示长度”。
        # 如果 XML 仍是理想模型，每个单元 pos=0.0585，那么每段长度为：
        # 0.0585 * 5 = 0.2925 m。
        #
        # 注意：这不一定等于真实机器人 L0。
        # 真实 L0 来自 /continuum/state 中的 section*_model_l0_m。
        self.declare_parameter("xml_sec2_length_m", 0.14)
        self.declare_parameter("xml_sec1_length_m", 0.2)

        # 是否使用 /continuum/state 中的 L0 作为 slide_z 的参考长度。
        #
        # False：适合当前阶段。
        #   XML 还是理想长度 0.2925 m，但真实 section2/section1 可能是 0.14/0.20 m。
        #   此时 qz = (arc_length - xml_length) / n，
        #   可以把理想 XML 显示模型压缩到接近真实弧长。
        #
        # True：适合后续阶段。
        #   如果 MuJoCo XML 已经根据 robot_calibration.yaml 自动生成，
        #   XML 本身就等于真实 L0，则用 qz = (arc_length - L0) / n，
        #   零输入直线状态下 qz = 0。
        self.declare_parameter("use_state_l0_for_qz", False)

        # 弯曲关节和轴向伸缩关节的显示限幅。
        # 这些限幅应与 XML 中 joint range 尽量一致。
        self.declare_parameter("bend_limit_rad", 0.6)
        self.declare_parameter("slide_limit_m", 0.03)

        # 是否打印每次接收到的状态，默认关闭，避免刷屏。
        self.declare_parameter("debug_print_state", False)

        # =====================================================
        # 2. 读取 ROS2 参数
        # =====================================================
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.model_xml = str(self.get_parameter("model_xml").value)
        self.n = int(self.get_parameter("intervals_per_section").value)

        self.xml_section_length: Dict[str, float] = {
            "sec2": float(self.get_parameter("xml_sec2_length_m").value),
            "sec1": float(self.get_parameter("xml_sec1_length_m").value),
        }

        self.use_state_l0_for_qz = bool(
            self.get_parameter("use_state_l0_for_qz").value
        )
        self.bend_limit = float(self.get_parameter("bend_limit_rad").value)
        self.slide_limit = float(self.get_parameter("slide_limit_m").value)
        self.debug_print_state = bool(self.get_parameter("debug_print_state").value)

        if self.n <= 0:
            raise ValueError("intervals_per_section 必须大于 0")

        # =====================================================
        # 3. 初始化缓存状态
        # =====================================================
        # section_state 用来保存最近一次从 /continuum/state 接收到的 PCC 构型。
        # 之所以不直接在回调里操作 MuJoCo，是因为 MuJoCo viewer 主循环和 ROS 回调
        # 分开处理更稳定。
        self.section_state = {
            "sec2": {
                "theta": 0.0,
                "phi": 0.0,
                "arc_length": self.xml_section_length["sec2"],
                "l0": self.xml_section_length["sec2"],
                "valid": False,
            },
            "sec1": {
                "theta": 0.0,
                "phi": 0.0,
                "arc_length": self.xml_section_length["sec1"],
                "l0": self.xml_section_length["sec1"],
                "valid": False,
            },
        }

        # 用于限制“qz 被限幅”的日志次数，避免运行时刷屏。
        self._clip_warn_count = 0
        self._clip_warn_max = 5

        # =====================================================
        # 4. 创建 ROS2 订阅器
        # =====================================================
        # 这里是本节点最关键的修改：
        # 旧版订阅 /pcc/section_state，类型 Float64MultiArray；
        # 新版订阅 /continuum/state，类型 ContinuumState。
        self.sub = self.create_subscription(
            ContinuumState,
            self.state_topic,
            self.continuum_state_callback,
            10,
        )

        # =====================================================
        # 5. 加载 MuJoCo 模型
        # =====================================================
        # model_xml 可以是：
        # 1. 文件名，例如 continuum_kinematic.xml；
        # 2. 绝对路径，例如 /home/.../continuum_kinematic.xml。
        model_path = self._resolve_model_path(self.model_xml)

        self.get_logger().info(f"Loading MuJoCo model: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)

        self.get_logger().info(
            "Subscribed to ContinuumState: "
            f"{self.state_topic}. "
            "Using fields: "
            "section2_model_theta_rad, section2_model_phi_rad, "
            "section2_model_arc_length_m, section2_model_l0_m, "
            "section1_model_theta_rad, section1_model_phi_rad, "
            "section1_model_arc_length_m, section1_model_l0_m."
        )

    def _resolve_model_path(self, model_xml: str) -> Path:
        """
        解析 MuJoCo XML 模型路径。

        如果传入的是绝对路径，则直接使用。
        如果传入的是文件名，则默认去当前 ROS2 包的 share/models 目录中查找。
        """
        path = Path(model_xml).expanduser()

        if path.is_absolute():
            return path

        pkg_share = Path(get_package_share_directory("continuum_mujoco_sim"))
        return pkg_share / "models" / model_xml

    # =========================================================
    # ROS2 回调：接收完整连续体状态
    # =========================================================
    def continuum_state_callback(self, msg: ContinuumState) -> None:
        """
        接收 /continuum/state 并提取 MuJoCo 显示需要的数据。

        ContinuumState 中有很多字段，例如：
        - 电机/绳长状态；
        - IMU 状态；
        - 安全状态；
        - 每段 PCC 模型状态；
        - 末端位姿。

        MuJoCo viewer 当前只需要每段的：
        - theta；
        - phi；
        - arc_length；
        - L0。
        """
        if not bool(msg.pcc_model_valid):
            self.get_logger().warn(
                "Received ContinuumState but pcc_model_valid is false, ignored"
            )
            return

        # 注意顺序：真实链路是 section2 -> section1。
        # 这里用 sec2 / sec1 是为了与 MuJoCo XML 中的 joint 命名保持一致。
        self.section_state["sec2"] = {
            "theta": float(msg.section2_model_theta_rad),
            "phi": float(msg.section2_model_phi_rad),
            "arc_length": float(msg.section2_model_arc_length_m),
            "l0": float(msg.section2_model_l0_m),
            "valid": True,
        }

        self.section_state["sec1"] = {
            "theta": float(msg.section1_model_theta_rad),
            "phi": float(msg.section1_model_phi_rad),
            "arc_length": float(msg.section1_model_arc_length_m),
            "l0": float(msg.section1_model_l0_m),
            "valid": True,
        }

        if self.debug_print_state:
            self.get_logger().info(
                "ContinuumState received: "
                f"sec2(theta={self.section_state['sec2']['theta']:.4f}, "
                f"phi={self.section_state['sec2']['phi']:.4f}, "
                f"arc={self.section_state['sec2']['arc_length']:.4f}, "
                f"l0={self.section_state['sec2']['l0']:.4f}), "
                f"sec1(theta={self.section_state['sec1']['theta']:.4f}, "
                f"phi={self.section_state['sec1']['phi']:.4f}, "
                f"arc={self.section_state['sec1']['arc_length']:.4f}, "
                f"l0={self.section_state['sec1']['l0']:.4f})"
            )

    # =========================================================
    # MuJoCo 工具函数：设置关节 qpos
    # =========================================================
    def _set_joint_qpos(self, joint_name: str, value: float) -> None:
        """
        根据关节名称设置 MuJoCo qpos。

        当前 XML 中每个单元有三个关节，例如：
        - sec2_u1_bend_x
        - sec2_u1_bend_y
        - sec2_u1_slide_z

        如果 XML 中的关节名和这里不一致，就会进入 KeyError。
        """
        try:
            self.data.joint(joint_name).qpos[0] = float(value)
        except KeyError:
            self.get_logger().error(f"Joint not found in MuJoCo model: {joint_name}")

    def _get_qz_reference_length(self, prefix: str, l0_from_state: float) -> float:
        """
        获取 slide_z 的参考长度。

        prefix:
            sec2 或 sec1。

        l0_from_state:
            /continuum/state 中的真实装配初始长度。

        返回值：
            用于计算 qz 的参考长度。

        计算方式：
            qz = (arc_length - reference_length) / n
        """
        if self.use_state_l0_for_qz and l0_from_state > 0.0:
            return float(l0_from_state)

        return float(self.xml_section_length[prefix])

    # =========================================================
    # PCC -> MuJoCo 关节映射
    # =========================================================
    def _apply_one_section(self, prefix: str, state: Dict[str, float]) -> None:
        """
        将单段 PCC 状态应用到 MuJoCo 对应的 5 个盘间单元。

        参数：
            prefix:
                sec2 或 sec1，对应 XML 中的关节名前缀。

            state:
                某一段的 PCC 状态，包含：
                - theta：该段总弯曲角，rad；
                - phi：该段弯曲方向，rad；
                - arc_length：该段当前中心线弧长，m；
                - l0：该段真实初始长度，m。

        映射逻辑：
            1. 每段总弯曲角 theta 平均分配到 n 个单元：
                   unit_theta = theta / n

            2. 根据弯曲方向 phi，把 unit_theta 分解到两个正交铰链：
                   qx = -unit_theta * sin(phi)
                   qy =  unit_theta * cos(phi)

            3. 轴向伸缩按每个单元平均分配：
                   qz = (arc_length - reference_length) / n
        """
        theta = float(state["theta"])
        phi = float(state["phi"])
        arc_length = float(state["arc_length"])
        l0 = float(state["l0"])

        if not np.isfinite([theta, phi, arc_length, l0]).all():
            self.get_logger().warn(f"{prefix}: non-finite state ignored")
            return

        # 1. 每个离散单元平均分配弯曲角。
        unit_theta = theta / self.n

        # 2. 将 PCC 的 theta/phi 分解为 MuJoCo 的 bend_x / bend_y。
        # 这个映射要和你之前的 MuJoCo 建模约定保持一致。
        qx_raw = -unit_theta * math.sin(phi)
        qy_raw = unit_theta * math.cos(phi)

        # 3. 计算轴向伸缩。
        reference_length = self._get_qz_reference_length(prefix, l0)
        qz_raw = (arc_length - reference_length) / self.n

        # 4. 安全限幅，防止异常数据导致显示模型飞掉。
        qx = float(np.clip(qx_raw, -self.bend_limit, self.bend_limit))
        qy = float(np.clip(qy_raw, -self.bend_limit, self.bend_limit))
        qz = float(np.clip(qz_raw, -self.slide_limit, self.slide_limit))

        if abs(qz_raw - qz) > 1e-12 and self._clip_warn_count < self._clip_warn_max:
            self.get_logger().warn(
                f"{prefix}: slide_z clipped from {qz_raw:.6f} to {qz:.6f}. "
                "If this is expected, either increase slide_limit_m or regenerate XML "
                "with real section lengths."
            )
            self._clip_warn_count += 1

        # 5. 把同样的 qx/qy/qz 写入该段的所有离散单元。
        for i in range(1, self.n + 1):
            self._set_joint_qpos(f"{prefix}_u{i}_bend_x", qx)
            self._set_joint_qpos(f"{prefix}_u{i}_bend_y", qy)
            self._set_joint_qpos(f"{prefix}_u{i}_slide_z", qz)

    def apply_current_state_to_mujoco(self) -> None:
        """
        将最近一次接收到的连续体状态应用到 MuJoCo。

        注意：
            这里不调用 mj_step()，因为当前不是动力学仿真；
            而是直接写 qpos 后调用 mj_forward() 更新几何位置。
        """
        self._apply_one_section("sec2", self.section_state["sec2"])
        self._apply_one_section("sec1", self.section_state["sec1"])

        mujoco.mj_forward(self.model, self.data)


def main(args=None) -> None:
    """
    ROS2 节点入口。

    主循环逻辑：
        1. rclpy.spin_once() 处理一次 ROS 回调；
        2. apply_current_state_to_mujoco() 把最新状态写入 MuJoCo；
        3. viewer.sync() 刷新显示窗口；
        4. sleep 0.01 s，约 100 Hz。
    """
    rclpy.init(args=args)
    node = MujocoPccViewer()

    with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
        node.get_logger().info("MuJoCo viewer started.")

        while rclpy.ok() and viewer.is_running():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.apply_current_state_to_mujoco()
            viewer.sync()
            time.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
