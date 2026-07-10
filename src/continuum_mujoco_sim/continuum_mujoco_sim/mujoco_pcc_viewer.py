import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from ament_index_python.packages import get_package_share_directory


class MujocoPccViewer(Node):
    def __init__(self):
        super().__init__('mujoco_pcc_viewer')

        self.declare_parameter('topic_name', '/pcc/section_state')
        self.declare_parameter('section_length_m', 0.2)
        self.declare_parameter('intervals_per_section', 5)

        self.topic_name = self.get_parameter('topic_name').value
        self.section_length = float(self.get_parameter('section_length_m').value)
        self.n = int(self.get_parameter('intervals_per_section').value)

        self.section_state = [
            0.0, 0.0, self.section_length,
            0.0, 0.0, self.section_length,
        ]

        self.sub = self.create_subscription(
            Float64MultiArray,
            self.topic_name,
            self.section_state_callback,
            10
        )

        pkg_share = Path(get_package_share_directory('continuum_mujoco_sim'))
        model_path = pkg_share / 'models' / 'continuum_kinematic.xml'

        self.get_logger().info(f'Loading MuJoCo model: {model_path}')
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)

        self.get_logger().info(
            'Subscribed to /pcc/section_state, data order: '
            '[sec2_theta, sec2_phi, sec2_l, sec1_theta, sec1_phi, sec1_l]'
        )

    def section_state_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            self.get_logger().warn('section_state length < 6, ignored')
            return

        self.section_state = list(msg.data[:6])

    def _set_joint_qpos(self, joint_name: str, value: float):
        try:
            self.data.joint(joint_name).qpos[0] = float(value)
        except KeyError:
            self.get_logger().error(f'Joint not found in MuJoCo model: {joint_name}')

    def _apply_one_section(self, prefix: str, theta: float, phi: float, arc_length: float):
        """
        prefix: sec2 或 sec1
        theta: 该段总弯曲角 rad
        phi: 该段弯曲方向 rad
        arc_length: 该段当前中心线长度 m
        """

        # 每个单元平均分配弯曲角。
        unit_theta = theta / self.n

        # 将 PCC 的 theta/phi 分解为两个正交弯曲关节。
        qx = -unit_theta * math.sin(phi)
        qy = unit_theta * math.cos(phi)

        # 每个单元平均分配轴向伸缩量。
        qz = (arc_length - self.section_length) / self.n

        # 安全限幅，防止错误数据让显示模型瞬间乱飞。
        qx = float(np.clip(qx, -0.6, 0.6))
        qy = float(np.clip(qy, -0.6, 0.6))
        qz = float(np.clip(qz, -0.03, 0.03))

        for i in range(1, self.n + 1):
            self._set_joint_qpos(f'{prefix}_u{i}_bend_x', qx)
            self._set_joint_qpos(f'{prefix}_u{i}_bend_y', qy)
            self._set_joint_qpos(f'{prefix}_u{i}_slide_z', qz)

    def apply_current_state_to_mujoco(self):
        sec2_theta, sec2_phi, sec2_l, sec1_theta, sec1_phi, sec1_l = self.section_state

        # 你的真实链路顺序是 section2 -> section1
        self._apply_one_section('sec2', sec2_theta, sec2_phi, sec2_l)
        self._apply_one_section('sec1', sec1_theta, sec1_phi, sec1_l)

        mujoco.mj_forward(self.model, self.data)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoPccViewer()

    with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
        node.get_logger().info('MuJoCo viewer started.')

        while rclpy.ok() and viewer.is_running():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.apply_current_state_to_mujoco()
            viewer.sync()
            time.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
