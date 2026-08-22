from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from robot_interfaces.msg import (
    NmpcCommand,
    ContinuumState,
)

from continuum_mpc.pcc_control_model import PccControlModel


class MujocoNmpcBridge(Node):

    def __init__(self):

        super().__init__(
            "mujoco_nmpc_bridge"
        )

        self.dt = 0.05   # 20Hz



        # =========================
        # 加载机器人标定文件
        # =========================

        bringup_share = Path(
            get_package_share_directory(
                "robot_bringup"
            )
        )

        calibration_file = (
            bringup_share
            /
            "config"
            /
            "robot_calibration.yaml"
        )


        with calibration_file.open(
            "r",
            encoding="utf-8"
        ) as f:

            cfg = yaml.safe_load(f)

        # =========================
        # 从robot_calibration读取初始绳长
        # zero_length_mm:
        #   1: ...
        #   2: ...
        #   ...
        #   6: ...
        # =========================

        zero_length_cfg = cfg.get(
            "zero_length_mm",
            {}
        )


        if isinstance(zero_length_cfg, dict):

            self.tendon_length_mm = np.array(
                [
                    zero_length_cfg.get(1, 0.0),
                    zero_length_cfg.get(2, 0.0),
                    zero_length_cfg.get(3, 0.0),
                    zero_length_cfg.get(4, 0.0),
                    zero_length_cfg.get(5, 0.0),
                    zero_length_cfg.get(6, 0.0),
                ],
                dtype=float,
            )

        else:

            self.tendon_length_mm = np.zeros(
                6,
                dtype=float,
            )


        # 创建PCC模型
        self.model = (
            PccControlModel
            .from_config(
            cfg
            )
        )


        self.get_logger().info(
            "PCC model loaded"
        )


        # =========================
        # 输入 NMPC command
        # =========================

        self.command_sub = (
            self.create_subscription(
                NmpcCommand,
                "/continuum/nmpc_command",
                self.command_callback,
                10,
            )
        )


        # =========================
        # 输出状态
        # =========================

        self.state_pub = (
            self.create_publisher(
                ContinuumState,
                "/continuum/state",
                10,
            )
        )


        self.latest_velocity = np.zeros(
            6,
            dtype=float
        )


        self.timer = (
            self.create_timer(
                self.dt,
                self.update,
            )
        )


        self.get_logger().info(
            "MuJoCo NMPC bridge started"
        )


    def command_callback(
        self,
        msg: NmpcCommand
    ):

        if not msg.valid:
            return


        velocity = np.asarray(
            msg.tendon_velocity_mm_s,
            dtype=float
        )


        if velocity.shape == (6,):

            self.latest_velocity = velocity



    def update(self):


        # =========================
        # 执行器积分
        #
        # L(k+1)=L(k)+v*dt
        # =========================

        self.tendon_length_mm += (
            self.latest_velocity
            *
            self.dt
        )


        try:

            output = (
                self.model
                .evaluate_from_tendon_mm(
                    self.tendon_length_mm
                )
            )

        except Exception as e:

            self.get_logger().warn(
                f"PCC evaluate failed: {e}"
            )

            return



        msg = ContinuumState()


        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            "continuum_base"
        )


        msg.tendon_length_mm = (
            self.tendon_length_mm
            .tolist()
        )

        msg.tendon_slack = [
            False,
            False,
            False,
            False,
            False,
            False,
        ]

        msg.pcc_model_valid = True



        # =========================
        # section状态
        # =========================

        sec1 = output.section_infos[
            "section1"
        ]

        sec2 = output.section_infos[
            "section2"
        ]


        msg.section1_model_theta_rad = (
            sec1["theta"]
        )

        msg.section1_model_phi_rad = (
            sec1["phi"]
        )

        msg.section1_model_kappa_1pm = (
            sec1["kappa"]
        )

        msg.section1_model_arc_length_m = (
            sec1["arc_length"]
        )
        
        msg.section1_model_l0_m = (
            float(sec1["arc_length"])
        )


        msg.section2_model_theta_rad = (
            sec2["theta"]
        )

        msg.section2_model_phi_rad = (
            sec2["phi"]
        )

        msg.section2_model_kappa_1pm = (
            sec2["kappa"]
        )

        msg.section2_model_arc_length_m = (
            sec2["arc_length"]
        )

        msg.section2_model_l0_m = (
            float(sec2["arc_length"])
        )


        # =========================
        # tip
        # =========================

        tip = output.tip_position_m


        msg.end_model_px_m = float(
            tip[0]
        )

        msg.end_model_py_m = float(
            tip[1]
        )

        msg.end_model_pz_m = float(
            tip[2]
        )


        self.state_pub.publish(
            msg
        )



def main(args=None):

    rclpy.init(args=args)

    node = MujocoNmpcBridge()

    rclpy.spin(node)


    node.destroy_node()

    rclpy.shutdown()



if __name__ == "__main__":

    main()
