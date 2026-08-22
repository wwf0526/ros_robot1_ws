#!/usr/bin/env python3

import csv
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import ContinuumState

from continuum_mpc.pcc_control_model import PccControlModel


class NmpcPccStatePublisher(Node):

    def __init__(self):

        super().__init__(
            "nmpc_pcc_state_publisher"
        )


        # =========================
        # publisher
        # =========================

        self.publisher = self.create_publisher(
            ContinuumState,
            "/continuum/state",
            10
        )


        # =========================
        # load PCC model
        # =========================

        self.model = self.create_pcc_model()


        # =========================
        # load tendon trajectory
        # =========================

        self.tendon_history = (
            self.load_tendon_history()
        )


        self.index = 0


        self.timer = self.create_timer(
            0.05,
            self.publish_state
        )


        self.get_logger().info(
            "NMPC PCC state publisher started"
        )


    def create_pcc_model(self):

        import yaml

        ws = Path.home() / "ros_robot1_ws"

        cfg_file = (
            ws
            /
            "src"
            /
            "robot_bringup"
            /
            "config"
            /
            "robot_calibration.yaml"
        )


        with open(cfg_file,"r") as f:
            cfg = yaml.safe_load(f)


        return PccControlModel.from_config(
            cfg,
            samples_per_section=5
        )


    def load_tendon_history(self):

        file = (
            Path.home()
            /
            "ros_robot1_ws"
            /
            "src"
            /
            "continuum_mpc"
            /
            "results"
            /
            "tendon_history.csv"
        )


        data = np.loadtxt(
            file,
            delimiter=",",
            skiprows=1
        )

        return data


    def publish_state(self):

        # ==========================================
        # 循环播放NMPC轨迹
        # 用于MuJoCo PCC可视化验证
        #
        # 原模式:
        # step0 -> step130 -> stop
        #
        # 修改后:
        # step0 -> step130 -> step0 -> ...
        #
        # 保持/continuum/state持续发布
        # ==========================================

        if self.index >= len(self.tendon_history):

            # =====================================
            # NMPC轨迹播放完成
            # 保持最终姿态
            #
            # 不重新回到初始状态
            # 避免MuJoCo重复弯曲
            # =====================================

            self.index = len(self.tendon_history) - 1


        tendon = (
            self.tendon_history[self.index]
        )


        output = (
            self.model.evaluate_from_tendon_mm(
                tendon
            )
        )


        msg = ContinuumState()


        # =========================
        # Header
        # =========================

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            "continuum_base"
        )


        # =========================
        # tendon
        # =========================

        msg.tendon_length_mm = (
            tendon.tolist()
        )


        msg.tendon_slack = [
            False
            for _ in range(6)
        ]


        msg.pcc_model_valid = True



        # =========================
        # section states
        # =========================

        self.fill_section_state(
            msg,
            "section1",
            output.section_infos["section1"]
        )


        self.fill_section_state(
            msg,
            "section2",
            output.section_infos["section2"]
        )


        # =========================
        # tip position
        # =========================

        tip = (
            output.tip_transform[:3,3]
        )


        msg.end_model_px_m = float(tip[0])
        msg.end_model_py_m = float(tip[1])
        msg.end_model_pz_m = float(tip[2])


        self.publisher.publish(
            msg
        )


        self.index += 1



    def fill_section_state(
        self,
        msg,
        name,
        info
    ):


        kappa = float(
            info["kappa"]
        )

        phi = float(
            info["phi"]
        )

        length = float(
            info["arc_length"]
        )


        theta = (
            kappa *
            length
        )


        if name == "section1":

            msg.section1_model_theta_rad = theta
            msg.section1_model_phi_rad = phi
            msg.section1_model_kappa_1pm = kappa
            msg.section1_model_arc_length_m = length


            msg.section1_model_l0_m = length


        else:

            msg.section2_model_theta_rad = theta
            msg.section2_model_phi_rad = phi
            msg.section2_model_kappa_1pm = kappa
            msg.section2_model_arc_length_m = length


            msg.section2_model_l0_m = length




def main():

    rclpy.init()

    node = NmpcPccStatePublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()



if __name__ == "__main__":

    main()
