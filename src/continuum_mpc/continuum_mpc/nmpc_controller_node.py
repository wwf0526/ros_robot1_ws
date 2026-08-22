from pathlib import Path
import yaml

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import (
    ContinuumState,
    NmpcReference,
    NmpcCommand,
)

from ament_index_python.packages import get_package_share_directory

from .pcc_control_model import PccControlModel
from .pcc_nmpc_solver import (
    PccNmpcSolver,
    PccNmpcReference,
)


class NmpcControllerNode(Node):

    def __init__(self):
        super().__init__("nmpc_controller_node")

        bringup_share = Path(
            get_package_share_directory("robot_bringup")
        )

        calibration_file = (
            bringup_share /
            "config" /
            "robot_calibration.yaml"
        )

        with calibration_file.open(
            "r",
            encoding="utf-8"
        ) as f:
            cfg = yaml.safe_load(f)

        self.model = PccControlModel.from_config(cfg)

        self.solver = PccNmpcSolver(
            self.model
        )

        self.current_tendon_mm = None

        self.reference = None

        self.previous_velocity = None


        self.state_sub = self.create_subscription(
            ContinuumState,
            "/continuum/state",
            self.state_callback,
            10,
        )

        self.reference_sub = self.create_subscription(
            NmpcReference,
            "/nmpc/reference",
            self.reference_callback,
            10,
        )


        self.command_pub = self.create_publisher(
            NmpcCommand,
            "/continuum/nmpc_command",
            10,
        )


        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )


        self.get_logger().info(
            "NMPC controller started"
        )


    def state_callback(self, msg):

        self.current_tendon_mm = list(
            msg.tendon_length_mm
        )


    def reference_callback(self, msg):

        if not msg.use_position:
            self.reference = None
            return


        self.reference = PccNmpcReference(
            tip_position_m=[
                msg.target_x_m,
                msg.target_y_m,
                msg.target_z_m,
            ]
        )


    def control_loop(self):

        if self.current_tendon_mm is None:
            return

        if self.reference is None:
            return


        result = self.solver.solve(
            self.current_tendon_mm,
            self.reference,
            previous_independent_velocity_mm_s=
            self.previous_velocity,
        )


        self.previous_velocity = (
            result.first_independent_velocity_mm_s
        )


        out = NmpcCommand()

        out.independent_velocity_mm_s = (
            result.first_independent_velocity_mm_s.tolist()
        )

        out.tendon_velocity_mm_s = (
            result.first_tendon_velocity_mm_s.tolist()
        )

        out.valid = result.valid
        out.status = result.status

        out.objective_value = (
            result.objective_value
        )

        out.solve_time_ms = (
            result.solve_time_ms
        )


        self.command_pub.publish(out)



def main(args=None):

    rclpy.init(args=args)

    node = NmpcControllerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
