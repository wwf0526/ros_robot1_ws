import math
import yaml
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MotorState, TendonState, ContinuumState
from continuum_model.pcc_model import compute_section_curvature


class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__('continuum_state_estimator')

        self.declare_parameter(
            "calibration_file",
            "/home/wangwenfeng/ros_robot1_ws/src/robot_bringup/config/robot_calibration.yaml"
        )

        self.calibration_file = self.get_parameter("calibration_file").value

        self.motor_position_deg = {}
        self.load_calibration()

        self.tendon_pub = self.create_publisher(
            TendonState,
            '/continuum/tendon_state',
            10
        )

        self.continuum_pub = self.create_publisher(
            ContinuumState,
            '/continuum/state',
            10
        )

        self.motor_sub = self.create_subscription(
            MotorState,
            '/motor/state',
            self.motor_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.publish_tendon_state)

        self.get_logger().info("continuum_state_estimator started")

    def load_calibration(self):
        with open(self.calibration_file, "r") as f:
            cfg = yaml.safe_load(f)

        self.motor_ids = cfg["motor"]["motor_ids"]
        self.sign = cfg["motor"]["sign"]
        self.spool_radius_mm = cfg["motor"]["spool_radius_mm"]
        self.motor_to_tendon = cfg["tendon"]["motor_to_tendon"]
        self.slack_threshold_mm = float(cfg["tendon"]["slack_threshold_mm"])

        self.tendon_angles_deg = cfg["tendon"]["angle_deg"]
        self.sections = cfg["sections"]
        
        for mid in self.motor_ids:
            self.motor_position_deg[int(mid)] = 0.0

    def motor_callback(self, msg: MotorState):
        self.motor_position_deg[int(msg.motor_id)] = float(msg.position_deg)

    def publish_tendon_state(self):
        tendon_length_mm = [0.0] * 6
        motor_position_deg = [0.0] * 6
        tendon_slack = [False] * 6

        for motor_id in self.motor_ids:
            motor_id = int(motor_id)

            pos_deg = self.motor_position_deg.get(motor_id, 0.0)
            theta_rad = math.radians(pos_deg)

            sign = float(self.sign[motor_id])
            radius = float(self.spool_radius_mm[motor_id])

            delta_l = sign * radius * theta_rad

            tendon_id = int(self.motor_to_tendon[motor_id])
            index = tendon_id - 1

            tendon_length_mm[index] = delta_l
            motor_position_deg[index] = pos_deg
            tendon_slack[index] = abs(delta_l) < self.slack_threshold_mm

        msg = TendonState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "continuum_base"
        msg.tendon_length_mm = tendon_length_mm
        msg.motor_position_deg = motor_position_deg
        msg.tendon_slack = tendon_slack

        self.tendon_pub.publish(msg)
        self.publish_continuum_state(tendon_length_mm, tendon_slack)

    def publish_continuum_state(self, tendon_length_mm, tendon_slack):
        section1 = self.sections["section1"]
        section2 = self.sections["section2"]

        section1_tendon_lengths = {}
        section1_tendon_angles = {}

        for tid in section1["tendon_ids"]:
            tid = int(tid)
            section1_tendon_lengths[tid] = tendon_length_mm[tid - 1]
            section1_tendon_angles[tid] = float(self.tendon_angles_deg[tid])

        section2_tendon_lengths = {}
        section2_tendon_angles = {}

        for tid in section2["tendon_ids"]:
            tid = int(tid)
            section2_tendon_lengths[tid] = tendon_length_mm[tid - 1]
            section2_tendon_angles[tid] = float(self.tendon_angles_deg[tid])

        section1_radius = float(section1["tendon_radius_mm"])
        section2_radius = float(section2["tendon_radius_mm"])

        section1_kx, section1_ky = compute_section_curvature(
            section1_tendon_lengths,
            section1_tendon_angles,
            section1_radius
        )

        section2_kx, section2_ky = compute_section_curvature(
            section2_tendon_lengths,
            section2_tendon_angles,
            section2_radius
        )

        state_msg = ContinuumState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = "continuum_base"

        state_msg.section1_kx = float(section1_kx)
        state_msg.section1_ky = float(section1_ky)
        state_msg.section2_kx = float(section2_kx)
        state_msg.section2_ky = float(section2_ky)

        state_msg.imu1_roll = 0.0
        state_msg.imu1_pitch = 0.0
        state_msg.imu1_yaw = 0.0

        state_msg.imu2_roll = 0.0
        state_msg.imu2_pitch = 0.0
        state_msg.imu2_yaw = 0.0

        state_msg.tendon_length_mm = tendon_length_mm
        state_msg.tendon_slack = tendon_slack

        self.continuum_pub.publish(state_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
