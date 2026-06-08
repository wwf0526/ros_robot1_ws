import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from robot_interfaces.msg import ZeroCheckState


def quaternion_to_euler_deg(x, y, z, w):
    # roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


class ImuZeroCheckNode(Node):
    def __init__(self):
        super().__init__("imu_zero_check_node")

        self.declare_parameter("imu1_topic", "/imu1/data_raw")
        self.declare_parameter("imu2_topic", "/imu2/data_raw")
        self.declare_parameter("zero_check_topic", "/continuum/zero_check")
        self.declare_parameter("roll_tolerance_deg", 1.0)
        self.declare_parameter("pitch_tolerance_deg", 1.0)
        self.declare_parameter("publish_rate", 10.0)

        self.imu1_topic = self.get_parameter("imu1_topic").value
        self.imu2_topic = self.get_parameter("imu2_topic").value
        self.zero_check_topic = self.get_parameter("zero_check_topic").value
        self.roll_tol = float(self.get_parameter("roll_tolerance_deg").value)
        self.pitch_tol = float(self.get_parameter("pitch_tolerance_deg").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.imu1_euler = None
        self.imu2_euler = None

        self.imu1_sub = self.create_subscription(
            Imu,
            self.imu1_topic,
            self.imu1_callback,
            10,
        )

        self.imu2_sub = self.create_subscription(
            Imu,
            self.imu2_topic,
            self.imu2_callback,
            10,
        )

        self.zero_pub = self.create_publisher(
            ZeroCheckState,
            self.zero_check_topic,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_zero_check,
        )

        self.get_logger().info("imu_zero_check_node started")

    def imu1_callback(self, msg: Imu):
        q = msg.orientation
        self.imu1_euler = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)

    def imu2_callback(self, msg: Imu):
        q = msg.orientation
        self.imu2_euler = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)

    def publish_zero_check(self):
        if self.imu1_euler is None or self.imu2_euler is None:
            return

        imu1_roll, imu1_pitch, imu1_yaw = self.imu1_euler
        imu2_roll, imu2_pitch, imu2_yaw = self.imu2_euler

        imu1_level = (
            abs(imu1_roll) <= self.roll_tol
            and abs(imu1_pitch) <= self.pitch_tol
        )

        imu2_level = (
            abs(imu2_roll) <= self.roll_tol
            and abs(imu2_pitch) <= self.pitch_tol
        )

        robot_level = imu1_level and imu2_level

        msg = ZeroCheckState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "continuum_base"

        msg.imu1_roll_deg = float(imu1_roll)
        msg.imu1_pitch_deg = float(imu1_pitch)
        msg.imu1_yaw_deg = float(imu1_yaw)

        msg.imu2_roll_deg = float(imu2_roll)
        msg.imu2_pitch_deg = float(imu2_pitch)
        msg.imu2_yaw_deg = float(imu2_yaw)

        msg.roll_tolerance_deg = self.roll_tol
        msg.pitch_tolerance_deg = self.pitch_tol

        msg.imu1_level = imu1_level
        msg.imu2_level = imu2_level
        msg.robot_level = robot_level

        if robot_level:
            msg.status = "LEVEL"
        else:
            msg.status = "NOT_LEVEL"

        self.zero_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuZeroCheckNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
