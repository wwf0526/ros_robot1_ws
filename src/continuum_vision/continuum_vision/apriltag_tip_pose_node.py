import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

import tf2_ros


class AprilTagTipPoseNode(Node):
    def __init__(self):
        super().__init__("apriltag_tip_pose_node")

        self.declare_parameter("base_frame", "camera")
        self.declare_parameter("tag_frame", "continuum_tip_tag")
        self.declare_parameter("output_pose_topic", "/continuum/vision_tip_pose")
        self.declare_parameter("output_longitudinal_topic", "/continuum/vision_longitudinal_m")
        self.declare_parameter("lookup_rate_hz", 30.0)
        self.declare_parameter("longitudinal_axis", "z")
        self.declare_parameter("zero_on_first_detection", True)
        self.declare_parameter("vision_timeout_sec", 0.5)

        self.base_frame = self.get_parameter("base_frame").value
        self.tag_frame = self.get_parameter("tag_frame").value
        self.output_pose_topic = self.get_parameter("output_pose_topic").value
        self.output_longitudinal_topic = self.get_parameter("output_longitudinal_topic").value
        self.lookup_rate_hz = float(self.get_parameter("lookup_rate_hz").value)
        self.longitudinal_axis = str(self.get_parameter("longitudinal_axis").value)
        self.zero_on_first_detection = bool(self.get_parameter("zero_on_first_detection").value)
        self.vision_timeout_sec = float(self.get_parameter("vision_timeout_sec").value)

        if self.longitudinal_axis not in ["x", "y", "z"]:
            raise ValueError("longitudinal_axis must be one of: x, y, z")

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.longitudinal_pub = self.create_publisher(Float64, self.output_longitudinal_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.zero_value = None
        self.last_valid_time = None

        period = 1.0 / max(self.lookup_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"Started. base_frame={self.base_frame}, tag_frame={self.tag_frame}, "
            f"axis={self.longitudinal_axis}"
        )

    def _axis_value(self, pose_msg: PoseStamped) -> float:
        p = pose_msg.pose.position
        if self.longitudinal_axis == "x":
            return float(p.x)
        if self.longitudinal_axis == "y":
            return float(p.y)
        return float(p.z)

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tag_frame,
                Time(),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.base_frame} -> {self.tag_frame}: {exc}",
                throttle_duration_sec=1.0,
            )
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.base_frame

        pose_msg.pose.position.x = float(transform.transform.translation.x)
        pose_msg.pose.position.y = float(transform.transform.translation.y)
        pose_msg.pose.position.z = float(transform.transform.translation.z)
        pose_msg.pose.orientation = transform.transform.rotation

        self.pose_pub.publish(pose_msg)

        value = self._axis_value(pose_msg)

        if self.zero_on_first_detection and self.zero_value is None:
            self.zero_value = value
            self.get_logger().info(
                f"Vision zero set: {self.longitudinal_axis}0 = {self.zero_value:.6f} m"
            )

        displacement = value - self.zero_value if self.zero_value is not None else value

        msg = Float64()
        msg.data = float(displacement)
        self.longitudinal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagTipPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
