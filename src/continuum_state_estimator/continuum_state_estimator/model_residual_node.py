import rclpy
from rclpy.node import Node

from robot_interfaces.msg import ContinuumState, ModelResidual

from .orientation_fusion import quat_to_euler_deg, wrap_deg


def msg_quaternion_to_tuple(q):
    return (
        float(q.w),
        float(q.x),
        float(q.y),
        float(q.z),
    )


class ModelResidualNode(Node):
    def __init__(self):
        super().__init__("model_residual_node")

        self.declare_parameter(
            "continuum_state_topic",
            "/continuum/state",
        )
        self.declare_parameter(
            "residual_topic",
            "/continuum/model_residual",
        )

        self.state_topic = self.get_parameter(
            "continuum_state_topic"
        ).value
        self.residual_topic = self.get_parameter(
            "residual_topic"
        ).value

        self.state_sub = self.create_subscription(
            ContinuumState,
            self.state_topic,
            self.state_callback,
            10,
        )

        self.residual_pub = self.create_publisher(
            ModelResidual,
            self.residual_topic,
            10,
        )

        self.get_logger().info(
            "model_residual_node started: quaternion-aligned PCC/IMU diagnostic residual"
        )

    def state_callback(self, msg: ContinuumState):
        out = ModelResidual()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "continuum_base"

        valid = bool(
            msg.pcc_model_valid
            and msg.imu_zero_ready
            and msg.imu1_aligned_valid
            and msg.imu2_aligned_valid
        )

        if not valid:
            out.residual_valid = False
            out.status = "INVALID: PCC/IMU aligned orientation not ready"
            self.residual_pub.publish(out)
            return

        s1_model = quat_to_euler_deg(
            msg_quaternion_to_tuple(msg.section1_model_orientation)
        )
        s1_imu = quat_to_euler_deg(
            msg_quaternion_to_tuple(msg.imu1_aligned_orientation)
        )

        s2_model = quat_to_euler_deg(
            msg_quaternion_to_tuple(msg.section2_model_orientation)
        )
        s2_imu = quat_to_euler_deg(
            msg_quaternion_to_tuple(msg.imu2_aligned_orientation)
        )

        s1_roll_res = wrap_deg(s1_imu[0] - s1_model[0])
        s1_pitch_res = wrap_deg(s1_imu[1] - s1_model[1])
        s2_roll_res = wrap_deg(s2_imu[0] - s2_model[0])
        s2_pitch_res = wrap_deg(s2_imu[1] - s2_model[1])

        out.section1_model_roll = float(s1_model[0])
        out.section1_model_pitch = float(s1_model[1])
        out.section2_model_roll = float(s2_model[0])
        out.section2_model_pitch = float(s2_model[1])

        out.section1_imu_roll = float(s1_imu[0])
        out.section1_imu_pitch = float(s1_imu[1])
        out.section2_imu_roll = float(s2_imu[0])
        out.section2_imu_pitch = float(s2_imu[1])

        out.section1_roll_residual = float(s1_roll_res)
        out.section1_pitch_residual = float(s1_pitch_res)
        out.section2_roll_residual = float(s2_roll_res)
        out.section2_pitch_residual = float(s2_pitch_res)

        out.residual_valid = True
        out.status = "OK: quaternion-aligned residual in deg"

        self.residual_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ModelResidualNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
