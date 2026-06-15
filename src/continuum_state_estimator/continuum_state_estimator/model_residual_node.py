import rclpy
from rclpy.node import Node

from robot_interfaces.msg import ContinuumState, ModelResidual


class ModelResidualNode(Node):
    def __init__(self):
        super().__init__("model_residual_node")

        self.declare_parameter("continuum_state_topic", "/continuum/state")
        self.declare_parameter("residual_topic", "/continuum/model_residual")

        # 第一版先用简单比例，把 kx/ky 转成近似模型姿态角
        # 后续可根据实验数据标定
        self.declare_parameter("section1_kx_to_pitch_gain", 1.0)
        self.declare_parameter("section1_ky_to_roll_gain", 1.0)
        self.declare_parameter("section2_kx_to_pitch_gain", 1.0)
        self.declare_parameter("section2_ky_to_roll_gain", 1.0)

        self.state_topic = self.get_parameter("continuum_state_topic").value
        self.residual_topic = self.get_parameter("residual_topic").value

        self.s1_kx_to_pitch = float(
            self.get_parameter("section1_kx_to_pitch_gain").value
        )
        self.s1_ky_to_roll = float(
            self.get_parameter("section1_ky_to_roll_gain").value
        )
        self.s2_kx_to_pitch = float(
            self.get_parameter("section2_kx_to_pitch_gain").value
        )
        self.s2_ky_to_roll = float(
            self.get_parameter("section2_ky_to_roll_gain").value
        )

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

        self.get_logger().info("model_residual_node started")

    def state_callback(self, msg: ContinuumState):
        # PCC模型预测姿态，第一版用 kx/ky 的比例近似
        section1_model_pitch = msg.section1_kx * self.s1_kx_to_pitch
        section1_model_roll = msg.section1_ky * self.s1_ky_to_roll

        section2_model_pitch = msg.section2_kx * self.s2_kx_to_pitch
        section2_model_roll = msg.section2_ky * self.s2_ky_to_roll

        # IMU实测姿态
        section1_imu_roll = msg.imu1_roll
        section1_imu_pitch = msg.imu1_pitch

        section2_imu_roll = msg.imu2_roll
        section2_imu_pitch = msg.imu2_pitch

        # 残差 = IMU实测 - PCC预测
        section1_roll_residual = section1_imu_roll - section1_model_roll
        section1_pitch_residual = section1_imu_pitch - section1_model_pitch

        section2_roll_residual = section2_imu_roll - section2_model_roll
        section2_pitch_residual = section2_imu_pitch - section2_model_pitch

        out = ModelResidual()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "continuum_base"

        out.section1_model_roll = float(section1_model_roll)
        out.section1_model_pitch = float(section1_model_pitch)
        out.section2_model_roll = float(section2_model_roll)
        out.section2_model_pitch = float(section2_model_pitch)

        out.section1_imu_roll = float(section1_imu_roll)
        out.section1_imu_pitch = float(section1_imu_pitch)
        out.section2_imu_roll = float(section2_imu_roll)
        out.section2_imu_pitch = float(section2_imu_pitch)

        out.section1_roll_residual = float(section1_roll_residual)
        out.section1_pitch_residual = float(section1_pitch_residual)
        out.section2_roll_residual = float(section2_roll_residual)
        out.section2_pitch_residual = float(section2_pitch_residual)

        out.residual_valid = True
        out.status = "OK"

        self.residual_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ModelResidualNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
