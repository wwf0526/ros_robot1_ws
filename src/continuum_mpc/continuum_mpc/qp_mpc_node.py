import rclpy
from rclpy.node import Node

import numpy as np
import osqp
from scipy import sparse

from robot_interfaces.msg import (
    ContinuumState,
    ContinuumTarget,
    SafetyState,
    MotorPositionTarget,
    MotorPositionTargetArray,
    MotorState,
)


class QpMpcNode(Node):
    def __init__(self):
        super().__init__("qp_mpc_node")

        # ==========================
        # 1. MPC 参数
        # ==========================
        self.declare_parameter("control_rate_hz", 5.0)
        self.declare_parameter("max_delta_deg", 1.0)
        self.declare_parameter("default_speed_rad_s", 0.3)
        self.declare_parameter("motor_limit_deg", 360.0)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.max_delta_deg = float(self.get_parameter("max_delta_deg").value)
        self.default_speed_rad_s = float(self.get_parameter("default_speed_rad_s").value)
        self.motor_limit_deg = float(self.get_parameter("motor_limit_deg").value)

        # ==========================
        # 2. 状态与控制量定义
        # x = [imu1_roll, imu1_pitch, imu2_roll, imu2_pitch]
        # u = [dtheta1, dtheta2, dtheta3, dtheta4, dtheta5, dtheta6]
        # ==========================
        self.nx = 4
        self.nu = 6

        # A = I：第一版假设无控制时姿态保持不变
        self.A = np.eye(self.nx)

        # B：六根线缆对两段姿态的线性影响矩阵
        # 当前是根据线缆60度布局给出的初始估计
        # 后续可通过实验辨识替换
        self.B = np.array([
            # m1     m2     m3     m4     m5     m6
            [ 0.5,   0.0,  -1.0,   0.0,   0.5,   0.0],    # imu1_roll
            [-0.866, 0.0,   0.0,   0.0,   0.866, 0.0],    # imu1_pitch
            [ 0.0,   0.5,   0.0,  -1.0,   0.0,   0.5],    # imu2_roll
            [ 0.0,  -0.866, 0.0,   0.0,   0.0,   0.866],  # imu2_pitch
        ], dtype=float)

        # 缩放B，避免第一版控制过猛
        self.B = 0.05 * self.B

        # Q：姿态误差权重
        self.Q = np.diag([10.0, 10.0, 10.0, 10.0])

        # R：电机动作权重，越大越保守
        self.R = np.eye(self.nu) * 0.5

        # ==========================
        # 3. 数据缓存
        # ==========================
        self.current_state = None
        self.current_target = None
        self.current_safety = None
        self.motor_feedback_deg = {}

        # MPC维护零点坐标系下的绝对电机目标。
        self.theta_cmd_deg = np.zeros(self.nu)
        self.theta_cmd_initialized = False
        self.last_output_enabled = False

        # ==========================
        # 4. ROS2 订阅
        # ==========================
        self.create_subscription(
            ContinuumState,
            "/continuum/state",
            self.state_callback,
            10,
        )

        self.create_subscription(
            ContinuumTarget,
            "/continuum/target",
            self.target_callback,
            10,
        )

        self.create_subscription(
            SafetyState,
            "/continuum/safety_state",
            self.safety_callback,
            10,
        )

        self.create_subscription(
            MotorState,
            "/motor/state",
            self.motor_state_callback,
            50,
        )

        # ==========================
        # 5. ROS2 发布
        # ==========================
        self.command_pub = self.create_publisher(
            MotorPositionTargetArray,
            "/motor/position_target",
            10,
        )

        # ==========================
        # 6. 定时控制循环
        # ==========================
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )

        self.get_logger().info("qp_mpc_node started")

    def state_callback(self, msg: ContinuumState):
        # 保存当前连续体状态
        self.current_state = msg

    def target_callback(self, msg: ContinuumTarget):
        # 保存目标姿态
        self.current_target = msg

    def safety_callback(self, msg: SafetyState):
        # 保存安全状态
        self.current_safety = msg

    def motor_state_callback(self, msg: MotorState):
        motor_id = int(msg.motor_id)
        if 1 <= motor_id <= self.nu:
            self.motor_feedback_deg[motor_id] = float(msg.position_deg)

    def publish_target_disabled(self):
        if not self.last_output_enabled:
            return
        msg = MotorPositionTargetArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.enable = False
        msg.source = "qp_mpc"
        self.command_pub.publish(msg)
        self.last_output_enabled = False
        self.theta_cmd_initialized = False

    def solve_qp(self, x, x_ref):
        """
        单步 QP MPC：

        模型：
            x_next = A x + B u

        目标：
            让 x_next 接近 x_ref，同时让 u 不要太大

        约束：
            每周期电机增量不超过 max_delta_deg
            电机目标角度不超过 motor_limit_deg
        """

        # 代价函数：
        # J = ||A x + B u - x_ref||_Q^2 + ||u||_R^2
        H = self.B.T @ self.Q @ self.B + self.R
        g = self.B.T @ self.Q @ (self.A @ x - x_ref)

        # OSQP标准形式：
        # min 0.5 u^T P u + q^T u
        P = sparse.csc_matrix(2.0 * H)
        q = 2.0 * g

        # 约束1：单步电机增量
        lower_delta = -self.max_delta_deg * np.ones(self.nu)
        upper_delta = self.max_delta_deg * np.ones(self.nu)

        # 约束2：电机绝对角度
        lower_motor = -self.motor_limit_deg - self.theta_cmd_deg
        upper_motor = self.motor_limit_deg - self.theta_cmd_deg

        lower = np.maximum(lower_delta, lower_motor)
        upper = np.minimum(upper_delta, upper_motor)

        A_cons = sparse.eye(self.nu, format="csc")

        solver = osqp.OSQP()
        solver.setup(
            P=P,
            q=q,
            A=A_cons,
            l=lower,
            u=upper,
            verbose=False,
            polish=True,
        )

        result = solver.solve()

        if result.info.status_val not in (1, 2):
            self.get_logger().warn(f"OSQP failed: {result.info.status}")
            return None

        return result.x

    def control_loop(self):
        # ==========================
        # 1. 数据完整性检查
        # ==========================
        if self.current_state is None:
            self.publish_target_disabled()
            return

        if self.current_target is None:
            self.publish_target_disabled()
            return

        if self.current_safety is None:
            self.publish_target_disabled()
            return

        # ==========================
        # 2. 目标使能检查
        # ==========================
        if not self.current_target.enable:
            self.publish_target_disabled()
            return

        # ==========================
        # 3. 安全门控
        # ==========================
        if not self.current_safety.safe_to_control:
            self.get_logger().warn(
                f"QP MPC blocked by safety: {self.current_safety.status}"
            )
            self.publish_target_disabled()
            return

        if len(self.motor_feedback_deg) != self.nu:
            self.get_logger().warn("QP MPC waiting for all six motor feedback values")
            self.publish_target_disabled()
            return

        if not self.theta_cmd_initialized:
            self.theta_cmd_deg = np.array([
                self.motor_feedback_deg[motor_id]
                for motor_id in range(1, self.nu + 1)
            ], dtype=float)
            self.theta_cmd_initialized = True

        # ==========================
        # 4. 当前状态
        # ==========================
        s = self.current_state
        x = np.array([
            float(s.imu1_roll),
            float(s.imu1_pitch),
            float(s.imu2_roll),
            float(s.imu2_pitch),
        ])

        # ==========================
        # 5. 目标状态
        # ==========================
        t = self.current_target
        x_ref = np.array([
            float(t.section1_roll_target),
            float(t.section1_pitch_target),
            float(t.section2_roll_target),
            float(t.section2_pitch_target),
        ])

        # ==========================
        # 6. 求解QP
        # ==========================
        delta_theta = self.solve_qp(x, x_ref)

        if delta_theta is None:
            return

        # ==========================
        # 7. 更新电机目标角度
        # ==========================
        self.theta_cmd_deg += delta_theta

        # ==========================
        # 8. 生成多电机命令
        # ==========================
        msg = MotorPositionTargetArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.enable = True
        msg.source = "qp_mpc"
        msg.targets = []

        for i in range(self.nu):
            target = MotorPositionTarget()
            target.motor_id = i + 1
            target.position_deg = float(self.theta_cmd_deg[i])
            target.max_speed_rad_s = float(self.default_speed_rad_s)
            msg.targets.append(target)

        # ==========================
        # 9. 发布命令
        # ==========================
        self.command_pub.publish(msg)
        self.last_output_enabled = True

        error = x_ref - x
        self.get_logger().info(
            f"QP MPC | error={np.round(error, 2)} "
            f"delta={np.round(delta_theta, 3)} "
            f"theta={np.round(self.theta_cmd_deg, 2)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = QpMpcNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
