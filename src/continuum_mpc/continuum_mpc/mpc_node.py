import rclpy
from rclpy.node import Node

import numpy as np

from robot_interfaces.msg import (
    ContinuumState,
    ContinuumTarget,
    SafetyState,
    MotorCommand,
    MotorCommandArray,
)

# ==========================
# 1. 工具函数
# ==========================

def clamp(x, xmin, xmax):
    """
    限幅函数：
    防止电机单步变化过大导致振荡
    """
    return max(xmin, min(x, xmax))


# ==========================
# 2. MPC节点主体
# ==========================

class LinearMPC(Node):

    def __init__(self):
        super().__init__("linear_mpc")

        # ==========================
        # 参数声明（ROS2标准方式）
        # ==========================

        self.declare_parameter("alpha", 0.5)  # 曲率增益
        self.declare_parameter("dt_gain", 1.0)
        self.declare_parameter("max_delta", 1.0)  # 每步最大电机变化

        self.alpha = self.get_parameter("alpha").value
        self.dt_gain = self.get_parameter("dt_gain").value
        self.max_delta = self.get_parameter("max_delta").value

        # ==========================
        # 六绳角度（你的真实结构）
        # ==========================

        angles_deg = [300, 240, 180, 120, 60, 0]
        angles = np.deg2rad(angles_deg)

        # ==========================
        # B矩阵（6绳 → 曲率空间）
        # ==========================

        # x = [kx, ky]
        self.B = np.array([
            np.cos(angles),
            np.sin(angles)
        ])  # 2×6

        # ==========================
        # 状态缓存
        # ==========================

        self.current_state = None
        self.target_state = None
        self.safety_state = None

        # ==========================
        # ROS2订阅
        # ==========================

        self.create_subscription(
            ContinuumState,
            "/continuum/state",
            self.state_callback,
            10
        )

        self.create_subscription(
            ContinuumTarget,
            "/continuum/target",
            self.target_callback,
            10
        )

        self.create_subscription(
            SafetyState,
            "/continuum/safety_state",
            self.safety_callback,
            10
        )

        # ==========================
        # ROS2发布
        # ==========================

        self.pub_cmd = self.create_publisher(
            MotorCommandArray,
            "/motor/command_array",
            10
        )

        # ==========================
        # 控制周期（10Hz）
        # ==========================

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Linear MPC Node Started")

    # ==========================
    # 回调函数
    # ==========================

    def state_callback(self, msg):
        """
        当前状态：
        从 IMU + PCC 得到 kx, ky
        """
        self.current_state = msg

    def target_callback(self, msg):
        """
        目标状态：
        MPC目标输入
        """
        self.target_state = msg

    def safety_callback(self, msg):
        """
        安全状态：
        决定是否允许控制
        """
        self.safety_state = msg

    # ==========================
    # 核心控制逻辑
    # ==========================

    def control_loop(self):

        # --------------------------
        # 1. 数据完整性检查
        # --------------------------

        if self.current_state is None:
            return

        if self.target_state is None:
            return

        if self.safety_state is None:
            return

        if not self.safety_state.safe_to_control:
            self.get_logger().warn("Safety blocked MPC")
            return

        # --------------------------
        # 2. 构造状态向量 x
        # --------------------------

        x = np.array([
            self.current_state.section1_kx,
            self.current_state.section1_ky
        ])

        # --------------------------
        # 3. 构造目标 x_ref
        # --------------------------

        x_ref = np.array([
            self.target_state.section1_roll_target,   # 简化映射kx
            self.target_state.section1_pitch_target    # 简化映射ky
        ])

        # ==========================
        # 4. 误差计算
        # ==========================

        error = x_ref - x

        # ==========================
        # 5. MPC核心（最小二乘解）
        # ==========================

        # u = B^T * error
        # 物理意义：误差投影到6个电机方向

        u = self.B.T @ error

        # ==========================
        # 6. 增益调节
        # ==========================

        u = self.alpha * u

        # ==========================
        # 7. 限幅（安全关键）
        # ==========================

        u = np.clip(u, -self.max_delta, self.max_delta)

        # ==========================
        # 8. 转换为电机命令
        # ==========================

        cmds = []

        for i in range(6):

            cmd = MotorCommand()
            cmd.motor_id = i + 1
            cmd.mode = 2  # position mode

            # 累加控制（关键）
            cmd.target_deg = float(u[i])

            cmd.speed_rad_s = 0.3

            cmds.append(cmd)

        msg = MotorCommandArray()
        msg.commands = cmds

        # ==========================
        # 9. 发布控制指令
        # ==========================

        self.pub_cmd.publish(msg)

        # ==========================
        # 10. 日志输出
        # ==========================

        self.get_logger().info(
            f"MPC | error=({error[0]:.3f},{error[1]:.3f}) "
            f"u={np.round(u,3)}"
        )


# ==========================
# 入口函数（ROS2标准）
# ==========================

def main(args=None):
    rclpy.init(args=args)
    node = LinearMPC()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
