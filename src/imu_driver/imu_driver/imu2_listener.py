import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quaternion_to_euler(w, x, y, z):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class Imu2Listener(Node):
    def __init__(self):
        super().__init__('imu2_listener')
        self.declare_parameter('topic', '/imu2/data_raw')
        self.declare_parameter('log_every_n', 20)

        self.topic = self.get_parameter('topic').value
        self.log_every_n = int(self.get_parameter('log_every_n').value)
        self.count = 0

        self.subscription = self.create_subscription(
            Imu,
            self.topic,
            self.imu_callback,
            10,
        )
        self.get_logger().info(f'Subscribed IMU2: {self.topic}')

    def imu_callback(self, msg: Imu):
        self.count += 1
        if self.log_every_n > 0 and self.count % self.log_every_n != 0:
            return

        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.w, q.x, q.y, q.z)
        av = msg.angular_velocity
        la = msg.linear_acceleration

        self.get_logger().info(
            f'IMU2 #{self.count} frame={msg.header.frame_id} '
            f'rpy_rad=({roll:.4f}, {pitch:.4f}, {yaw:.4f}) '
            f'gyro=({av.x:.4f}, {av.y:.4f}, {av.z:.4f}) '
            f'acc=({la.x:.4f}, {la.y:.4f}, {la.z:.4f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Imu2Listener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

