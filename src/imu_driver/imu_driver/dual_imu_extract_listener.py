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


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class DualImuExtractListener(Node):
    def __init__(self):
        super().__init__('dual_imu_extract_listener')

        self.declare_parameter('imu1_topic', '/imu1/data_raw')
        self.declare_parameter('imu2_topic', '/imu2/data_raw')
        self.declare_parameter('log_every_n', 10)

        self.imu1_topic = self.get_parameter('imu1_topic').value
        self.imu2_topic = self.get_parameter('imu2_topic').value
        self.log_every_n = int(self.get_parameter('log_every_n').value)

        self.counts = {'imu1': 0, 'imu2': 0}
        self.prev = {
            'imu1': {'t': None, 'wx': None, 'wy': None, 'wz': None},
            'imu2': {'t': None, 'wx': None, 'wy': None, 'wz': None},
        }

        self.imu1_sub = self.create_subscription(
            Imu,
            self.imu1_topic,
            lambda msg: self.handle_msg('imu1', msg),
            10,
        )
        self.imu2_sub = self.create_subscription(
            Imu,
            self.imu2_topic,
            lambda msg: self.handle_msg('imu2', msg),
            10,
        )

        self.get_logger().info(f'Subscribed IMU1: {self.imu1_topic}')
        self.get_logger().info(f'Subscribed IMU2: {self.imu2_topic}')

    def handle_msg(self, name: str, msg: Imu):
        self.counts[name] += 1
        count = self.counts[name]

        q = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration
        roll, pitch, yaw = quaternion_to_euler(q.w, q.x, q.y, q.z)

        now = stamp_to_sec(msg.header.stamp)
        last = self.prev[name]
        angular_acceleration = None

        if last['t'] is not None:
            dt = now - last['t']
            if dt > 0.0:
                angular_acceleration = (
                    (av.x - last['wx']) / dt,
                    (av.y - last['wy']) / dt,
                    (av.z - last['wz']) / dt,
                )

        last['t'] = now
        last['wx'] = av.x
        last['wy'] = av.y
        last['wz'] = av.z

        if self.log_every_n > 0 and count % self.log_every_n != 0:
            return

        if angular_acceleration is None:
            aa_text = '(nan, nan, nan)'
        else:
            aa_text = (
                f'({angular_acceleration[0]:.6f}, '
                f'{angular_acceleration[1]:.6f}, '
                f'{angular_acceleration[2]:.6f})'
            )

        self.get_logger().info(
            f'{name.upper()} #{count} '
            f'frame={msg.header.frame_id} '
            f'quat_wxyz=({q.w:.6f}, {q.x:.6f}, {q.y:.6f}, {q.z:.6f}) '
            f'rpy_rad=({roll:.6f}, {pitch:.6f}, {yaw:.6f}) '
            f'rpy_deg=({math.degrees(roll):.3f}, {math.degrees(pitch):.3f}, {math.degrees(yaw):.3f}) '
            f'angular_velocity_rad_s=({av.x:.6f}, {av.y:.6f}, {av.z:.6f}) '
            f'angular_acceleration_rad_s2={aa_text} '
            f'linear_acceleration_m_s2=({la.x:.6f}, {la.y:.6f}, {la.z:.6f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DualImuExtractListener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

