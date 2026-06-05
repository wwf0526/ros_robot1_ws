import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time


G_TO_MPS2 = 9.80665


class DualImuSerialNode(Node):
    def __init__(self):
        super().__init__('dual_imu_serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 200.0)
        self.declare_parameter('imu1_topic', '/imu1/data_raw')
        self.declare_parameter('imu2_topic', '/imu2/data_raw')
        self.declare_parameter('imu1_frame_id', 'imu1_link')
        self.declare_parameter('imu2_frame_id', 'imu2_link')
        self.declare_parameter('status_period', 5.0)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.imu1_topic = self.get_parameter('imu1_topic').value
        self.imu2_topic = self.get_parameter('imu2_topic').value
        self.imu1_frame_id = self.get_parameter('imu1_frame_id').value
        self.imu2_frame_id = self.get_parameter('imu2_frame_id').value
        self.status_period = float(self.get_parameter('status_period').value)

        self.imu_publishers = {
            1: self.create_publisher(Imu, self.imu1_topic, 10),
            2: self.create_publisher(Imu, self.imu2_topic, 10),
        }
        self.frame_ids = {
            1: self.imu1_frame_id,
            2: self.imu2_frame_id,
        }
        self.rx_buffer = bytearray()
        self.published_counts = {1: 0, 2: 0}
        self.bad_line_count = 0
        self.last_status_time = time.monotonic()

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            self.get_logger().info(
                f'IMU serial opened: {self.port}, baudrate={self.baudrate}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            self.ser = None

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.ser is None:
            return

        waiting = self.ser.in_waiting
        if waiting <= 0:
            return

        try:
            self.rx_buffer.extend(self.ser.read(waiting))

            if len(self.rx_buffer) > 4096:
                self.get_logger().warn('Serial receive buffer overflow; clearing buffer')
                self.rx_buffer.clear()
                return

            # Parse only complete newline-terminated frames. Serial reads can
            # start in the middle of a CSV line, so the first partial line is skipped.
            while b'\n' in self.rx_buffer:
                raw_line, _, rest = self.rx_buffer.partition(b'\n')
                self.rx_buffer = bytearray(rest)

                line = raw_line.decode('utf-8', errors='ignore').strip()
                if not line or line.startswith('#'):
                    continue

                self.parse_and_publish(line)

            self.report_status()

        except Exception as e:
            self.get_logger().warn(f'IMU parse error: {e}')
            return

    def report_status(self):
        if self.status_period <= 0.0:
            return

        now = time.monotonic()
        if now - self.last_status_time < self.status_period:
            return

        self.last_status_time = now
        self.get_logger().info(
            f'published imu1={self.published_counts[1]} '
            f'imu2={self.published_counts[2]} '
            f'bad_lines={self.bad_line_count}'
        )

    def parse_and_publish(self, line: str):
        # Firmware CSV format:
        # imu_id,ms,frames,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,
        # q0,q1,q2,q3,checksum_err,drop_err,rx_overrun
        if not (line.startswith('1,') or line.startswith('2,')):
            self.bad_line_count += 1
            return

        parts = line.split(',')
        if len(parts) != 22:
            self.bad_line_count += 1
            return

        try:
            imu_id = int(float(parts[0]))
            values = [float(x) for x in parts]
        except ValueError:
            self.bad_line_count += 1
            return

        publisher = self.imu_publishers.get(imu_id)
        if publisher is None:
            self.bad_line_count += 1
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_ids[imu_id]

        # Firmware outputs acceleration in g. sensor_msgs/Imu expects m/s^2.
        msg.linear_acceleration.x = values[3] * G_TO_MPS2
        msg.linear_acceleration.y = values[4] * G_TO_MPS2
        msg.linear_acceleration.z = values[5] * G_TO_MPS2

        msg.angular_velocity.x = values[6]
        msg.angular_velocity.y = values[7]
        msg.angular_velocity.z = values[8]

        msg.orientation.w = values[15]
        msg.orientation.x = values[16]
        msg.orientation.y = values[17]
        msg.orientation.z = values[18]

        # Unknown covariance. ROS convention: first element -1 means unavailable.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        publisher.publish(msg)
        self.published_counts[imu_id] += 1


def main(args=None):
    rclpy.init(args=args)
    node = DualImuSerialNode()
    try:
        rclpy.spin(node)
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

