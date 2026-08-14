import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class PccSectionDemo(Node):
    def __init__(self):
        super().__init__('pcc_section_demo')

        self.pub = self.create_publisher(Float64MultiArray, '/pcc/section_state', 10)

        self.section_length = 0.2925
        self.t0 = self.get_clock().now()

        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info(
            'Publishing demo /pcc/section_state: '
            '[sec2_theta, sec2_phi, sec2_l, sec1_theta, sec1_phi, sec1_l]'
        )

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        # section2 小幅弯曲
        sec2_theta = 0.15 * math.sin(0.8 * t)
        sec2_phi = math.radians(120.0)

        # section1 小幅弯曲
        sec1_theta = 0.20 * math.sin(1.1 * t)
        sec1_phi = math.radians(60.0)

        # 模拟轻微伸缩，最大压缩约 5 mm
        sec2_l = self.section_length - 0.003 * abs(math.sin(0.7 * t))
        sec1_l = self.section_length - 0.005 * abs(math.sin(0.9 * t))

        msg = Float64MultiArray()
        msg.data = [
            sec2_theta, sec2_phi, sec2_l,
            sec1_theta, sec1_phi, sec1_l,
        ]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PccSectionDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
