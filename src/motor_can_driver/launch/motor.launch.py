from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "hardware_params.yaml",
    )

    motor_node = Node(
        package="motor_can_driver",
        executable="motor_node",
        name="motor_node",
        parameters=[config_file],
        output="screen",
    )

    return LaunchDescription([motor_node])
