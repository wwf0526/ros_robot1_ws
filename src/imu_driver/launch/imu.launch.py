from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_bringup')
    config_file = os.path.join(pkg_dir, 'config', 'hardware_params.yaml')

    imu_node = Node(
        package='imu_driver',
        executable='dual_imu_serial_node',
        name='dual_imu_serial_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([imu_node])
