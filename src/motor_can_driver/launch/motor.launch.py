"""Compatibility entry point for the final real-hardware motor stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    final_launch = bringup_share / "launch" / "motor_control.launch.py"
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(final_launch)),
        launch_arguments={
            "use_mock_hardware": "false",
            "require_safety_state": "true",
            "start_enabled": "false",
        }.items(),
    )
    return LaunchDescription([include])
