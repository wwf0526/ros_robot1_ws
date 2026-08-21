"""Launch the final absolute-position execution layer with real or mock CAN."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    hardware_config = str(bringup_share / "config" / "hardware_params.yaml")
    controller_config = str(
        bringup_share / "config" / "motor_control_params.yaml"
    )
    calibration_file = str(
        bringup_share / "config" / "robot_calibration.yaml"
    )

    use_mock = LaunchConfiguration("use_mock_hardware")
    require_safety = LaunchConfiguration("require_safety_state")
    start_enabled = LaunchConfiguration("start_enabled")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Use deterministic mock motors instead of physical CAN",
        ),
        DeclareLaunchArgument(
            "require_safety_state",
            default_value="false",
            description=(
                "Require fresh /continuum/safety_state. Use true for real hardware"
            ),
        ),
        DeclareLaunchArgument(
            "start_enabled",
            default_value="false",
            description="Keep false for both commissioning and real hardware",
        ),
        Node(
            package="motor_can_driver",
            executable="mock_motor_hardware_node",
            name="motor_node",
            parameters=[hardware_config],
            output="screen",
            condition=IfCondition(use_mock),
        ),
        Node(
            package="motor_can_driver",
            executable="motor_node",
            name="motor_node",
            parameters=[
                hardware_config,
                {
                    "require_safety_state": ParameterValue(
                        require_safety,
                        value_type=bool,
                    ),
                },
            ],
            output="screen",
            condition=UnlessCondition(use_mock),
        ),
        Node(
            package="motor_can_driver",
            executable="motor_position_controller",
            name="motor_position_controller",
            parameters=[
                controller_config,
                {
                    "calibration_file": calibration_file,
                    "require_safety_state": ParameterValue(
                        require_safety,
                        value_type=bool,
                    ),
                    "start_enabled": ParameterValue(
                        start_enabled,
                        value_type=bool,
                    ),
                },
            ],
            output="screen",
        ),
    ])
