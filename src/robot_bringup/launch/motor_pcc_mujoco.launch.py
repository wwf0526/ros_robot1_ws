"""Start motor execution, PCC estimation and real-time MuJoCo display."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    motor_launch = bringup_share / "launch" / "motor_control.launch.py"
    calibration_file = bringup_share / "config" / "robot_calibration.yaml"
    hardware_config = bringup_share / "config" / "hardware_params.yaml"
    viewer_config = bringup_share / "config" / "mujoco_pcc_params.yaml"

    mujoco_share = Path(
        get_package_share_directory("continuum_mujoco_sim")
    )
    mesh_file = mujoco_share / "models" / "assets" / "hex_disk.stl"
    runtime_model_xml = (
        Path.home()
        / ".ros"
        / "ros_robot1"
        / "mujoco"
        / "continuum_kinematic_generated.xml"
    )

    use_mock = LaunchConfiguration("use_mock_hardware")
    require_safety = LaunchConfiguration("require_safety_state")
    start_enabled = LaunchConfiguration("start_enabled")
    use_mujoco = LaunchConfiguration("use_mujoco")
    start_imu = LaunchConfiguration("start_imu")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Use mock motors; false opens physical SocketCAN",
        ),
        DeclareLaunchArgument(
            "require_safety_state",
            default_value="false",
            description="Must be true for physical hardware",
        ),
        DeclareLaunchArgument(
            "start_enabled",
            default_value="false",
            description="Position controller must always start disarmed",
        ),
        DeclareLaunchArgument(
            "use_mujoco",
            default_value="true",
            description="Open the real-time MuJoCo PCC window",
        ),
        DeclareLaunchArgument(
            "start_imu",
            default_value="false",
            description="Start the physical dual-IMU serial node",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(motor_launch)),
            launch_arguments={
                "use_mock_hardware": use_mock,
                "require_safety_state": require_safety,
                "start_enabled": start_enabled,
            }.items(),
        ),
        Node(
            package="imu_driver",
            executable="dual_imu_serial_node",
            name="dual_imu_serial_node",
            parameters=[str(hardware_config)],
            output="screen",
            condition=IfCondition(start_imu),
        ),
        Node(
            package="continuum_state_estimator",
            executable="state_estimator_node",
            name="continuum_state_estimator",
            parameters=[
                {
                    "calibration_file": str(calibration_file),
                    "pcc_only_mode": ParameterValue(
                        use_mock,
                        value_type=bool,
                    ),
                },
            ],
            output="screen",
        ),
        Node(
            package="continuum_mujoco_sim",
            executable="mujoco_pcc_viewer",
            name="mujoco_pcc_viewer",
            parameters=[
                str(viewer_config),
                {
                    "model_xml": str(runtime_model_xml),
                    "regenerate_model": True,
                    "calibration_file": str(calibration_file),
                    "mesh_file": str(mesh_file),
                },
            ],
            output="screen",
            condition=IfCondition(use_mujoco),
        ),
    ])
