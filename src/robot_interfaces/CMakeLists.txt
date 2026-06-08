cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MotorCommand.msg"
    "msg/MotorState.msg"
    "msg/MotorCommandArray.msg"
    "msg/TendonState.msg"
    "msg/ContinuumState.msg"
    "msg/ZeroCheckState.msg"
    "srv/SetZero.srv"
    "srv/HomeMotors.srv"
    "srv/EmergencyStop.srv"
    "srv/ClearEmergencyStop.srv"
    DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
