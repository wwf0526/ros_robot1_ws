"""Deterministic six-motor mock hardware for closed-loop and MPC testing."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from robot_interfaces.msg import (
    MotorCommand,
    MotorCommandArray,
    MotorPositionControlState,
    MotorState,
)
from robot_interfaces.srv import (
    ClearEmergencyStop,
    EmergencyStop,
    SetZero,
)


MODE_STOP = 0
MODE_RELATIVE_POSITION = 2


class MockMotorHardwareNode(Node):
    """Emulate mode-2 relative moves and encoder feedback without CAN."""

    def __init__(self):
        super().__init__("motor_node")
        self.declare_parameter("motor_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("secondary_ids", [4, 5, 6])
        self.declare_parameter("command_topic", "/motor/raw_command")
        self.declare_parameter(
            "command_array_topic",
            "/motor/raw_command_array",
        )
        self.declare_parameter("state_topic", "/motor/state")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("max_relative_command_deg", 5.0)
        self.declare_parameter("max_speed_rad_s", 1.0)
        self.declare_parameter("controller_state_timeout_sec", 0.5)

        self.motor_ids = [
            int(value) for value in self.get_parameter("motor_ids").value
        ]
        self.secondary_ids = {
            int(value) for value in self.get_parameter("secondary_ids").value
        }
        self.command_topic = str(
            self.get_parameter("command_topic").value
        )
        self.command_array_topic = str(
            self.get_parameter("command_array_topic").value
        )
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.max_relative_command_deg = float(
            self.get_parameter("max_relative_command_deg").value
        )
        self.max_speed_rad_s = float(
            self.get_parameter("max_speed_rad_s").value
        )
        self.controller_state_timeout_sec = float(
            self.get_parameter("controller_state_timeout_sec").value
        )

        self.raw_deg = {mid: 0.0 for mid in self.motor_ids}
        self.zero_offset_deg = {mid: 0.0 for mid in self.motor_ids}
        self.target_raw_deg = {mid: 0.0 for mid in self.motor_ids}
        self.speed_rad_s = {mid: 0.0 for mid in self.motor_ids}
        self.reached = {mid: True for mid in self.motor_ids}
        self.estop_active = False
        self.controller_enabled = False
        self.controller_batch_in_flight = False
        self.last_controller_state_time: float | None = None
        self.last_update_time = time.monotonic()

        self.create_subscription(
            MotorCommand,
            self.command_topic,
            self.command_callback,
            10,
        )
        self.create_subscription(
            MotorCommandArray,
            self.command_array_topic,
            self.command_array_callback,
            10,
        )
        self.create_subscription(
            MotorPositionControlState,
            "/motor/position_control_state",
            self.controller_state_callback,
            10,
        )
        self.state_pub = self.create_publisher(
            MotorState,
            self.state_topic,
            50,
        )
        self.estop_pub = self.create_publisher(
            Bool,
            "/motor/emergency_stop_active",
            10,
        )

        self.create_service(
            SetZero,
            "/motor/set_zero",
            self.set_zero_callback,
        )
        self.create_service(
            EmergencyStop,
            "/motor/emergency_stop",
            self.emergency_stop_callback,
        )
        self.create_service(
            ClearEmergencyStop,
            "/motor/clear_estop",
            self.clear_estop_callback,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.update_and_publish,
        )
        self.get_logger().info(
            "Mock six-motor hardware started; no physical CAN commands are sent"
        )

    def _validate(self, command: MotorCommand):
        motor_id = int(command.motor_id)
        if motor_id not in self.motor_ids:
            raise ValueError(f"unknown motor id: {motor_id}")
        mode = int(command.mode)
        if mode == MODE_STOP:
            return motor_id, mode, 0.0, 0.0
        if mode != MODE_RELATIVE_POSITION:
            raise ValueError(f"unsupported raw mode: {mode}")
        delta = float(command.target_deg)
        speed = float(command.speed_rad_s)
        if not math.isfinite(delta) or abs(delta) > self.max_relative_command_deg:
            raise ValueError(f"motor {motor_id}: invalid relative delta")
        if not math.isfinite(speed) or not 0.0 < speed <= self.max_speed_rad_s:
            raise ValueError(f"motor {motor_id}: invalid speed")
        return motor_id, mode, delta, speed

    def _apply(self, validated) -> None:
        motor_id, mode, delta, speed = validated
        if mode == MODE_STOP:
            self.target_raw_deg[motor_id] = self.raw_deg[motor_id]
            self.speed_rad_s[motor_id] = 0.0
            self.reached[motor_id] = True
            return
        controller_authorized = bool(
            self.last_controller_state_time is not None
            and time.monotonic() - self.last_controller_state_time
            <= self.controller_state_timeout_sec
            and self.controller_enabled
        )
        if self.estop_active or not controller_authorized:
            return
        self.target_raw_deg[motor_id] = self.raw_deg[motor_id] + delta
        self.speed_rad_s[motor_id] = speed
        self.reached[motor_id] = False

    def command_callback(self, msg: MotorCommand) -> None:
        try:
            self._apply(self._validate(msg))
        except ValueError as exc:
            self.get_logger().error(f"Mock command rejected: {exc}")

    def command_array_callback(self, msg: MotorCommandArray) -> None:
        if not msg.commands:
            return
        modes = [int(command.mode) for command in msg.commands]
        if MODE_STOP in modes and any(mode != MODE_STOP for mode in modes):
            self.get_logger().error(
                "Mock mixed stop/move command arrays are rejected"
            )
            return
        ids = [int(command.motor_id) for command in msg.commands]
        if len(ids) != len(set(ids)):
            self.get_logger().error("Mock command array has duplicate motor ids")
            return
        try:
            validated = [self._validate(command) for command in msg.commands]
        except ValueError as exc:
            self.get_logger().error(f"Mock command array rejected: {exc}")
            return
        for item in validated:
            self._apply(item)

    def controller_state_callback(
        self,
        msg: MotorPositionControlState,
    ) -> None:
        self.controller_enabled = bool(msg.controller_enabled)
        self.controller_batch_in_flight = bool(msg.batch_in_flight)
        self.last_controller_state_time = time.monotonic()

    def update_and_publish(self) -> None:
        now = time.monotonic()
        dt = max(0.0, min(now - self.last_update_time, 0.1))
        self.last_update_time = now

        controller_authorized = bool(
            self.last_controller_state_time is not None
            and now - self.last_controller_state_time
            <= self.controller_state_timeout_sec
            and self.controller_enabled
            and not self.estop_active
        )
        if not controller_authorized:
            for motor_id in self.motor_ids:
                self.target_raw_deg[motor_id] = self.raw_deg[motor_id]
                self.speed_rad_s[motor_id] = 0.0
                self.reached[motor_id] = True

        for motor_id in self.motor_ids:
            error = self.target_raw_deg[motor_id] - self.raw_deg[motor_id]
            max_step = math.degrees(self.speed_rad_s[motor_id]) * dt
            if abs(error) <= max(max_step, 1.0e-6):
                self.raw_deg[motor_id] = self.target_raw_deg[motor_id]
                self.speed_rad_s[motor_id] = 0.0
                self.reached[motor_id] = True
            elif max_step > 0.0:
                self.raw_deg[motor_id] += math.copysign(max_step, error)
                self.reached[motor_id] = False

            msg = MotorState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "motor_zero_frame"
            msg.motor_id = motor_id
            msg.raw_deg = float(self.raw_deg[motor_id])
            msg.position_deg = float(
                self.raw_deg[motor_id] - self.zero_offset_deg[motor_id]
            )
            msg.speed_rad_s = float(self.speed_rad_s[motor_id])
            msg.reached = bool(self.reached[motor_id])
            msg.channel = (
                "mock_can1" if motor_id in self.secondary_ids else "mock_can0"
            )
            self.state_pub.publish(msg)

        estop = Bool()
        estop.data = bool(self.estop_active)
        self.estop_pub.publish(estop)

    def set_zero_callback(self, request, response):
        state_fresh = bool(
            self.last_controller_state_time is not None
            and time.monotonic() - self.last_controller_state_time
            <= self.controller_state_timeout_sec
        )
        if not state_fresh:
            response.success = False
            response.message = (
                "Mock set-zero rejected: fresh disabled controller state required"
            )
            return response
        if self.controller_enabled or self.controller_batch_in_flight:
            response.success = False
            response.message = "Mock set-zero rejected: disable controller first"
            return response
        requested = [int(mid) for mid in request.motor_ids]
        if not requested:
            requested = list(self.motor_ids)
        unknown = sorted(set(requested) - set(self.motor_ids))
        if unknown:
            response.success = False
            response.message = f"Unknown motor ids: {unknown}"
            return response
        for motor_id in requested:
            self.zero_offset_deg[motor_id] = self.raw_deg[motor_id]
        response.success = True
        response.message = f"Mock zero set for motors {requested}"
        return response

    def emergency_stop_callback(self, request, response):
        del request
        self.estop_active = True
        for motor_id in self.motor_ids:
            self.target_raw_deg[motor_id] = self.raw_deg[motor_id]
            self.speed_rad_s[motor_id] = 0.0
            self.reached[motor_id] = True
        response.success = True
        response.message = "Mock emergency stop activated"
        return response

    def clear_estop_callback(self, request, response):
        del request
        self.estop_active = False
        response.success = True
        response.message = "Mock emergency stop cleared"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockMotorHardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
