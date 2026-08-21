"""Safety-gated raw MS42DDC hardware node with asynchronous feedback."""

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
    SafetyState,
)
from robot_interfaces.srv import (
    ClearEmergencyStop,
    EmergencyStop,
    SetZero,
)

from .ms42ddc_driver import MS42DDCDriver


MODE_STOP = 0
MODE_RELATIVE_POSITION = 2


class MotorNode(Node):
    """Own the physical CAN buses and enforce the final raw-command boundary."""

    def __init__(self):
        super().__init__("motor_node")

        self.declare_parameter("interface", "socketcan")
        self.declare_parameter("primary_channel", "can0")
        self.declare_parameter("secondary_channel", "can1")
        self.declare_parameter("bitrate", 1000000)
        self.declare_parameter("feedback_id", 10)
        self.declare_parameter("microstep", 32)
        self.declare_parameter("feedback_service_timeout_sec", 0.1)
        self.declare_parameter("motor_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("secondary_ids", [4, 5, 6])
        self.declare_parameter("command_topic", "/motor/raw_command")
        self.declare_parameter(
            "command_array_topic",
            "/motor/raw_command_array",
        )
        self.declare_parameter("state_topic", "/motor/state")
        self.declare_parameter("feedback_request_rate_hz", 300.0)
        self.declare_parameter("feedback_publish_rate_hz", 200.0)
        self.declare_parameter("safety_timeout_sec", 0.25)
        self.declare_parameter("controller_state_timeout_sec", 0.5)
        self.declare_parameter("require_safety_state", True)
        self.declare_parameter("max_relative_command_deg", 5.0)
        self.declare_parameter("max_speed_rad_s", 1.0)
        self.declare_parameter("zero_file", "/tmp/ms42ddc_zero_offsets.json")

        self.interface = str(self.get_parameter("interface").value)
        self.primary_channel = str(
            self.get_parameter("primary_channel").value
        )
        self.secondary_channel = str(
            self.get_parameter("secondary_channel").value
        )
        self.bitrate = int(self.get_parameter("bitrate").value)
        self.feedback_id = int(self.get_parameter("feedback_id").value)
        self.microstep = int(self.get_parameter("microstep").value)
        self.feedback_service_timeout_sec = float(
            self.get_parameter("feedback_service_timeout_sec").value
        )
        self.motor_ids = [
            int(value) for value in self.get_parameter("motor_ids").value
        ]
        self.secondary_ids = [
            int(value) for value in self.get_parameter("secondary_ids").value
        ]
        self.command_topic = str(
            self.get_parameter("command_topic").value
        )
        self.command_array_topic = str(
            self.get_parameter("command_array_topic").value
        )
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.feedback_request_rate_hz = float(
            self.get_parameter("feedback_request_rate_hz").value
        )
        self.feedback_publish_rate_hz = float(
            self.get_parameter("feedback_publish_rate_hz").value
        )
        self.safety_timeout_sec = float(
            self.get_parameter("safety_timeout_sec").value
        )
        self.controller_state_timeout_sec = float(
            self.get_parameter("controller_state_timeout_sec").value
        )
        self.require_safety_state = bool(
            self.get_parameter("require_safety_state").value
        )
        self.max_relative_command_deg = float(
            self.get_parameter("max_relative_command_deg").value
        )
        self.max_speed_rad_s = float(
            self.get_parameter("max_speed_rad_s").value
        )
        self.zero_file = str(self.get_parameter("zero_file").value)

        if self.feedback_request_rate_hz <= 0.0:
            raise ValueError("feedback_request_rate_hz must be positive")
        if self.feedback_publish_rate_hz <= 0.0:
            raise ValueError("feedback_publish_rate_hz must be positive")

        self.estop_active = False
        self.safety_safe = False
        self.safety_status = "not received"
        self.last_safety_time: float | None = None
        self._safety_stop_sent = False
        self._feedback_request_index = 0
        self.controller_enabled = False
        self.controller_batch_in_flight = False
        self.last_controller_state_time: float | None = None
        self._last_published_sequence = {
            motor_id: 0 for motor_id in self.motor_ids
        }

        self.driver = MS42DDCDriver(
            interface=self.interface,
            primary_channel=self.primary_channel,
            secondary_channel=self.secondary_channel,
            bitrate=self.bitrate,
            feedback_id=self.feedback_id,
            microstep=self.microstep,
            timeout=self.feedback_service_timeout_sec,
            motor_ids=self.motor_ids,
            secondary_ids=self.secondary_ids,
            zero_file=self.zero_file,
        )
        self.driver.open()

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
            SafetyState,
            "/continuum/safety_state",
            self.safety_callback,
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
        self.estop_state_pub = self.create_publisher(
            Bool,
            "/motor/emergency_stop_active",
            10,
        )

        self.set_zero_srv = self.create_service(
            SetZero,
            "/motor/set_zero",
            self.set_zero_callback,
        )
        self.estop_srv = self.create_service(
            EmergencyStop,
            "/motor/emergency_stop",
            self.emergency_stop_callback,
        )
        self.clear_estop_srv = self.create_service(
            ClearEmergencyStop,
            "/motor/clear_estop",
            self.clear_estop_callback,
        )

        self.feedback_request_timer = self.create_timer(
            1.0 / self.feedback_request_rate_hz,
            self.request_next_feedback,
        )
        self.feedback_publish_timer = self.create_timer(
            1.0 / self.feedback_publish_rate_hz,
            self.publish_new_feedback,
        )
        self.safety_timer = self.create_timer(0.05, self.safety_watchdog)
        self.estop_timer = self.create_timer(0.1, self.publish_estop_state)

        self.get_logger().info(
            "Motor CAN opened with threaded feedback: "
            f"{self.primary_channel}, {self.secondary_channel}; "
            f"raw topic={self.command_array_topic}"
        )

    def safety_callback(self, msg: SafetyState) -> None:
        self.safety_safe = bool(msg.safe_to_control)
        self.safety_status = str(msg.status)
        self.last_safety_time = time.monotonic()

    def controller_state_callback(
        self,
        msg: MotorPositionControlState,
    ) -> None:
        self.controller_enabled = bool(msg.controller_enabled)
        self.controller_batch_in_flight = bool(msg.batch_in_flight)
        self.last_controller_state_time = time.monotonic()

    def _safety_valid(self) -> bool:
        if not self.require_safety_state:
            return True
        return bool(
            self.last_safety_time is not None
            and time.monotonic() - self.last_safety_time
            <= self.safety_timeout_sec
            and self.safety_safe
        )

    def _movement_allowed(self) -> bool:
        controller_authorized = bool(
            self.last_controller_state_time is not None
            and time.monotonic() - self.last_controller_state_time
            <= self.controller_state_timeout_sec
            and self.controller_enabled
        )
        return bool(
            not self.estop_active
            and self._safety_valid()
            and controller_authorized
        )

    def _validate_move(self, msg: MotorCommand) -> tuple[int, float, float]:
        motor_id = int(msg.motor_id)
        if motor_id not in self.motor_ids:
            raise ValueError(f"unknown motor id: {motor_id}")
        if int(msg.mode) != MODE_RELATIVE_POSITION:
            raise ValueError(
                f"unsupported raw mode {int(msg.mode)}; only 0=stop and "
                "2=relative position are allowed"
            )
        delta_deg = float(msg.target_deg)
        speed_rad_s = float(msg.speed_rad_s)
        if not math.isfinite(delta_deg):
            raise ValueError(f"motor {motor_id}: non-finite relative delta")
        if abs(delta_deg) > self.max_relative_command_deg:
            raise ValueError(
                f"motor {motor_id}: relative delta {delta_deg:.6f} exceeds "
                f"limit {self.max_relative_command_deg:.6f} deg"
            )
        if not math.isfinite(speed_rad_s) or speed_rad_s <= 0.0:
            raise ValueError(f"motor {motor_id}: invalid speed")
        if speed_rad_s > self.max_speed_rad_s:
            raise ValueError(
                f"motor {motor_id}: speed {speed_rad_s:.6f} exceeds "
                f"limit {self.max_speed_rad_s:.6f} rad/s"
            )
        return motor_id, delta_deg, speed_rad_s

    def command_callback(self, msg: MotorCommand) -> None:
        if int(msg.mode) == MODE_STOP:
            try:
                self.driver.stop(int(msg.motor_id))
            except Exception as exc:
                self.get_logger().error(f"stop command failed: {exc}")
            return
        if not self._movement_allowed():
            self.get_logger().warn(
                f"Raw move rejected: estop={self.estop_active}, "
                f"safety={self.safety_status}, "
                f"controller_enabled={self.controller_enabled}"
            )
            return
        try:
            motor_id, delta_deg, speed_rad_s = self._validate_move(msg)
            self.driver.send_position(motor_id, delta_deg, speed_rad_s)
        except Exception as exc:
            self.get_logger().error(f"raw motor command rejected/failed: {exc}")

    def command_array_callback(self, msg: MotorCommandArray) -> None:
        if not msg.commands:
            return
        if all(int(command.mode) == MODE_STOP for command in msg.commands):
            try:
                self.driver.stop_all()
            except Exception as exc:
                self.get_logger().error(f"stop-all failed: {exc}")
            return
        if any(int(command.mode) == MODE_STOP for command in msg.commands):
            self.get_logger().error("mixed stop/move raw arrays are rejected")
            return
        if not self._movement_allowed():
            self.get_logger().warn(
                f"Raw move array rejected: estop={self.estop_active}, "
                f"safety={self.safety_status}, "
                f"controller_enabled={self.controller_enabled}"
            )
            return

        ids = [int(command.motor_id) for command in msg.commands]
        if len(ids) != len(set(ids)):
            self.get_logger().error("raw move array contains duplicate motor ids")
            return
        try:
            validated = [self._validate_move(command) for command in msg.commands]
            for motor_id, delta_deg, speed_rad_s in validated:
                self.driver.send_position(motor_id, delta_deg, speed_rad_s)
        except Exception as exc:
            self.get_logger().error(f"raw move array rejected/failed: {exc}")
            try:
                self.driver.stop_all()
            except Exception:
                pass

    def request_next_feedback(self) -> None:
        motor_id = self.motor_ids[self._feedback_request_index]
        self._feedback_request_index = (
            self._feedback_request_index + 1
        ) % len(self.motor_ids)
        try:
            self.driver.request_status(motor_id)
        except Exception as exc:
            self.get_logger().warn(
                f"feedback request motor {motor_id} failed: {exc}"
            )

    def publish_new_feedback(self) -> None:
        for motor_id in self.motor_ids:
            cached = self.driver.latest_status(motor_id)
            if cached is None:
                continue
            status, _, sequence = cached
            if sequence <= self._last_published_sequence[motor_id]:
                continue
            self._last_published_sequence[motor_id] = sequence

            msg = MotorState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "motor_zero_frame"
            msg.motor_id = int(status.motor_id)
            msg.raw_deg = float(status.raw_deg)
            msg.position_deg = float(self.driver.relative_position(status))
            msg.speed_rad_s = float(status.speed_rad_s)
            msg.reached = bool(status.reached)
            msg.channel = self.driver.channel_for_motor(motor_id)
            self.state_pub.publish(msg)

    def safety_watchdog(self) -> None:
        if self._movement_allowed():
            self._safety_stop_sent = False
            return
        if self._safety_stop_sent:
            return
        try:
            self.driver.stop_all()
            self._safety_stop_sent = True
            self.get_logger().warn(
                "Execution watchdog stopped all motors: "
                f"safety={self.safety_status}, "
                f"controller_enabled={self.controller_enabled}"
            )
        except Exception as exc:
            self.get_logger().error(f"Safety watchdog stop failed: {exc}")

    def set_zero_callback(self, request, response):
        if self.estop_active:
            response.success = False
            response.message = "Set-zero rejected: emergency stop active"
            return response
        controller_state_fresh = bool(
            self.last_controller_state_time is not None
            and time.monotonic() - self.last_controller_state_time
            <= self.controller_state_timeout_sec
        )
        if not controller_state_fresh:
            response.success = False
            response.message = (
                "Set-zero rejected: fresh disabled controller state required"
            )
            return response
        if self.controller_enabled or self.controller_batch_in_flight:
            response.success = False
            response.message = (
                "Set-zero rejected: disable the controller and stop motion first"
            )
            return response
        requested = [int(mid) for mid in request.motor_ids]
        if not requested:
            requested = list(self.motor_ids)
        unknown = sorted(set(requested) - set(self.motor_ids))
        if unknown:
            response.success = False
            response.message = f"Unknown motor ids: {unknown}"
            return response

        try:
            self.driver.set_zero_many(requested)
        except Exception as exc:
            response.success = False
            response.message = f"Set-zero failed atomically: {exc}"
            return response
        response.success = True
        response.message = f"Zero offsets saved atomically for motors {requested}"
        return response

    def emergency_stop_callback(self, request, response):
        del request
        self.estop_active = True
        try:
            self.driver.stop_all()
            response.success = True
            response.message = "Emergency stop activated; all motors stopped"
        except Exception as exc:
            response.success = False
            response.message = f"Emergency stop transmission failed: {exc}"
        self.get_logger().warn(response.message)
        return response

    def clear_estop_callback(self, request, response):
        del request
        self.estop_active = False
        response.success = True
        response.message = (
            "Emergency stop cleared; motion still requires fresh SAFE state "
            "and an explicitly enabled position controller"
        )
        self.get_logger().info(response.message)
        return response

    def publish_estop_state(self) -> None:
        msg = Bool()
        msg.data = bool(self.estop_active)
        self.estop_state_pub.publish(msg)

    def destroy_node(self):
        try:
            self.driver.stop_all()
        except Exception:
            pass
        try:
            self.driver.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
