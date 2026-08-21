"""Feedback-based absolute motor position execution for MPC and homing."""

from __future__ import annotations

import math
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import yaml

from robot_interfaces.msg import (
    MotorCommand,
    MotorCommandArray,
    MotorPositionControlState,
    MotorPositionTargetArray,
    MotorState,
    SafetyState,
)
from robot_interfaces.srv import (
    ClearMotorControlFault,
    HomeMotors,
    SetMotorControlEnabled,
)

from .position_control_core import (
    PositionTarget,
    compute_relative_steps,
    validate_target_set,
)


MODE_STOP = 0
MODE_RELATIVE_POSITION = 2


class MotorPositionController(Node):
    """Convert absolute zero-referenced targets to bounded relative moves."""

    def __init__(self):
        super().__init__("motor_position_controller")

        self.declare_parameter("calibration_file", "")
        self.declare_parameter("target_topic", "/motor/position_target")
        self.declare_parameter("raw_command_topic", "/motor/raw_command_array")
        self.declare_parameter("motor_state_topic", "/motor/state")
        self.declare_parameter("safety_topic", "/continuum/safety_state")
        self.declare_parameter(
            "controller_state_topic",
            "/motor/position_control_state",
        )
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("target_timeout_sec", 0.5)
        self.declare_parameter("feedback_timeout_sec", 0.25)
        self.declare_parameter("safety_timeout_sec", 0.25)
        self.declare_parameter("batch_motion_timeout_sec", 2.0)
        self.declare_parameter("position_tolerance_deg", 0.2)
        self.declare_parameter("proportional_gain", 1.0)
        self.declare_parameter("default_speed_rad_s", 0.3)
        self.declare_parameter("speed_limit_rad_s", 1.0)
        self.declare_parameter("require_safety_state", True)
        self.declare_parameter("start_enabled", False)

        self.calibration_file = self._resolve_calibration_file(
            str(self.get_parameter("calibration_file").value)
        )
        self._load_calibration()

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.raw_command_topic = str(
            self.get_parameter("raw_command_topic").value
        )
        self.motor_state_topic = str(
            self.get_parameter("motor_state_topic").value
        )
        self.safety_topic = str(self.get_parameter("safety_topic").value)
        self.controller_state_topic = str(
            self.get_parameter("controller_state_topic").value
        )
        self.control_rate_hz = float(
            self.get_parameter("control_rate_hz").value
        )
        self.target_timeout_sec = float(
            self.get_parameter("target_timeout_sec").value
        )
        self.feedback_timeout_sec = float(
            self.get_parameter("feedback_timeout_sec").value
        )
        self.safety_timeout_sec = float(
            self.get_parameter("safety_timeout_sec").value
        )
        self.batch_motion_timeout_sec = float(
            self.get_parameter("batch_motion_timeout_sec").value
        )
        self.position_tolerance_deg = float(
            self.get_parameter("position_tolerance_deg").value
        )
        self.proportional_gain = float(
            self.get_parameter("proportional_gain").value
        )
        self.default_speed_rad_s = float(
            self.get_parameter("default_speed_rad_s").value
        )
        self.speed_limit_rad_s = float(
            self.get_parameter("speed_limit_rad_s").value
        )
        self.require_safety_state = bool(
            self.get_parameter("require_safety_state").value
        )
        self.controller_enabled = bool(
            self.get_parameter("start_enabled").value
        )

        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")

        self.measured_position_deg: dict[int, float] = {}
        self.motor_reached: dict[int, bool] = {}
        self.last_feedback_time: dict[int, float] = {}

        self.targets: dict[int, PositionTarget] = {}
        self.target_valid = False
        self.target_requires_heartbeat = True
        self.last_target_time: float | None = None
        self.target_source = ""

        self.safety_safe = False
        self.safety_status = "not received"
        self.last_safety_time: float | None = None
        self.estop_active = False

        self.batch_in_flight = False
        self.batch_motor_ids: set[int] = set()
        self.batch_sent_time: float | None = None
        self.last_command_delta_deg = {
            mid: 0.0 for mid in self.motor_ids
        }

        self.fault_active = False
        self.fault_reason = ""
        self._stop_sent = False
        self._status = "DISABLED"

        self.raw_command_pub = self.create_publisher(
            MotorCommandArray,
            self.raw_command_topic,
            10,
        )
        self.controller_state_pub = self.create_publisher(
            MotorPositionControlState,
            self.controller_state_topic,
            10,
        )

        self.create_subscription(
            MotorPositionTargetArray,
            self.target_topic,
            self.target_callback,
            10,
        )
        self.create_subscription(
            MotorState,
            self.motor_state_topic,
            self.motor_state_callback,
            50,
        )
        self.create_subscription(
            SafetyState,
            self.safety_topic,
            self.safety_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/motor/emergency_stop_active",
            self.estop_callback,
            10,
        )

        self.enable_srv = self.create_service(
            SetMotorControlEnabled,
            "/motor_controller/set_enabled",
            self.set_enabled_callback,
        )
        self.clear_fault_srv = self.create_service(
            ClearMotorControlFault,
            "/motor_controller/clear_fault",
            self.clear_fault_callback,
        )
        self.home_srv = self.create_service(
            HomeMotors,
            "/motor/home_motors",
            self.home_motors_callback,
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )
        self.get_logger().info(
            "Final motor position controller started: absolute targets -> "
            "bounded mode-2 relative batches; starts disarmed"
        )

    @staticmethod
    def _resolve_calibration_file(value: str) -> Path:
        if value:
            return Path(value).expanduser().resolve()
        share = Path(get_package_share_directory("robot_bringup"))
        return share / "config" / "robot_calibration.yaml"

    def _load_calibration(self) -> None:
        with self.calibration_file.open("r", encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream)
        self.motor_ids = [int(v) for v in cfg["motor"]["motor_ids"]]
        self.position_limits_deg = {
            int(mid): (float(value[0]), float(value[1]))
            for mid, value in cfg["motor"]["position_limit_deg"].items()
        }
        self.max_delta_deg_per_step = {
            int(mid): float(value)
            for mid, value in cfg["motor"]["max_delta_deg_per_step"].items()
        }

    def motor_state_callback(self, msg: MotorState) -> None:
        motor_id = int(msg.motor_id)
        if motor_id not in self.motor_ids:
            return
        position = float(msg.position_deg)
        if not math.isfinite(position):
            self._enter_fault(f"motor {motor_id}: non-finite feedback")
            return
        self.measured_position_deg[motor_id] = position
        self.motor_reached[motor_id] = bool(msg.reached)
        self.last_feedback_time[motor_id] = time.monotonic()

    def safety_callback(self, msg: SafetyState) -> None:
        self.safety_safe = bool(msg.safe_to_control)
        self.safety_status = str(msg.status)
        self.last_safety_time = time.monotonic()

    def estop_callback(self, msg: Bool) -> None:
        self.estop_active = bool(msg.data)
        if self.estop_active:
            self._enter_fault("emergency stop active")

    def target_callback(self, msg: MotorPositionTargetArray) -> None:
        if not bool(msg.enable):
            self.target_valid = False
            self.targets.clear()
            self.target_source = str(msg.source)
            self._publish_stop_once("target disabled")
            return

        received: dict[int, PositionTarget] = {}
        duplicate_ids = set()
        for item in msg.targets:
            motor_id = int(item.motor_id)
            if motor_id in received:
                duplicate_ids.add(motor_id)
            received[motor_id] = PositionTarget(
                position_deg=float(item.position_deg),
                max_speed_rad_s=float(item.max_speed_rad_s),
            )

        if duplicate_ids:
            self._enter_fault(
                f"duplicate motor ids in target: {sorted(duplicate_ids)}"
            )
            return

        try:
            validate_target_set(
                self.motor_ids,
                received,
                self.position_limits_deg,
                self.speed_limit_rad_s,
            )
        except ValueError as exc:
            self._enter_fault(f"invalid target: {exc}")
            return

        self.targets = received
        self.target_valid = True
        self.target_requires_heartbeat = True
        self.last_target_time = time.monotonic()
        self.target_source = str(msg.source) or "unspecified"
        self._stop_sent = False

    def set_enabled_callback(self, request, response):
        if not bool(request.enable):
            self.controller_enabled = False
            self.target_valid = False
            self.targets.clear()
            self._publish_stop_once("controller disabled")
            response.success = True
            response.message = "Motor position controller disabled"
            return response

        now = time.monotonic()
        if self.fault_active:
            response.success = False
            response.message = f"Cannot enable: {self.fault_reason}"
            return response
        if self.estop_active:
            response.success = False
            response.message = "Cannot enable: emergency stop active"
            return response
        if not self._feedback_valid(now):
            response.success = False
            response.message = "Cannot enable: complete fresh motor feedback required"
            return response
        if not self._safety_valid(now):
            response.success = False
            response.message = (
                f"Cannot enable: safety unavailable/unsafe ({self.safety_status})"
            )
            return response

        self.controller_enabled = True
        self._stop_sent = False
        self.publish_controller_state(
            now,
            feedback_valid=True,
            safety_valid=True,
            target_timeout=False,
        )
        response.success = True
        response.message = "Motor position controller enabled; waiting for target"
        return response

    def clear_fault_callback(self, request, response):
        del request
        now = time.monotonic()
        if self.estop_active:
            response.success = False
            response.message = "Cannot clear fault while emergency stop is active"
            return response
        if not self._feedback_valid(now):
            response.success = False
            response.message = "Cannot clear fault without complete fresh feedback"
            return response
        if not self._safety_valid(now):
            response.success = False
            response.message = "Cannot clear fault while safety is unavailable/unsafe"
            return response

        self.fault_active = False
        self.fault_reason = ""
        self.batch_in_flight = False
        self.batch_motor_ids.clear()
        self.batch_sent_time = None
        response.success = True
        response.message = "Motor controller fault cleared; controller remains disabled"
        return response

    def home_motors_callback(self, request, response):
        now = time.monotonic()
        if not self.controller_enabled or self.fault_active:
            response.success = False
            response.message = "Home rejected: controller must be enabled and fault-free"
            return response
        if not self._feedback_valid(now) or not self._safety_valid(now):
            response.success = False
            response.message = "Home rejected: fresh feedback and SAFE state required"
            return response

        requested = [int(mid) for mid in request.motor_ids]
        if not requested:
            requested = list(self.motor_ids)
        unknown = sorted(set(requested) - set(self.motor_ids))
        if unknown:
            response.success = False
            response.message = f"Home rejected: unknown motor ids {unknown}"
            return response

        speed = float(request.speed_rad_s)
        if speed <= 0.0:
            speed = self.default_speed_rad_s
        if not math.isfinite(speed) or speed > self.speed_limit_rad_s:
            response.success = False
            response.message = "Home rejected: invalid speed"
            return response

        self.targets = {
            mid: PositionTarget(
                position_deg=(
                    0.0 if mid in requested else self.measured_position_deg[mid]
                ),
                max_speed_rad_s=speed,
            )
            for mid in self.motor_ids
        }
        self.target_valid = True
        self.target_requires_heartbeat = False
        self.last_target_time = now
        self.target_source = "home_service"
        self._stop_sent = False
        response.success = True
        response.message = f"Home target accepted for motors {requested}"
        return response

    def _feedback_valid(self, now: float) -> bool:
        return all(
            mid in self.measured_position_deg
            and mid in self.last_feedback_time
            and now - self.last_feedback_time[mid] <= self.feedback_timeout_sec
            for mid in self.motor_ids
        )

    def _safety_valid(self, now: float) -> bool:
        if not self.require_safety_state:
            return not self.estop_active
        return bool(
            self.last_safety_time is not None
            and now - self.last_safety_time <= self.safety_timeout_sec
            and self.safety_safe
            and not self.estop_active
        )

    def _target_timed_out(self, now: float) -> bool:
        return bool(
            self.target_valid
            and self.target_requires_heartbeat
            and self.last_target_time is not None
            and now - self.last_target_time > self.target_timeout_sec
        )

    def _batch_complete(self) -> bool:
        if not self.batch_in_flight or self.batch_sent_time is None:
            return True
        return all(
            self.last_feedback_time.get(mid, -1.0) > self.batch_sent_time
            and self.motor_reached.get(mid, False)
            for mid in self.batch_motor_ids
        )

    def _enter_fault(self, reason: str) -> None:
        if not self.fault_active:
            self.get_logger().error(f"Motor controller fault: {reason}")
        self.fault_active = True
        self.fault_reason = str(reason)
        self.controller_enabled = False
        self.target_valid = False
        self.batch_in_flight = False
        self.batch_motor_ids.clear()
        self._publish_stop_once(reason)

    def _publish_stop_once(self, reason: str) -> None:
        if self._stop_sent:
            return
        msg = MotorCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for motor_id in self.motor_ids:
            command = MotorCommand()
            command.motor_id = motor_id
            command.mode = MODE_STOP
            command.target_deg = 0.0
            command.speed_rad_s = 0.0
            msg.commands.append(command)
            self.last_command_delta_deg[motor_id] = 0.0
        self.raw_command_pub.publish(msg)
        self._stop_sent = True
        self.get_logger().warn(f"Stop command published: {reason}")

    def _publish_relative_batch(self, steps) -> None:
        msg = MotorCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for motor_id in self.motor_ids:
            step = steps.get(motor_id)
            self.last_command_delta_deg[motor_id] = (
                float(step.delta_deg) if step is not None else 0.0
            )
            if step is None:
                continue
            command = MotorCommand()
            command.motor_id = motor_id
            command.mode = MODE_RELATIVE_POSITION
            command.target_deg = float(step.delta_deg)
            command.speed_rad_s = float(step.speed_rad_s)
            msg.commands.append(command)

        if not msg.commands:
            return
        self.raw_command_pub.publish(msg)
        self.batch_in_flight = True
        self.batch_motor_ids = {int(cmd.motor_id) for cmd in msg.commands}
        self.batch_sent_time = time.monotonic()
        self._stop_sent = False

    def control_loop(self) -> None:
        now = time.monotonic()
        feedback_valid = self._feedback_valid(now)
        safety_valid = self._safety_valid(now)
        target_timeout = self._target_timed_out(now)

        if self.controller_enabled and not feedback_valid:
            self._enter_fault("motor feedback timeout")
        elif self.controller_enabled and not safety_valid:
            self._enter_fault(f"safety lost: {self.safety_status}")
        elif self.controller_enabled and target_timeout:
            self._enter_fault("absolute position target heartbeat timeout")

        if self.batch_in_flight:
            if self._batch_complete():
                self.batch_in_flight = False
                self.batch_motor_ids.clear()
                self.batch_sent_time = None
            elif (
                self.batch_sent_time is not None
                and now - self.batch_sent_time > self.batch_motion_timeout_sec
            ):
                self._enter_fault("relative motion batch timeout")

        if self.fault_active:
            self._status = f"FAULT: {self.fault_reason}"
        elif not self.controller_enabled:
            self._status = "DISABLED"
        elif not self.target_valid:
            self._status = "ENABLED: waiting for complete absolute target"
        elif self.batch_in_flight:
            self._status = "RUNNING: waiting for mode-2 batch completion"
        else:
            try:
                steps, _, reached = compute_relative_steps(
                    self.motor_ids,
                    self.targets,
                    self.measured_position_deg,
                    self.position_tolerance_deg,
                    self.max_delta_deg_per_step,
                    self.proportional_gain,
                )
            except (KeyError, ValueError) as exc:
                self._enter_fault(f"closed-loop computation failed: {exc}")
            else:
                if steps:
                    self._publish_relative_batch(steps)
                    self._status = "RUNNING: relative batch published"
                elif all(reached.values()):
                    self._status = "TARGET_REACHED"

        self.publish_controller_state(
            now,
            feedback_valid,
            safety_valid,
            target_timeout,
        )

    def publish_controller_state(
        self,
        now: float,
        feedback_valid: bool,
        safety_valid: bool,
        target_timeout: bool,
    ) -> None:
        msg = MotorPositionControlState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "motor_zero_frame"
        msg.controller_enabled = bool(self.controller_enabled)
        msg.target_valid = bool(self.target_valid)
        msg.target_timeout = bool(target_timeout)
        msg.safety_valid = bool(safety_valid)
        msg.safety_timeout = bool(
            self.require_safety_state
            and (
                self.last_safety_time is None
                or now - self.last_safety_time > self.safety_timeout_sec
            )
        )
        msg.feedback_valid = bool(feedback_valid)
        msg.batch_in_flight = bool(self.batch_in_flight)
        msg.fault_active = bool(self.fault_active)
        msg.motor_ids = list(self.motor_ids)

        all_reached = bool(self.target_valid and feedback_valid)
        for motor_id in self.motor_ids:
            target = self.targets.get(motor_id)
            measured = self.measured_position_deg.get(motor_id, math.nan)
            target_position = (
                float(target.position_deg) if target is not None else math.nan
            )
            error = target_position - measured
            reached = bool(
                math.isfinite(error)
                and abs(error) <= self.position_tolerance_deg
            )
            all_reached = all_reached and reached
            msg.target_position_deg.append(float(target_position))
            msg.measured_position_deg.append(float(measured))
            msg.position_error_deg.append(float(error))
            msg.last_command_delta_deg.append(
                float(self.last_command_delta_deg[motor_id])
            )
            msg.motor_reached.append(reached)

        msg.all_reached = bool(all_reached)
        msg.source = self.target_source
        msg.fault_reason = self.fault_reason
        msg.status = self._status
        self.controller_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorPositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_stop_once("node shutdown")
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
