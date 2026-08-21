"""Threaded SocketCAN driver for the six MS42DDC motor execution layer."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import can

from .motor_protocol import (
    MotorStatus,
    build_feedback_request,
    build_position_cmd,
    build_stop_cmd,
    parse_feedback,
)


class MS42DDCDriver:
    """Send commands and cache asynchronous feedback from both CAN buses."""

    def __init__(
        self,
        interface: str,
        primary_channel: str,
        secondary_channel: str,
        bitrate: int,
        feedback_id: int,
        microstep: int,
        timeout: float,
        motor_ids: list[int],
        secondary_ids: list[int],
        zero_file: str,
    ) -> None:
        self.interface = interface
        self.primary_channel = primary_channel
        self.secondary_channel = secondary_channel
        self.bitrate = int(bitrate)
        self.feedback_id = int(feedback_id)
        self.microstep = int(microstep)
        self.timeout = float(timeout)
        self.motor_ids = [int(mid) for mid in motor_ids]
        self.secondary_ids = {int(mid) for mid in secondary_ids}

        self.motor_channels = {
            mid: (
                self.secondary_channel
                if mid in self.secondary_ids
                else self.primary_channel
            )
            for mid in self.motor_ids
        }

        self.buses: dict[str, can.BusABC] = {}
        self._bus_send_locks: dict[str, threading.Lock] = {}
        self._rx_threads: list[threading.Thread] = []
        self._stop_event = threading.Event()
        self._status_condition = threading.Condition()
        self._status_cache: dict[int, MotorStatus] = {}
        self._status_time: dict[int, float] = {}
        self._status_sequence: dict[int, int] = {
            mid: 0 for mid in self.motor_ids
        }

        self.zero_file = Path(zero_file).expanduser()
        self.zero_offsets = self._load_zero_offsets()

    def open(self) -> None:
        """Open both buses and start one receiver thread per physical channel."""

        self._stop_event.clear()
        channels = list(dict.fromkeys([
            self.primary_channel,
            self.secondary_channel,
        ]))
        for channel in channels:
            bus = self._get_bus(channel)
            thread = threading.Thread(
                target=self._receive_loop,
                args=(channel, bus),
                name=f"ms42ddc-rx-{channel}",
                daemon=True,
            )
            thread.start()
            self._rx_threads.append(thread)

    def close(self) -> None:
        """Stop receiver threads before shutting down the CAN objects."""

        self._stop_event.set()
        for thread in self._rx_threads:
            thread.join(timeout=0.5)
        self._rx_threads.clear()
        for bus in self.buses.values():
            bus.shutdown()
        self.buses.clear()
        self._bus_send_locks.clear()

    def _get_bus(self, channel: str):
        if channel not in self.buses:
            # One background receiver and ROS executor send callbacks share
            # each physical channel, so use python-can's synchronized wrapper.
            self.buses[channel] = can.ThreadSafeBus(
                channel=channel,
                interface=self.interface,
                bitrate=self.bitrate,
            )
            self._bus_send_locks[channel] = threading.Lock()
        return self.buses[channel]

    def _validate_motor_id(self, motor_id: int) -> int:
        value = int(motor_id)
        if value not in self.motor_channels:
            raise ValueError(f"unknown motor id: {value}")
        return value

    def _bus_for_motor(self, motor_id: int):
        value = self._validate_motor_id(motor_id)
        return self._get_bus(self.motor_channels[value])

    def channel_for_motor(self, motor_id: int) -> str:
        value = self._validate_motor_id(motor_id)
        return self.motor_channels[value]

    def _send(self, motor_id: int, data: bytes) -> None:
        value = self._validate_motor_id(motor_id)
        channel = self.motor_channels[value]
        bus = self._get_bus(channel)
        message = can.Message(
            arbitration_id=value,
            data=data,
            is_extended_id=False,
        )
        with self._bus_send_locks[channel]:
            bus.send(message)

    def _receive_loop(self, channel: str, bus) -> None:
        while not self._stop_event.is_set():
            try:
                message = bus.recv(timeout=0.1)
            except Exception:
                if self._stop_event.is_set():
                    return
                continue
            if message is None or message.arbitration_id != self.feedback_id:
                continue
            try:
                status = parse_feedback(bytes(message.data))
            except (TypeError, ValueError):
                continue
            if status.motor_id not in self.motor_channels:
                continue
            if self.motor_channels[status.motor_id] != channel:
                continue

            with self._status_condition:
                self._status_cache[status.motor_id] = status
                self._status_time[status.motor_id] = time.monotonic()
                self._status_sequence[status.motor_id] += 1
                self._status_condition.notify_all()

    def send_position(
        self,
        motor_id: int,
        angle_deg: float,
        speed_rad_s: float,
    ) -> None:
        """Send one manufacturer mode-2 relative displacement command."""

        self._send(
            motor_id,
            build_position_cmd(angle_deg, speed_rad_s, self.microstep),
        )

    def request_status(self, motor_id: int) -> None:
        """Request feedback without blocking the ROS executor."""

        self._send(motor_id, build_feedback_request())

    def latest_status(
        self,
        motor_id: int,
    ) -> tuple[MotorStatus, float, int] | None:
        """Return the cached status, monotonic receive time and sequence."""

        value = self._validate_motor_id(motor_id)
        with self._status_condition:
            status = self._status_cache.get(value)
            if status is None:
                return None
            return (
                status,
                self._status_time[value],
                self._status_sequence[value],
            )

    def read_status(self, motor_id: int) -> MotorStatus:
        """Request and wait for one new response; used only by services."""

        value = self._validate_motor_id(motor_id)
        with self._status_condition:
            start_sequence = self._status_sequence[value]
        self.request_status(value)
        deadline = time.monotonic() + self.timeout
        with self._status_condition:
            while self._status_sequence[value] <= start_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(f"no feedback from motor {value}")
                self._status_condition.wait(timeout=remaining)
            return self._status_cache[value]

    def read_all_status(self) -> list[MotorStatus]:
        states = []
        for motor_id in self.motor_ids:
            try:
                states.append(self.read_status(motor_id))
            except (can.CanError, TimeoutError, ValueError):
                continue
        return states

    def set_zero(self, motor_id: int) -> None:
        status = self.read_status(motor_id)
        if not status.reached:
            raise RuntimeError(f"motor {motor_id} is moving; zero not allowed")
        self.zero_offsets[int(motor_id)] = status.raw_deg
        self._save_zero_offsets()

    def set_zero_many(self, motor_ids: list[int]) -> None:
        """Capture several zero offsets atomically after all motors are still."""

        statuses = [self.read_status(motor_id) for motor_id in motor_ids]
        moving = [status.motor_id for status in statuses if not status.reached]
        if moving:
            raise RuntimeError(f"motors still moving; zero not allowed: {moving}")
        for status in statuses:
            self.zero_offsets[status.motor_id] = float(status.raw_deg)
        self._save_zero_offsets()

    def home(
        self,
        motor_id: int,
        speed_rad_s: float,
        tolerance_deg: float,
    ) -> None:
        """Compatibility helper; final homing is owned by the closed-loop node."""

        status = self.read_status(motor_id)
        relative = self.relative_position(status)
        if abs(relative) <= tolerance_deg:
            return
        self.send_position(motor_id, -relative, speed_rad_s)

    def stop(self, motor_id: int) -> None:
        data = build_stop_cmd(self.microstep)
        for _ in range(3):
            self._send(motor_id, data)
            time.sleep(0.02)

    def stop_all(self) -> None:
        for motor_id in self.motor_ids:
            self.stop(motor_id)

    def _load_zero_offsets(self) -> dict[int, float]:
        if not self.zero_file.exists():
            return {}
        data = json.loads(self.zero_file.read_text(encoding="utf-8"))
        return {int(key): float(value) for key, value in data.items()}

    def _save_zero_offsets(self) -> None:
        self.zero_file.parent.mkdir(parents=True, exist_ok=True)
        data = {
            str(key): value
            for key, value in sorted(self.zero_offsets.items())
        }
        self.zero_file.write_text(
            json.dumps(data, indent=2),
            encoding="utf-8",
        )

    def relative_position(self, status: MotorStatus) -> float:
        raw = float(status.raw_deg)
        zero = self.zero_offsets.get(status.motor_id, 0.0)
        return raw - zero
