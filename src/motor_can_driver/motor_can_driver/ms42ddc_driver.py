#CAN总线驱动，通过 SocketCAN 发送/接收CAN帧
from __future__ import annotations

import json
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
        self.bitrate = bitrate
        self.feedback_id = feedback_id
        self.microstep = microstep
        self.timeout = timeout
        self.motor_ids = motor_ids
        self.secondary_ids = secondary_ids

        self.motor_channels = {
            mid: self.primary_channel for mid in self.motor_ids
        }

        for mid in self.secondary_ids:
            self.motor_channels[mid] = self.secondary_channel

        self.buses = {}
        self.zero_file = Path(zero_file).expanduser()
        self.zero_offsets = self._load_zero_offsets()

    def open(self) -> None:
        self._get_bus(self.primary_channel)
        self._get_bus(self.secondary_channel)

    def close(self) -> None:
        for bus in self.buses.values():
            bus.shutdown()
        self.buses.clear()

    def _get_bus(self, channel: str):
        if channel not in self.buses:
            self.buses[channel] = can.interface.Bus(
                channel=channel,
                interface=self.interface,
                bitrate=self.bitrate,
            )
        return self.buses[channel]

    def _bus_for_motor(self, motor_id: int):
        channel = self.motor_channels.get(motor_id, self.primary_channel)
        return self._get_bus(channel)
        
    def channel_for_motor(self, motor_id: int) -> str:
        return self.motor_channels.get(motor_id, self.primary_channel)

    def _drain(self, bus) -> None:
        while bus.recv(timeout=0.0) is not None:
            pass

    def send_position(self, motor_id: int, angle_deg: float, speed_rad_s: float) -> None:
        bus = self._bus_for_motor(motor_id)
        data = build_position_cmd(angle_deg, speed_rad_s, self.microstep)

        msg = can.Message(
            arbitration_id=motor_id,
            data=data,
            is_extended_id=False,
        )

        bus.send(msg)

    def read_status(self, motor_id: int) -> MotorStatus:
        bus = self._bus_for_motor(motor_id)

        self._drain(bus)

        request = can.Message(
            arbitration_id=motor_id,
            data=build_feedback_request(),
            is_extended_id=False,
        )
        bus.send(request)

        end_time = time.monotonic() + self.timeout

        while time.monotonic() < end_time:
            msg = bus.recv(timeout=max(0.0, end_time - time.monotonic()))

            if msg is None:
                continue

            if msg.arbitration_id != self.feedback_id:
                continue

            status = parse_feedback(bytes(msg.data))

            if status.motor_id == motor_id:
                return status

        raise TimeoutError(f"no feedback from motor {motor_id}")

    def read_all_status(self) -> list[MotorStatus]:
        states = []

        for motor_id in self.motor_ids:
            try:
                states.append(self.read_status(motor_id))
            except Exception:
                continue

        return states

    def set_zero(self, motor_id: int) -> None:
        status = self.read_status(motor_id)
        self.zero_offsets[motor_id] = status.raw_deg
        self._save_zero_offsets()

    def home(self, motor_id: int, speed_rad_s: float, tolerance_deg: float) -> None:
        status = self.read_status(motor_id)
        rel = self.relative_position(status)

        if abs(rel) <= tolerance_deg:
            return

        self.send_position(motor_id, rel, speed_rad_s)

    def stop(self, motor_id: int) -> None:
        bus = self._bus_for_motor(motor_id)
        data = build_stop_cmd(self.microstep)

        msg = can.Message(
            arbitration_id=motor_id,
            data=data,
            is_extended_id=False,
        )

        for _ in range(3):
            bus.send(msg)
            time.sleep(0.02)

    def stop_all(self) -> None:
        for motor_id in self.motor_ids:
            self.stop(motor_id)

    def _load_zero_offsets(self) -> dict[int, float]:
        if not self.zero_file.exists():
            return {}

        data = json.loads(self.zero_file.read_text(encoding="utf-8"))
        return {int(k): float(v) for k, v in data.items()}

    def _save_zero_offsets(self) -> None:
        self.zero_file.parent.mkdir(parents=True, exist_ok=True)
        data = {str(k): v for k, v in sorted(self.zero_offsets.items())}
        self.zero_file.write_text(json.dumps(data, indent=2), encoding="utf-8")
        
    def relative_position(self, status):
        raw = float(status.raw_deg)
        zero = self.zero_offsets.get(status.motor_id, 0.0)
        return raw - zero
