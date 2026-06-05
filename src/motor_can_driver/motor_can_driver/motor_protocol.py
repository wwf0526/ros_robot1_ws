#电机协议转换，角度/速度 ↔ 7字节CAN数据
from dataclasses import dataclass


MODE_SPEED = 0x01       # 速度控制模式
MODE_POSITION = 0x02    # 位置控制模式
DIR_CCW = 0x00          # 逆时针方向 (Counter-Clockwise)
DIR_CW = 0x01           # 顺时针方向 (Clockwise)


@dataclass
class MotorStatus:
    motor_id: int
    reached: bool
    speed_rad_s: float
    raw_deg: float


def u16(value: float) -> tuple[int, int]:
    value_i = round(value)
    if not 0 <= value_i <= 0xFFFF:
        raise ValueError(f"value out of range: {value_i}")
    return (value_i >> 8) & 0xFF, value_i & 0xFF


def build_position_cmd(angle_deg: float, speed_rad_s: float, microstep: int) -> bytes:
    direction = DIR_CCW if angle_deg >= 0 else DIR_CW

    pos_h, pos_l = u16(abs(angle_deg) * 10.0)
    speed_h, speed_l = u16(abs(speed_rad_s) * 10.0)

    return bytes([
        MODE_POSITION,
        direction,
        microstep,
        pos_h,
        pos_l,
        speed_h,
        speed_l,
    ])


def build_stop_cmd(microstep: int) -> bytes:
    return bytes([
        MODE_SPEED,
        DIR_CW,
        microstep,
        0x00,
        0x00,
        0x00,
        0x00,
    ])


def build_feedback_request() -> bytes:
    return b"\x00" * 7


def parse_feedback(data: bytes) -> MotorStatus:
    if len(data) < 8:
        raise ValueError(f"feedback too short: {data.hex(' ')}")

    motor_id = data[0]
    reached = data[1] == 0x01

    speed_raw = (data[2] << 8) | data[3]
    speed_rad_s = speed_raw / 10.0

    pos_raw = (
        (data[4] << 24)
        | (data[5] << 16)
        | (data[6] << 8)
        | data[7]
    )

    if pos_raw & 0x80000000:
        pos_raw -= 0x100000000

    raw_deg = pos_raw / 10.0

    return MotorStatus(
        motor_id=motor_id,
        reached=reached,
        speed_rad_s=speed_rad_s,
        raw_deg=raw_deg,
    )
