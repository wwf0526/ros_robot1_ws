"""Pure functions for the absolute-position motor execution layer."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence


@dataclass(frozen=True)
class PositionTarget:
    """One zero-referenced absolute motor target."""

    position_deg: float
    max_speed_rad_s: float


@dataclass(frozen=True)
class RelativeStep:
    """One bounded mode-2 relative displacement command."""

    delta_deg: float
    speed_rad_s: float
    error_deg: float


def validate_target_set(
    motor_ids: Sequence[int],
    targets: Mapping[int, PositionTarget],
    position_limits_deg: Mapping[int, tuple[float, float]],
    speed_limit_rad_s: float,
) -> None:
    """Validate a complete six-motor absolute target atomically."""

    expected = {int(mid) for mid in motor_ids}
    received = {int(mid) for mid in targets}
    if received != expected:
        missing = sorted(expected - received)
        extra = sorted(received - expected)
        raise ValueError(
            f"target ids must exactly match configured motors; "
            f"missing={missing}, extra={extra}"
        )

    if not math.isfinite(speed_limit_rad_s) or speed_limit_rad_s <= 0.0:
        raise ValueError("speed_limit_rad_s must be finite and positive")

    for motor_id in motor_ids:
        target = targets[int(motor_id)]
        position = float(target.position_deg)
        speed = float(target.max_speed_rad_s)
        if not math.isfinite(position):
            raise ValueError(f"motor {motor_id}: non-finite target position")
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError(f"motor {motor_id}: speed must be finite and positive")
        if speed > speed_limit_rad_s:
            raise ValueError(
                f"motor {motor_id}: speed {speed:.6f} exceeds "
                f"limit {speed_limit_rad_s:.6f} rad/s"
            )

        low, high = position_limits_deg[int(motor_id)]
        if position < float(low) or position > float(high):
            raise ValueError(
                f"motor {motor_id}: target {position:.6f} deg outside "
                f"[{float(low):.6f}, {float(high):.6f}] deg"
            )


def compute_relative_steps(
    motor_ids: Sequence[int],
    targets: Mapping[int, PositionTarget],
    measured_position_deg: Mapping[int, float],
    position_tolerance_deg: float,
    max_delta_deg_per_step: Mapping[int, float],
    proportional_gain: float = 1.0,
) -> tuple[dict[int, RelativeStep], dict[int, float], dict[int, bool]]:
    """Convert absolute targets into one safe mode-2 relative command batch."""

    if position_tolerance_deg < 0.0:
        raise ValueError("position_tolerance_deg cannot be negative")
    if not math.isfinite(proportional_gain) or proportional_gain <= 0.0:
        raise ValueError("proportional_gain must be finite and positive")

    steps: dict[int, RelativeStep] = {}
    errors: dict[int, float] = {}
    reached: dict[int, bool] = {}

    for motor_id_value in motor_ids:
        motor_id = int(motor_id_value)
        measured = float(measured_position_deg[motor_id])
        if not math.isfinite(measured):
            raise ValueError(f"motor {motor_id}: non-finite measured position")

        error = float(targets[motor_id].position_deg) - measured
        errors[motor_id] = error
        motor_reached = abs(error) <= float(position_tolerance_deg)
        reached[motor_id] = motor_reached
        if motor_reached:
            continue

        max_delta = float(max_delta_deg_per_step[motor_id])
        if not math.isfinite(max_delta) or max_delta <= 0.0:
            raise ValueError(
                f"motor {motor_id}: max_delta_deg_per_step must be positive"
            )

        raw_delta = proportional_gain * error
        delta = max(-max_delta, min(raw_delta, max_delta))
        steps[motor_id] = RelativeStep(
            delta_deg=float(delta),
            speed_rad_s=float(targets[motor_id].max_speed_rad_s),
            error_deg=float(error),
        )

    return steps, errors, reached
