#!/usr/bin/env python3
"""Run hardware-free validation of motor protocol and closed-loop math."""

from __future__ import annotations

from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "motor_can_driver"))

from motor_can_driver.motor_protocol import (  # noqa: E402
    build_feedback_request,
    build_position_cmd,
    build_stop_cmd,
    parse_feedback,
)
from motor_can_driver.position_control_core import (  # noqa: E402
    PositionTarget,
    compute_relative_steps,
    validate_target_set,
)


def main() -> None:
    calibration = ROOT / "src" / "robot_bringup" / "config"
    calibration /= "robot_calibration.yaml"
    cfg = yaml.safe_load(calibration.read_text(encoding="utf-8"))

    motor_ids = [int(value) for value in cfg["motor"]["motor_ids"]]
    limits = {
        int(mid): (float(value[0]), float(value[1]))
        for mid, value in cfg["motor"]["position_limit_deg"].items()
    }
    max_delta = {
        int(mid): float(value)
        for mid, value in cfg["motor"]["max_delta_deg_per_step"].items()
    }
    targets = {
        mid: PositionTarget(position_deg=12.0, max_speed_rad_s=0.3)
        for mid in motor_ids
    }
    measured = {mid: 0.0 for mid in motor_ids}
    validate_target_set(motor_ids, targets, limits, 1.0)

    batches = 0
    while batches < 100:
        steps, _, reached = compute_relative_steps(
            motor_ids,
            targets,
            measured,
            position_tolerance_deg=0.2,
            max_delta_deg_per_step=max_delta,
        )
        if all(reached.values()):
            break
        for motor_id, step in steps.items():
            measured[motor_id] += step.delta_deg
        batches += 1

    assert batches == 3
    assert all(abs(measured[mid] - 12.0) <= 0.2 for mid in motor_ids)

    assert build_position_cmd(12.3, 0.4, 32) == bytes([
        2, 0, 32, 0, 123, 0, 4,
    ])
    assert build_position_cmd(-12.3, 0.4, 32) == bytes([
        2, 1, 32, 0, 123, 0, 4,
    ])
    feedback = parse_feedback(bytes([
        2, 1, 0, 10, 255, 255, 255, 246,
    ]))
    assert feedback.motor_id == 2
    assert feedback.reached is True
    assert feedback.raw_deg == -1.0
    assert len(build_feedback_request()) == 7
    assert len(build_stop_cmd(32)) == 7

    print("PASS: protocol frames and absolute-to-relative closed-loop math")
    print(f"PASS: 12 deg target converged in {batches} bounded batches")


if __name__ == "__main__":
    main()
