import math

import pytest

from motor_can_driver.position_control_core import (
    PositionTarget,
    compute_relative_steps,
    validate_target_set,
)


MOTOR_IDS = [1, 2, 3, 4, 5, 6]
LIMITS = {mid: (-180.0, 180.0) for mid in MOTOR_IDS}
MAX_DELTA = {mid: 5.0 for mid in MOTOR_IDS}


def targets(position=0.0, speed=0.3):
    return {
        mid: PositionTarget(position_deg=position, max_speed_rad_s=speed)
        for mid in MOTOR_IDS
    }


def test_complete_target_validation():
    validate_target_set(MOTOR_IDS, targets(), LIMITS, 1.0)


def test_missing_motor_is_rejected_atomically():
    value = targets()
    value.pop(6)
    with pytest.raises(ValueError, match=r"missing=\[6\]"):
        validate_target_set(MOTOR_IDS, value, LIMITS, 1.0)


def test_limit_and_nonfinite_values_are_rejected():
    value = targets()
    value[3] = PositionTarget(position_deg=181.0, max_speed_rad_s=0.3)
    with pytest.raises(ValueError, match="outside"):
        validate_target_set(MOTOR_IDS, value, LIMITS, 1.0)

    value = targets()
    value[3] = PositionTarget(position_deg=math.nan, max_speed_rad_s=0.3)
    with pytest.raises(ValueError, match="non-finite"):
        validate_target_set(MOTOR_IDS, value, LIMITS, 1.0)


def test_absolute_error_becomes_bounded_relative_mode2_step():
    value = targets(position=12.0)
    measured = {mid: 0.0 for mid in MOTOR_IDS}
    steps, errors, reached = compute_relative_steps(
        MOTOR_IDS,
        value,
        measured,
        position_tolerance_deg=0.2,
        max_delta_deg_per_step=MAX_DELTA,
    )
    assert all(step.delta_deg == 5.0 for step in steps.values())
    assert all(error == 12.0 for error in errors.values())
    assert not any(reached.values())


def test_tolerance_suppresses_repeated_commands():
    value = targets(position=10.0)
    measured = {mid: 9.9 for mid in MOTOR_IDS}
    steps, _, reached = compute_relative_steps(
        MOTOR_IDS,
        value,
        measured,
        position_tolerance_deg=0.2,
        max_delta_deg_per_step=MAX_DELTA,
    )
    assert steps == {}
    assert all(reached.values())
