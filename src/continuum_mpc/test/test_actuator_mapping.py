import math

import numpy as np
import pytest

from continuum_mpc.actuator_mapping import (
    ActuatorMappingError,
    ContinuumActuatorMapping,
)


def calibration():
    return {
        "motor": {
            "motor_ids": [1, 2, 3, 4, 5, 6],
            "sign": {mid: 1.0 for mid in range(1, 7)},
            "spool_radius_mm": {mid: 8.0 for mid in range(1, 7)},
            "position_limit_deg": {
                mid: [-180.0, 180.0] for mid in range(1, 7)
            },
        },
        "tendon": {
            "motor_to_tendon": {mid: mid for mid in range(1, 7)},
        },
        "pcc_model": {
            "chain_order": ["section2", "section1"],
            "sections": {
                "section1": {
                    "tendon_ids": [5, 3, 1],
                    "dl_signs": [1.0, 1.0, 1.0],
                },
                "section2": {
                    "tendon_ids": [4, 2, 6],
                    "dl_signs": [1.0, 1.0, 1.0],
                },
            },
        },
    }


def test_motor_tendon_round_trip_and_known_section1_motion():
    mapping = ContinuumActuatorMapping.from_config(calibration())
    motor_deg = np.array([45.0, 0.0, -22.5, 0.0, -22.5, 0.0])

    tendon_mm = mapping.motor_deg_to_tendon_mm(motor_deg)

    assert tendon_mm[0] == pytest.approx(2.0 * math.pi)
    assert tendon_mm[2] == pytest.approx(-math.pi)
    assert tendon_mm[4] == pytest.approx(-math.pi)
    assert tendon_mm[0] + tendon_mm[2] + tendon_mm[4] == pytest.approx(0.0)
    assert mapping.tendon_mm_to_motor_deg(tendon_mm) == pytest.approx(motor_deg)

    pcc = mapping.pcc_sections_from_tendon_mm(tendon_mm)
    assert pcc["section1"] == pytest.approx(
        [-math.pi, -math.pi, 2.0 * math.pi]
    )
    assert pcc["section2"] == pytest.approx([0.0, 0.0, 0.0])


def test_four_independent_rates_enforce_both_section_sums():
    mapping = ContinuumActuatorMapping.from_config(calibration())

    # Order follows chain_order: section2(u1,u2), section1(u1,u2).
    tendon_rate = mapping.independent_to_tendon_velocity_mm_s(
        [1.0, 2.0, 3.0, 4.0]
    )

    assert tendon_rate == pytest.approx([-7.0, 2.0, 4.0, 1.0, 3.0, -3.0])
    sums = mapping.modeled_section_sums(tendon_rate)
    assert sums["section2"] == pytest.approx(0.0)
    assert sums["section1"] == pytest.approx(0.0)


def test_projection_removes_only_common_mode_component():
    mapping = ContinuumActuatorMapping.from_config(calibration())
    projected = mapping.project_tendon_velocity_zero_sum(
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    )

    assert projected == pytest.approx([-2.0, -2.0, 0.0, 0.0, 2.0, 2.0])
    assert all(
        abs(value) < 1.0e-12
        for value in mapping.modeled_section_sums(projected).values()
    )


def test_invalid_non_bijective_tendon_mapping_is_rejected():
    cfg = calibration()
    cfg["tendon"]["motor_to_tendon"][6] = 5
    with pytest.raises(ActuatorMappingError, match="bijection"):
        ContinuumActuatorMapping.from_config(cfg)


def test_motor_limit_is_enforced_during_inverse_mapping():
    mapping = ContinuumActuatorMapping.from_config(calibration())
    tendon_mm = np.zeros(6)
    tendon_mm[0] = 8.0 * math.radians(181.0)
    with pytest.raises(ActuatorMappingError, match="outside"):
        mapping.tendon_mm_to_motor_deg(tendon_mm)


def test_independent_actuation_matrix_matches_direct_mapping():
    mapping = ContinuumActuatorMapping.from_config(calibration())
    independent = np.array([1.2, -0.4, 0.7, -0.3])

    direct = mapping.independent_to_tendon_velocity_mm_s(independent)
    matrix_result = mapping.independent_to_tendon_matrix() @ independent

    assert matrix_result == pytest.approx(direct)
    assert mapping.independent_to_tendon_matrix().shape == (6, 4)


def test_tendon_bounds_include_motor_direction_and_margin():
    cfg = calibration()
    cfg["motor"]["sign"][2] = -1.0
    mapping = ContinuumActuatorMapping.from_config(cfg)

    lower, upper = mapping.tendon_position_bounds_mm(
        motor_limit_margin_deg=10.0
    )
    expected = 8.0 * math.radians(170.0)

    assert lower == pytest.approx(-expected * np.ones(6))
    assert upper == pytest.approx(expected * np.ones(6))
