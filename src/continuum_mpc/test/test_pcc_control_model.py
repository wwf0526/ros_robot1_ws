from pathlib import Path

import numpy as np
import pytest
import yaml

from continuum_mpc.pcc_control_model import (
    PccControlModel,
    body_orientation_delta,
)


@pytest.fixture(scope="module")
def model():
    src_root = Path(__file__).resolve().parents[2]
    calibration_file = (
        src_root / "robot_bringup" / "config" / "robot_calibration.yaml"
    )
    cfg = yaml.safe_load(calibration_file.read_text(encoding="utf-8"))
    return PccControlModel.from_config(cfg, samples_per_section=5)


def test_zero_pose_uses_real_section_lengths_and_chain_order(model):
    output = model.evaluate_from_motor_deg(np.zeros(6))

    assert output.mid_position_m == pytest.approx([0.0, 0.0, 0.14])
    assert output.tip_position_m == pytest.approx([0.0, 0.0, 0.34])
    assert output.mid_rotation == pytest.approx(np.eye(3))
    assert output.tip_rotation == pytest.approx(np.eye(3))
    assert output.body_points_m.shape == (11, 3)
    assert output.body_points_m[0] == pytest.approx([0.0, 0.0, 0.0])
    assert output.body_points_m[-1] == pytest.approx(output.tip_position_m)


def test_distal_section_motion_does_not_change_mid_pose(model):
    motor_deg = np.array([45.0, 0.0, -22.5, 0.0, -22.5, 0.0])
    output = model.evaluate_from_motor_deg(motor_deg)

    assert output.mid_position_m == pytest.approx([0.0, 0.0, 0.14])
    assert output.mid_rotation == pytest.approx(np.eye(3))
    assert not np.allclose(output.tip_position_m, [0.0, 0.0, 0.34])
    assert not np.allclose(output.tip_rotation, np.eye(3))


def test_9_by_4_jacobian_has_correct_section_causality(model):
    tendon = model.mapping.motor_deg_to_tendon_mm(
        [5.0, -3.0, -2.0, 1.0, -3.0, 2.0]
    )
    jacobians = model.finite_difference_jacobians(tendon)
    stacked = jacobians.stacked_local_output

    assert stacked.shape == (9, 4)
    assert np.all(np.isfinite(stacked))
    # section1 is distal, so its two columns cannot change the section2 end.
    assert jacobians.mid_orientation_rad_per_mm[:, 2:] == pytest.approx(
        np.zeros((3, 2)), abs=1.0e-10
    )
    assert np.linalg.matrix_rank(stacked, tol=1.0e-8) == 4


def test_jacobian_predicts_a_small_four_dof_increment(model):
    tendon = model.mapping.motor_deg_to_tendon_mm(
        [5.0, -3.0, -2.0, 1.0, -3.0, 2.0]
    )
    nominal = model.evaluate_from_tendon_mm(tendon)
    jacobians = model.finite_difference_jacobians(tendon)
    independent_delta = np.array([2.0e-3, -1.0e-3, 1.5e-3, -0.5e-3])
    tendon_delta = model.mapping.independent_to_tendon_velocity_mm_s(
        independent_delta
    )
    perturbed = model.evaluate_from_tendon_mm(tendon + tendon_delta)

    predicted_position_delta = (
        jacobians.tip_position_m_per_mm @ independent_delta
    )
    measured_position_delta = perturbed.tip_position_m - nominal.tip_position_m
    predicted_mid_delta = (
        jacobians.mid_orientation_rad_per_mm @ independent_delta
    )
    measured_mid_delta = body_orientation_delta(
        nominal.mid_rotation,
        perturbed.mid_rotation,
    )
    predicted_tip_delta = (
        jacobians.tip_orientation_rad_per_mm @ independent_delta
    )
    measured_tip_delta = body_orientation_delta(
        nominal.tip_rotation,
        perturbed.tip_rotation,
    )

    assert measured_position_delta == pytest.approx(
        predicted_position_delta, abs=2.0e-8
    )
    assert measured_mid_delta == pytest.approx(predicted_mid_delta, abs=2.0e-7)
    assert measured_tip_delta == pytest.approx(predicted_tip_delta, abs=2.0e-7)


def test_body_orientation_delta_is_zero_for_equal_rotations(model):
    output = model.evaluate_from_motor_deg(np.zeros(6))
    assert body_orientation_delta(
        output.tip_rotation,
        output.tip_rotation,
    ) == pytest.approx([0.0, 0.0, 0.0])
