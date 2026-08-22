from pathlib import Path

import numpy as np
import pytest
import yaml

from continuum_mpc.pcc_control_model import PccControlModel
from continuum_mpc.pcc_nmpc_solver import (
    PccNmpcConfig,
    PccNmpcReference,
    PccNmpcSolver,
    PccNmpcSolverError,
    PccNmpcWeights,
)


@pytest.fixture(scope="module")
def model():
    src_root = Path(__file__).resolve().parents[2]
    calibration_file = (
        src_root / "robot_bringup" / "config" / "robot_calibration.yaml"
    )
    cfg = yaml.safe_load(calibration_file.read_text(encoding="utf-8"))
    return PccControlModel.from_config(cfg, samples_per_section=3)


@pytest.fixture()
def solver(model):
    config = PccNmpcConfig(
        control_period_sec=0.1,
        horizon_steps=5,
        max_independent_velocity_mm_s=(4.0, 4.0, 4.0, 4.0),
        max_tendon_velocity_mm_s=(6.0, 6.0, 6.0, 6.0, 6.0, 6.0),
        optimizer_max_iterations=80,
        max_solve_time_sec=5.0,
    )
    weights = PccNmpcWeights(
        tip_position=(500000.0, 500000.0, 500000.0),
        tendon_velocity=(0.002, 0.002, 0.002, 0.002, 0.002, 0.002),
        tendon_velocity_delta=(
            0.004,
            0.004,
            0.004,
            0.004,
            0.004,
            0.004,
        ),
        curvature_regularization=(0.0, 0.0),
    )
    return PccNmpcSolver(model, config=config, weights=weights)


def test_zero_reference_keeps_zero_control(model, solver):
    current = np.zeros(6)
    target = model.evaluate_from_tendon_mm(current).tip_position_m

    result = solver.solve(
        current,
        PccNmpcReference(tip_position_m=target),
    )

    assert result.valid
    assert result.status == "success"
    assert result.independent_velocity_mm_s == pytest.approx(
        np.zeros((5, 4)), abs=2.0e-6
    )
    assert result.tendon_velocity_mm_s == pytest.approx(
        np.zeros((5, 6)), abs=2.0e-6
    )


def test_reachable_position_target_reduces_terminal_error(model, solver):
    current = np.zeros(6)
    actuation = model.mapping.independent_to_tendon_matrix()
    target_tendon = current + actuation @ np.array([0.8, -0.4, 0.6, -0.3])
    current_tip = model.evaluate_from_tendon_mm(current).tip_position_m
    target_tip = model.evaluate_from_tendon_mm(target_tendon).tip_position_m

    result = solver.solve(
        current,
        PccNmpcReference(tip_position_m=target_tip),
    )

    initial_error = np.linalg.norm(current_tip - target_tip)
    terminal_error = np.linalg.norm(
        result.predicted_tip_positions_m[-1] - target_tip
    )
    assert result.valid, result.status
    assert terminal_error < 0.35 * initial_error
    assert np.max(np.abs(result.independent_velocity_mm_s)) <= 4.0 + 1.0e-6
    assert np.max(np.abs(result.tendon_velocity_mm_s)) <= 6.0 + 1.0e-6
    for tendon_rate in result.tendon_velocity_mm_s:
        sums = model.mapping.modeled_section_sums(tendon_rate)
        assert sums["section2"] == pytest.approx(0.0, abs=1.0e-10)
        assert sums["section1"] == pytest.approx(0.0, abs=1.0e-10)


def test_prediction_respects_tightened_motor_position_bounds(model, solver):
    current = np.zeros(6)
    target_tendon = (
        model.mapping.independent_to_tendon_matrix()
        @ np.array([1.0, -0.5, -0.8, 0.4])
    )
    target_tip = model.evaluate_from_tendon_mm(target_tendon).tip_position_m

    result = solver.solve(
        current,
        PccNmpcReference(tip_position_m=target_tip),
    )

    lower, upper = model.mapping.tendon_position_bounds_mm(
        motor_limit_margin_deg=2.0
    )
    assert result.valid, result.status
    assert np.all(result.tendon_trajectory_mm[1:] >= lower - 1.0e-6)
    assert np.all(result.tendon_trajectory_mm[1:] <= upper + 1.0e-6)


def test_shape_only_reference_is_supported(model, solver):
    current = np.zeros(6)
    output = model.evaluate_from_tendon_mm(current)
    shape = {}
    for section_name in model.mapping.chain_order:
        info = output.section_infos[section_name]
        shape[section_name] = [
            info["kappa"] * np.cos(info["phi"]),
            info["kappa"] * np.sin(info["phi"]),
        ]

    result = solver.solve(
        current,
        PccNmpcReference(section_curvature_xy_1pm=shape),
    )

    assert result.valid
    assert result.first_tendon_velocity_mm_s == pytest.approx(
        np.zeros(6), abs=1.0e-6
    )


def test_enabled_curvature_constraint_path_is_feasible_at_zero(model):
    constrained_solver = PccNmpcSolver(
        model,
        config=PccNmpcConfig(
            horizon_steps=2,
            control_blocks=1,
            max_curvature_1pm=(8.0, 8.0),
            max_solve_time_sec=5.0,
        ),
    )
    current = np.zeros(6)
    target = model.evaluate_from_tendon_mm(current).tip_position_m

    result = constrained_solver.solve(
        current,
        PccNmpcReference(tip_position_m=target),
    )

    assert result.valid, result.status


def test_invalid_reference_and_out_of_bounds_state_fail_closed(model, solver):
    with pytest.raises(PccNmpcSolverError, match="at least one"):
        solver.solve(np.zeros(6), PccNmpcReference())

    _, upper = model.mapping.tendon_position_bounds_mm(
        motor_limit_margin_deg=2.0
    )
    unsafe_current = np.zeros(6)
    unsafe_current[0] = upper[0] + 1.0
    target = model.evaluate_from_tendon_mm(np.zeros(6)).tip_position_m
    result = solver.solve(
        unsafe_current,
        PccNmpcReference(tip_position_m=target),
    )

    assert not result.valid
    assert result.status == "current_state_outside_tendon_position_bounds"
    assert result.first_tendon_velocity_mm_s == pytest.approx(np.zeros(6))
