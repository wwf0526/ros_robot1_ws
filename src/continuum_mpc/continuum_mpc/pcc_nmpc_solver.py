"""Constrained nonlinear MPC over the calibrated two-section PCC model.

The solver is ROS independent.  Its state is the six physical tendon length
changes in tendon-id order 1..6, while each control move contains the four
independent section rates in base-to-tip order.  Every shooting step evaluates
the nonlinear PCC forward model.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from time import perf_counter
from typing import Mapping, Optional, Sequence

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, NonlinearConstraint
from scipy.optimize import minimize

from .actuator_mapping import ActuatorMappingError
from .pcc_control_model import (
    PccControlModel,
    PccControlModelError,
    PccControlOutput,
    body_orientation_delta,
)


class PccNmpcSolverError(ValueError):
    """Raised when an NMPC configuration or reference is invalid."""


class _SolveTimeout(RuntimeError):
    """Internal exception used to stop an over-budget optimization."""


def _finite_vector(
    values: Sequence[float],
    size: int,
    name: str,
    *,
    nonnegative: bool = False,
    positive: bool = False,
) -> np.ndarray:
    array = np.asarray(values, dtype=float).reshape(-1)
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        raise PccNmpcSolverError(f"{name} must be a finite ({size},) vector")
    if positive and np.any(array <= 0.0):
        raise PccNmpcSolverError(f"{name} must be strictly positive")
    if nonnegative and np.any(array < 0.0):
        raise PccNmpcSolverError(f"{name} must be nonnegative")
    return array


@dataclass(frozen=True)
class PccNmpcWeights:
    """Quadratic weights for task tracking and control regularization."""

    tip_position: tuple[float, float, float] = (
        200000.0,
        200000.0,
        200000.0,
    )
    tip_orientation: tuple[float, float, float] = (40.0, 40.0, 10.0)
    section_shape: tuple[float, float, float, float] = (
        0.2,
        0.2,
        0.2,
        0.2,
    )
    tendon_velocity: tuple[float, float, float, float, float, float] = (
        0.02,
        0.02,
        0.02,
        0.02,
        0.02,
        0.02,
    )
    tendon_velocity_delta: tuple[float, float, float, float, float, float] = (
        0.08,
        0.08,
        0.08,
        0.08,
        0.08,
        0.08,
    )
    curvature_regularization: tuple[float, float] = (0.002, 0.002)
    terminal_multiplier: float = 4.0


@dataclass(frozen=True)
class PccNmpcConfig:
    """Prediction, actuator and numerical constraints for NMPC."""

    control_period_sec: float = 0.1
    horizon_steps: int = 6
    control_blocks: int = 2
    max_independent_velocity_mm_s: tuple[float, float, float, float] = (
        4.0,
        4.0,
        4.0,
        4.0,
    )
    max_tendon_velocity_mm_s: tuple[
        float,
        float,
        float,
        float,
        float,
        float,
    ] = (6.0, 6.0, 6.0, 6.0, 6.0, 6.0)
    motor_limit_margin_deg: float = 2.0
    minimum_remaining_tendon_length_mm: float = 10.0
    max_curvature_1pm: Optional[tuple[float, float]] = None
    optimizer_max_iterations: int = 50
    optimizer_ftol: float = 1.0e-7
    optimizer_fd_step_mm_s: float = 1.0e-3
    max_solve_time_sec: float = 0.25
    feasibility_tolerance: float = 1.0e-6


@dataclass(frozen=True)
class PccNmpcReference:
    """Any supported combination of tip pose and two-section shape targets."""

    tip_position_m: Optional[Sequence[float]] = None
    tip_rotation: Optional[np.ndarray] = None
    section_curvature_xy_1pm: Optional[Mapping[str, Sequence[float]]] = None


@dataclass(frozen=True)
class PccNmpcResult:
    """Safe-to-consume NMPC result and complete prediction diagnostics."""

    valid: bool
    status: str
    objective_value: float
    solve_time_ms: float
    iterations: int
    independent_velocity_mm_s: np.ndarray
    tendon_velocity_mm_s: np.ndarray
    tendon_trajectory_mm: np.ndarray
    predicted_tip_positions_m: np.ndarray

    @property
    def first_independent_velocity_mm_s(self) -> np.ndarray:
        return self.independent_velocity_mm_s[0].copy()

    @property
    def first_tendon_velocity_mm_s(self) -> np.ndarray:
        return self.tendon_velocity_mm_s[0].copy()


@dataclass(frozen=True)
class _PreparedReference:
    tip_position_m: Optional[np.ndarray]
    tip_rotation: Optional[np.ndarray]
    section_curvature_xy_1pm: Optional[np.ndarray]


class PccNmpcSolver:
    """Direct single-shooting PCC-NMPC with a shifted warm start."""

    def __init__(
        self,
        model: PccControlModel,
        *,
        config: Optional[PccNmpcConfig] = None,
        weights: Optional[PccNmpcWeights] = None,
    ) -> None:
        self.model = model
        self.config = config or PccNmpcConfig()
        self.weights = weights or PccNmpcWeights()
        self._validate_configuration()

        self._actuation = model.mapping.independent_to_tendon_matrix()
        if self._actuation.shape != (6, 4):
            raise PccNmpcSolverError("actuation matrix must have shape (6, 4)")

        self._max_independent_velocity = _finite_vector(
            self.config.max_independent_velocity_mm_s,
            4,
            "max_independent_velocity_mm_s",
            positive=True,
        )
        self._max_tendon_velocity = _finite_vector(
            self.config.max_tendon_velocity_mm_s,
            6,
            "max_tendon_velocity_mm_s",
            positive=True,
        )
        self._position_weight = _finite_vector(
            self.weights.tip_position,
            3,
            "tip_position weight",
            nonnegative=True,
        )
        self._orientation_weight = _finite_vector(
            self.weights.tip_orientation,
            3,
            "tip_orientation weight",
            nonnegative=True,
        )
        self._shape_weight = _finite_vector(
            self.weights.section_shape,
            4,
            "section_shape weight",
            nonnegative=True,
        )
        self._velocity_weight = _finite_vector(
            self.weights.tendon_velocity,
            6,
            "tendon_velocity weight",
            nonnegative=True,
        )
        self._velocity_delta_weight = _finite_vector(
            self.weights.tendon_velocity_delta,
            6,
            "tendon_velocity_delta weight",
            nonnegative=True,
        )
        self._curvature_regularization = _finite_vector(
            self.weights.curvature_regularization,
            2,
            "curvature_regularization weight",
            nonnegative=True,
        )
        self._max_curvature = None
        if self.config.max_curvature_1pm is not None:
            self._max_curvature = _finite_vector(
                self.config.max_curvature_1pm,
                2,
                "max_curvature_1pm",
                positive=True,
            )

        self._tendon_lower, self._tendon_upper = (
            self._build_tendon_position_bounds()
        )
        self._step_to_block = self._build_step_to_block()
        self._control_expansion = self._build_control_expansion()
        self._linear_matrix = self._build_linear_constraint_matrix()
        self._warm_start: Optional[np.ndarray] = None

    def _validate_configuration(self) -> None:
        cfg = self.config
        finite_positive = {
            "control_period_sec": cfg.control_period_sec,
            "optimizer_ftol": cfg.optimizer_ftol,
            "optimizer_fd_step_mm_s": cfg.optimizer_fd_step_mm_s,
            "max_solve_time_sec": cfg.max_solve_time_sec,
            "feasibility_tolerance": cfg.feasibility_tolerance,
        }
        for name, value in finite_positive.items():
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise PccNmpcSolverError(f"{name} must be finite and positive")
        if int(cfg.horizon_steps) < 1:
            raise PccNmpcSolverError("horizon_steps must be at least one")
        if int(cfg.control_blocks) < 1:
            raise PccNmpcSolverError("control_blocks must be at least one")
        if int(cfg.control_blocks) > int(cfg.horizon_steps):
            raise PccNmpcSolverError(
                "control_blocks cannot exceed horizon_steps"
            )
        if int(cfg.optimizer_max_iterations) < 1:
            raise PccNmpcSolverError(
                "optimizer_max_iterations must be at least one"
            )
        if (
            not math.isfinite(float(cfg.motor_limit_margin_deg))
            or float(cfg.motor_limit_margin_deg) < 0.0
        ):
            raise PccNmpcSolverError(
                "motor_limit_margin_deg must be finite and nonnegative"
            )
        if (
            not math.isfinite(float(cfg.minimum_remaining_tendon_length_mm))
            or float(cfg.minimum_remaining_tendon_length_mm) <= 0.0
        ):
            raise PccNmpcSolverError(
                "minimum_remaining_tendon_length_mm must be positive"
            )
        if (
            not math.isfinite(float(self.weights.terminal_multiplier))
            or float(self.weights.terminal_multiplier) < 1.0
        ):
            raise PccNmpcSolverError(
                "terminal_multiplier must be finite and at least one"
            )

    def reset(self) -> None:
        """Clear the shifted solution used to warm-start the next solve."""

        self._warm_start = None

    def _build_tendon_position_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        try:
            lower, upper = self.model.mapping.tendon_position_bounds_mm(
                motor_limit_margin_deg=self.config.motor_limit_margin_deg
            )
        except ActuatorMappingError as exc:
            raise PccNmpcSolverError(str(exc)) from exc

        lower = lower.copy()
        upper = upper.copy()
        remaining = float(self.config.minimum_remaining_tendon_length_mm)
        for section_name in self.model.mapping.chain_order:
            section = self.model.mapping.sections[section_name]
            section_geometry = self.model.geometry.sections[section_name]
            maximum_model_dl = (
                1000.0 * section_geometry.initial_tendon_length_m - remaining
            )
            if maximum_model_dl <= 0.0:
                raise PccNmpcSolverError(
                    f"{section_name}: minimum remaining length is too large"
                )
            for tendon_id, sign in zip(
                section.tendon_ids,
                section.dl_signs,
            ):
                index = tendon_id - 1
                boundary = maximum_model_dl / sign
                if sign > 0.0:
                    upper[index] = min(upper[index], boundary)
                else:
                    lower[index] = max(lower[index], boundary)

        if np.any(lower >= upper):
            raise PccNmpcSolverError("empty tendon position constraint range")
        return lower, upper

    def _build_step_to_block(self) -> np.ndarray:
        horizon = int(self.config.horizon_steps)
        blocks = int(self.config.control_blocks)
        return np.floor(np.arange(horizon) * blocks / horizon).astype(int)

    def _build_control_expansion(self) -> np.ndarray:
        horizon = int(self.config.horizon_steps)
        blocks = int(self.config.control_blocks)
        expansion = np.zeros((4 * horizon, 4 * blocks), dtype=float)
        for step, block in enumerate(self._step_to_block):
            rows = slice(4 * step, 4 * (step + 1))
            columns = slice(4 * block, 4 * (block + 1))
            expansion[rows, columns] = np.eye(4)
        return expansion

    def _build_linear_constraint_matrix(self) -> np.ndarray:
        horizon = int(self.config.horizon_steps)
        rate_rows = 6 * horizon
        full_matrix = np.zeros((12 * horizon, 4 * horizon), dtype=float)
        dt = float(self.config.control_period_sec)

        for step in range(horizon):
            columns = slice(4 * step, 4 * (step + 1))
            rate_slice = slice(6 * step, 6 * (step + 1))
            full_matrix[rate_slice, columns] = self._actuation

            state_slice = slice(
                rate_rows + 6 * step,
                rate_rows + 6 * (step + 1),
            )
            for previous_step in range(step + 1):
                previous_columns = slice(
                    4 * previous_step,
                    4 * (previous_step + 1),
                )
                full_matrix[state_slice, previous_columns] = (
                    dt * self._actuation
                )
        return full_matrix @ self._control_expansion

    def _prepare_reference(
        self,
        reference: PccNmpcReference,
    ) -> _PreparedReference:
        position = None
        if reference.tip_position_m is not None:
            position = _finite_vector(
                reference.tip_position_m,
                3,
                "tip_position_m",
            )

        rotation = None
        if reference.tip_rotation is not None:
            rotation = np.asarray(reference.tip_rotation, dtype=float)
            if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
                raise PccNmpcSolverError(
                    "tip_rotation must be a finite 3x3 matrix"
                )
            orthogonality = np.linalg.norm(rotation.T @ rotation - np.eye(3))
            determinant_error = abs(float(np.linalg.det(rotation)) - 1.0)
            if orthogonality > 1.0e-6 or determinant_error > 1.0e-6:
                raise PccNmpcSolverError("tip_rotation is not orthonormal")
            rotation = rotation.copy()

        shape = None
        if reference.section_curvature_xy_1pm is not None:
            values = []
            for section_name in self.model.mapping.chain_order:
                if section_name not in reference.section_curvature_xy_1pm:
                    raise PccNmpcSolverError(
                        f"shape reference is missing {section_name}"
                    )
                values.extend(
                    _finite_vector(
                        reference.section_curvature_xy_1pm[section_name],
                        2,
                        f"{section_name} curvature target",
                    ).tolist()
                )
            shape = np.asarray(values, dtype=float)

        if position is None and rotation is None and shape is None:
            raise PccNmpcSolverError("at least one NMPC target is required")
        return _PreparedReference(position, rotation, shape)

    def _section_curvature_xy(
        self,
        output: PccControlOutput,
    ) -> np.ndarray:
        values = []
        for section_name in self.model.mapping.chain_order:
            info = output.section_infos[section_name]
            kappa = float(info["kappa"])
            phi = float(info["phi"])
            values.extend((kappa * math.cos(phi), kappa * math.sin(phi)))
        return np.asarray(values, dtype=float)

    def _section_curvature_magnitude(
        self,
        output: PccControlOutput,
    ) -> np.ndarray:
        return np.asarray(
            [
                float(output.section_infos[name]["kappa"])
                for name in self.model.mapping.chain_order
            ],
            dtype=float,
        )

    def _rollout(
        self,
        current_tendon_mm: np.ndarray,
        flat_control: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, list[PccControlOutput]]:
        horizon = int(self.config.horizon_steps)
        block_controls = flat_control.reshape(
            int(self.config.control_blocks),
            4,
        )
        controls = block_controls[self._step_to_block]
        tendon_velocity = controls @ self._actuation.T
        trajectory = np.empty((horizon + 1, 6), dtype=float)
        trajectory[0] = current_tendon_mm
        outputs = []
        dt = float(self.config.control_period_sec)
        for step in range(horizon):
            trajectory[step + 1] = (
                trajectory[step] + dt * tendon_velocity[step]
            )
            outputs.append(
                self.model.evaluate_from_tendon_mm(trajectory[step + 1])
            )
        return trajectory, tendon_velocity, outputs

    def _objective(
        self,
        tendon_velocity: np.ndarray,
        outputs: Sequence[PccControlOutput],
        reference: _PreparedReference,
        previous_tendon_velocity: np.ndarray,
    ) -> float:
        total = 0.0
        horizon = int(self.config.horizon_steps)
        for step, output in enumerate(outputs):
            multiplier = 1.0
            if step == horizon - 1:
                multiplier = float(self.weights.terminal_multiplier)

            if reference.tip_position_m is not None:
                error = output.tip_position_m - reference.tip_position_m
                total += multiplier * float(
                    np.sum(self._position_weight * error * error)
                )
            if reference.tip_rotation is not None:
                error = body_orientation_delta(
                    output.tip_rotation,
                    reference.tip_rotation,
                )
                total += multiplier * float(
                    np.sum(self._orientation_weight * error * error)
                )

            curvature_xy = self._section_curvature_xy(output)
            if reference.section_curvature_xy_1pm is not None:
                error = curvature_xy - reference.section_curvature_xy_1pm
                total += multiplier * float(
                    np.sum(self._shape_weight * error * error)
                )
            else:
                magnitude = self._section_curvature_magnitude(output)
                total += multiplier * float(
                    np.sum(
                        self._curvature_regularization
                        * magnitude
                        * magnitude
                    )
                )

            velocity = tendon_velocity[step]
            total += float(np.sum(self._velocity_weight * velocity * velocity))
            difference = velocity - previous_tendon_velocity
            total += float(
                np.sum(
                    self._velocity_delta_weight * difference * difference
                )
            )
            previous_tendon_velocity = velocity
        return float(total)

    def _linear_constraint(
        self,
        current_tendon_mm: np.ndarray,
    ) -> LinearConstraint:
        horizon = int(self.config.horizon_steps)
        rate_lower = np.tile(-self._max_tendon_velocity, horizon)
        rate_upper = np.tile(self._max_tendon_velocity, horizon)
        state_lower = np.tile(
            self._tendon_lower - current_tendon_mm,
            horizon,
        )
        state_upper = np.tile(
            self._tendon_upper - current_tendon_mm,
            horizon,
        )
        return LinearConstraint(
            self._linear_matrix,
            np.concatenate((rate_lower, state_lower)),
            np.concatenate((rate_upper, state_upper)),
        )

    def _initial_guess(
        self,
        linear_constraint: LinearConstraint,
    ) -> np.ndarray:
        blocks = int(self.config.control_blocks)
        if self._warm_start is None:
            guess = np.zeros((blocks, 4), dtype=float)
        else:
            guess = self._warm_start.copy()
        guess = np.clip(
            guess,
            -self._max_independent_velocity,
            self._max_independent_velocity,
        )
        flat = guess.reshape(-1)
        values = self._linear_matrix @ flat
        tolerance = float(self.config.feasibility_tolerance)
        if (
            np.any(values < np.asarray(linear_constraint.lb) - tolerance)
            or np.any(values > np.asarray(linear_constraint.ub) + tolerance)
        ):
            flat = np.zeros(4 * blocks, dtype=float)
        return flat

    def _invalid_result(
        self,
        current_tendon_mm: np.ndarray,
        current_output: Optional[PccControlOutput],
        *,
        status: str,
        start_time: float,
        objective_value: float = math.inf,
        iterations: int = 0,
    ) -> PccNmpcResult:
        horizon = int(self.config.horizon_steps)
        tip = np.zeros(3, dtype=float)
        if current_output is not None:
            tip = current_output.tip_position_m
        return PccNmpcResult(
            valid=False,
            status=status,
            objective_value=float(objective_value),
            solve_time_ms=1000.0 * (perf_counter() - start_time),
            iterations=int(iterations),
            independent_velocity_mm_s=np.zeros((horizon, 4), dtype=float),
            tendon_velocity_mm_s=np.zeros((horizon, 6), dtype=float),
            tendon_trajectory_mm=np.tile(current_tendon_mm, (horizon + 1, 1)),
            predicted_tip_positions_m=np.tile(tip, (horizon, 1)),
        )

    def solve(
        self,
        current_tendon_mm: Sequence[float],
        reference: PccNmpcReference,
        *,
        previous_independent_velocity_mm_s: Optional[
            Sequence[float]
        ] = None,
    ) -> PccNmpcResult:
        """Solve one receding-horizon problem without applying its output."""

        start_time = perf_counter()
        current = _finite_vector(
            current_tendon_mm,
            6,
            "current_tendon_mm",
        )
        prepared_reference = self._prepare_reference(reference)
        previous_independent = np.zeros(4, dtype=float)
        if previous_independent_velocity_mm_s is not None:
            previous_independent = _finite_vector(
                previous_independent_velocity_mm_s,
                4,
                "previous_independent_velocity_mm_s",
            )
        previous_tendon_velocity = self._actuation @ previous_independent

        try:
            current_output = self.model.evaluate_from_tendon_mm(current)
        except (PccControlModelError, ValueError) as exc:
            return self._invalid_result(
                current,
                None,
                status=f"invalid_current_state: {exc}",
                start_time=start_time,
            )

        tolerance = float(self.config.feasibility_tolerance)
        if (
            np.any(current < self._tendon_lower - tolerance)
            or np.any(current > self._tendon_upper + tolerance)
        ):
            return self._invalid_result(
                current,
                current_output,
                status="current_state_outside_tendon_position_bounds",
                start_time=start_time,
            )
        if self._max_curvature is not None:
            current_curvature = self._section_curvature_magnitude(
                current_output
            )
            if np.any(current_curvature > self._max_curvature + tolerance):
                return self._invalid_result(
                    current,
                    current_output,
                    status="current_state_outside_curvature_bounds",
                    start_time=start_time,
                )

        linear_constraint = self._linear_constraint(current)
        initial_guess = self._initial_guess(linear_constraint)
        horizon = int(self.config.horizon_steps)
        blocks = int(self.config.control_blocks)
        bounds = Bounds(
            np.tile(-self._max_independent_velocity, blocks),
            np.tile(self._max_independent_velocity, blocks),
        )

        cached_flat: Optional[np.ndarray] = None
        cached_rollout = None

        def rollout(flat_control):
            nonlocal cached_flat, cached_rollout
            vector = np.asarray(flat_control, dtype=float)
            if cached_flat is None or not np.array_equal(vector, cached_flat):
                cached_flat = vector.copy()
                cached_rollout = self._rollout(current, vector)
            return cached_rollout

        def objective(flat_control):
            elapsed = perf_counter() - start_time
            if elapsed > float(self.config.max_solve_time_sec):
                raise _SolveTimeout
            _, tendon_velocity, outputs = rollout(flat_control)
            return self._objective(
                tendon_velocity,
                outputs,
                prepared_reference,
                previous_tendon_velocity,
            )

        constraints = [linear_constraint]
        if self._max_curvature is not None:

            def curvature_constraint(flat_control):
                _, _, outputs = rollout(flat_control)
                return np.concatenate(
                    [
                        self._section_curvature_magnitude(output)
                        for output in outputs
                    ]
                )

            constraints.append(
                NonlinearConstraint(
                    curvature_constraint,
                    np.zeros(2 * horizon, dtype=float),
                    np.tile(self._max_curvature, horizon),
                )
            )

        try:
            result = minimize(
                objective,
                initial_guess,
                method="SLSQP",
                bounds=bounds,
                constraints=constraints,
                options={
                    "maxiter": int(self.config.optimizer_max_iterations),
                    "ftol": float(self.config.optimizer_ftol),
                    "eps": float(self.config.optimizer_fd_step_mm_s),
                    "disp": False,
                },
            )
        except _SolveTimeout:
            return self._invalid_result(
                current,
                current_output,
                status="optimizer_time_budget_exceeded",
                start_time=start_time,
            )
        except (PccControlModelError, ValueError, FloatingPointError) as exc:
            return self._invalid_result(
                current,
                current_output,
                status=f"optimizer_exception: {exc}",
                start_time=start_time,
            )

        objective_value = float(result.fun)
        iterations = int(getattr(result, "nit", 0))
        if not result.success or not np.all(np.isfinite(result.x)):
            return self._invalid_result(
                current,
                current_output,
                status=f"optimizer_failed: {result.message}",
                start_time=start_time,
                objective_value=objective_value,
                iterations=iterations,
            )

        flat_solution = np.asarray(result.x, dtype=float)
        linear_values = self._linear_matrix @ flat_solution
        if (
            np.any(linear_values < np.asarray(linear_constraint.lb) - tolerance)
            or np.any(linear_values > np.asarray(linear_constraint.ub) + tolerance)
        ):
            return self._invalid_result(
                current,
                current_output,
                status="optimizer_returned_infeasible_linear_solution",
                start_time=start_time,
                objective_value=objective_value,
                iterations=iterations,
            )

        try:
            trajectory, tendon_velocity, outputs = rollout(flat_solution)
        except (PccControlModelError, ValueError) as exc:
            return self._invalid_result(
                current,
                current_output,
                status=f"invalid_prediction: {exc}",
                start_time=start_time,
                objective_value=objective_value,
                iterations=iterations,
            )
        if self._max_curvature is not None:
            predicted_curvature = np.vstack(
                [
                    self._section_curvature_magnitude(output)
                    for output in outputs
                ]
            )
            if np.any(
                predicted_curvature
                > self._max_curvature.reshape(1, 2) + tolerance
            ):
                return self._invalid_result(
                    current,
                    current_output,
                    status="optimizer_returned_infeasible_curvature_solution",
                    start_time=start_time,
                    objective_value=objective_value,
                    iterations=iterations,
                )

        block_controls = flat_solution.reshape(blocks, 4)
        controls = block_controls[self._step_to_block]
        shifted_controls = np.vstack((controls[1:], controls[-1]))
        self._warm_start = np.vstack(
            [
                np.mean(
                    shifted_controls[self._step_to_block == block],
                    axis=0,
                )
                for block in range(blocks)
            ]
        )
        return PccNmpcResult(
            valid=True,
            status="success",
            objective_value=objective_value,
            solve_time_ms=1000.0 * (perf_counter() - start_time),
            iterations=iterations,
            independent_velocity_mm_s=controls.copy(),
            tendon_velocity_mm_s=tendon_velocity.copy(),
            tendon_trajectory_mm=trajectory.copy(),
            predicted_tip_positions_m=np.vstack(
                [output.tip_position_m for output in outputs]
            ),
        )
