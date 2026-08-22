"""Control-oriented wrapper around the calibrated two-section PCC model."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np

from continuum_pccmodel.forward_kinematics import (
    forward_kinematics_from_dl_mm,
    section_transform_from_parameters,
)
from continuum_pccmodel.geometry import load_pcc_geometry_from_cfg

from .actuator_mapping import (
    ActuatorMappingError,
    ContinuumActuatorMapping,
)


class PccControlModelError(ValueError):
    """Raised when a control-model input or PCC result is invalid."""


@dataclass(frozen=True)
class PccControlOutput:
    """Geometric outputs required by NMPC and local compensation."""

    tendon_length_mm: np.ndarray
    section_pcc_dl_mm: Mapping[str, np.ndarray]
    section_transforms: Mapping[str, np.ndarray]
    section_infos: Mapping[str, Mapping]
    mid_transform: np.ndarray
    tip_transform: np.ndarray
    body_points_m: np.ndarray

    @property
    def mid_position_m(self) -> np.ndarray:
        return self.mid_transform[:3, 3].copy()

    @property
    def tip_position_m(self) -> np.ndarray:
        return self.tip_transform[:3, 3].copy()

    @property
    def mid_rotation(self) -> np.ndarray:
        return self.mid_transform[:3, :3].copy()

    @property
    def tip_rotation(self) -> np.ndarray:
        return self.tip_transform[:3, :3].copy()


@dataclass(frozen=True)
class PccControlJacobians:
    """Finite-difference Jacobians with respect to four independent mm rates."""

    tip_position_m_per_mm: np.ndarray
    mid_orientation_rad_per_mm: np.ndarray
    tip_orientation_rad_per_mm: np.ndarray

    @property
    def stacked_local_output(self) -> np.ndarray:
        """Return [tip position, mid attitude, tip attitude] as a 9x4 matrix."""

        return np.vstack(
            (
                self.tip_position_m_per_mm,
                self.mid_orientation_rad_per_mm,
                self.tip_orientation_rad_per_mm,
            )
        )


def so3_log(rotation: np.ndarray) -> np.ndarray:
    """Return the principal rotation vector of a 3x3 rotation matrix."""

    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise PccControlModelError("rotation must be a finite 3x3 matrix")

    orthogonality_error = np.linalg.norm(matrix.T @ matrix - np.eye(3))
    determinant_error = abs(float(np.linalg.det(matrix)) - 1.0)
    if orthogonality_error > 1.0e-6 or determinant_error > 1.0e-6:
        raise PccControlModelError("rotation matrix is not orthonormal")

    cosine = float(np.clip((np.trace(matrix) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    vee = np.array(
        [
            matrix[2, 1] - matrix[1, 2],
            matrix[0, 2] - matrix[2, 0],
            matrix[1, 0] - matrix[0, 1],
        ],
        dtype=float,
    )

    if angle < 1.0e-8:
        return 0.5 * vee

    if math.pi - angle < 1.0e-6:
        eigenvalues, eigenvectors = np.linalg.eig(matrix)
        index = int(np.argmin(np.abs(eigenvalues - 1.0)))
        axis = np.real(eigenvectors[:, index])
        norm = float(np.linalg.norm(axis))
        if norm < 1.0e-12:
            raise PccControlModelError("cannot recover pi-rotation axis")
        return angle * axis / norm

    return angle * vee / (2.0 * math.sin(angle))


def body_orientation_delta(
    rotation_from: np.ndarray,
    rotation_to: np.ndarray,
) -> np.ndarray:
    """Rotation from ``rotation_from`` to ``rotation_to`` in body coordinates."""

    first = np.asarray(rotation_from, dtype=float)
    second = np.asarray(rotation_to, dtype=float)
    return so3_log(first.T @ second)


class PccControlModel:
    """Evaluate calibrated PCC geometry in the final actuator coordinates."""

    def __init__(
        self,
        mapping: ContinuumActuatorMapping,
        geometry,
        *,
        samples_per_section: int = 5,
    ) -> None:
        self.mapping = mapping
        self.geometry = geometry
        self.samples_per_section = int(samples_per_section)
        if self.samples_per_section < 1:
            raise PccControlModelError("samples_per_section must be at least one")
        if tuple(geometry.chain_order) != mapping.chain_order:
            raise PccControlModelError(
                "geometry and actuator mapping chain_order do not match"
            )
        if mapping.chain_order != ("section2", "section1"):
            raise PccControlModelError(
                "final robot chain must be section2 -> section1"
            )

    @classmethod
    def from_config(
        cls,
        cfg: Mapping,
        *,
        samples_per_section: int = 5,
    ) -> "PccControlModel":
        """Build the complete control model from robot calibration contents."""

        try:
            mapping = ContinuumActuatorMapping.from_config(cfg)
            geometry = load_pcc_geometry_from_cfg(cfg)
        except (ActuatorMappingError, KeyError, TypeError, ValueError) as exc:
            raise PccControlModelError(f"invalid calibration: {exc}") from exc
        return cls(
            mapping,
            geometry,
            samples_per_section=samples_per_section,
        )

    def evaluate_from_motor_deg(
        self,
        motor_position_deg: Sequence[float],
    ) -> PccControlOutput:
        """Evaluate geometry directly from six zero-referenced motor angles."""

        tendon = self.mapping.motor_deg_to_tendon_mm(motor_position_deg)
        return self.evaluate_from_tendon_mm(tendon)

    def evaluate_from_tendon_mm(
        self,
        tendon_length_mm: Sequence[float],
    ) -> PccControlOutput:
        """Evaluate mid/tip transforms and sampled body points."""

        tendon = self.mapping._vector(
            tendon_length_mm,
            6,
            "tendon_length_mm",
        )
        section_dl = self.mapping.pcc_sections_from_tendon_mm(tendon)
        try:
            fk = forward_kinematics_from_dl_mm(
                dl_sections_mm=[
                    section_dl[name].tolist()
                    for name in self.mapping.chain_order
                ],
                geometry=self.geometry,
                section_names=list(self.mapping.chain_order),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise PccControlModelError(f"PCC evaluation failed: {exc}") from exc

        pose_by_name = {
            str(pose["section_name"]): pose
            for pose in fk["section_poses"]
        }
        info_by_name = {
            name: fk["section_infos"][index]
            for index, name in enumerate(self.mapping.chain_order)
        }
        if set(pose_by_name) != set(self.mapping.chain_order):
            raise PccControlModelError("PCC result has unexpected section names")

        section_transforms = {
            name: np.asarray(pose_by_name[name]["T"], dtype=float).copy()
            for name in self.mapping.chain_order
        }
        mid_transform = section_transforms["section2"].copy()
        tip_transform = np.asarray(fk["T_end"], dtype=float).copy()
        body_points = self._sample_body_points(
            info_by_name,
            section_transforms,
        )

        if not np.allclose(
            section_transforms["section1"],
            tip_transform,
            atol=1.0e-12,
            rtol=0.0,
        ):
            raise PccControlModelError(
                "distal section cumulative transform does not equal tip transform"
            )

        return PccControlOutput(
            tendon_length_mm=tendon.copy(),
            section_pcc_dl_mm={
                name: values.copy() for name, values in section_dl.items()
            },
            section_transforms=section_transforms,
            section_infos=info_by_name,
            mid_transform=mid_transform,
            tip_transform=tip_transform,
            body_points_m=body_points,
        )

    def _sample_body_points(
        self,
        info_by_name: Mapping[str, Mapping],
        section_transforms: Mapping[str, np.ndarray],
    ) -> np.ndarray:
        points = [np.zeros(3, dtype=float)]
        section_base = np.eye(4, dtype=float)

        for section_name in self.mapping.chain_order:
            info = info_by_name[section_name]
            full_length = float(info["arc_length"])
            for sample_index in range(1, self.samples_per_section + 1):
                fraction = sample_index / self.samples_per_section
                local = section_transform_from_parameters(
                    kappa=float(info["kappa"]),
                    phi=float(info["phi"]),
                    arc_length=full_length * fraction,
                )
                global_sample = section_base @ local
                points.append(global_sample[:3, 3].copy())
            section_base = section_transforms[section_name]

        result = np.asarray(points, dtype=float)
        expected_shape = (1 + 2 * self.samples_per_section, 3)
        if result.shape != expected_shape or not np.all(np.isfinite(result)):
            raise PccControlModelError("invalid sampled body point result")
        return result

    def finite_difference_jacobians(
        self,
        tendon_length_mm: Sequence[float],
        *,
        step_mm: float = 1.0e-3,
    ) -> PccControlJacobians:
        """Differentiate the 9D local output with central finite differences."""

        tendon = self.mapping._vector(
            tendon_length_mm,
            6,
            "tendon_length_mm",
        )
        step = float(step_mm)
        if not math.isfinite(step) or step <= 0.0:
            raise PccControlModelError("step_mm must be finite and positive")

        position_jacobian = np.zeros((3, 4), dtype=float)
        mid_orientation_jacobian = np.zeros((3, 4), dtype=float)
        tip_orientation_jacobian = np.zeros((3, 4), dtype=float)

        for column in range(4):
            independent_delta = np.zeros(4, dtype=float)
            independent_delta[column] = step
            tendon_delta = self.mapping.independent_to_tendon_velocity_mm_s(
                independent_delta
            )
            plus = self.evaluate_from_tendon_mm(tendon + tendon_delta)
            minus = self.evaluate_from_tendon_mm(tendon - tendon_delta)

            position_jacobian[:, column] = (
                plus.tip_position_m - minus.tip_position_m
            ) / (2.0 * step)
            mid_orientation_jacobian[:, column] = body_orientation_delta(
                minus.mid_rotation,
                plus.mid_rotation,
            ) / (2.0 * step)
            tip_orientation_jacobian[:, column] = body_orientation_delta(
                minus.tip_rotation,
                plus.tip_rotation,
            ) / (2.0 * step)

        stacked = np.vstack(
            (
                position_jacobian,
                mid_orientation_jacobian,
                tip_orientation_jacobian,
            )
        )
        if not np.all(np.isfinite(stacked)):
            raise PccControlModelError("PCC Jacobian contains non-finite values")

        return PccControlJacobians(
            tip_position_m_per_mm=position_jacobian,
            mid_orientation_rad_per_mm=mid_orientation_jacobian,
            tip_orientation_rad_per_mm=tip_orientation_jacobian,
        )
