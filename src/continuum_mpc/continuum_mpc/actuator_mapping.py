"""Validated motor, tendon and four-DOF actuation mappings.

解决“机器人需要的连续体运动”与“电机能够执行的动作”之间的转换问题。

All public tendon vectors use physical tendon-id order 1..6.  The four
independent rates use two coordinates per section in base-to-tip
``pcc_model.chain_order``.  For each section the third PCC-slot rate is
``-(u1 + u2)``, so the modeled three-tendon sum is exactly zero.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np


class ActuatorMappingError(ValueError):
    """Raised when calibration or an actuator vector is invalid."""


@dataclass(frozen=True)
class SectionActuation:
    """One section's PCC-slot mapping to physical tendon ids."""

    name: str
    tendon_ids: tuple[int, int, int]
    dl_signs: tuple[float, float, float]


def _value(mapping: Mapping, key):
    """Read YAML dictionaries whose numeric keys may be int or string."""

    if key in mapping:
        return mapping[key]
    text = str(key)
    if text in mapping:
        return mapping[text]
    raise ActuatorMappingError(f"missing calibration key: {key}")


class ContinuumActuatorMapping:
    """Single source of truth for the robot's final actuator coordinates."""

    def __init__(
        self,
        *,
        motor_ids: Sequence[int],
        motor_to_tendon: Mapping[int, int],
        motor_sign: Mapping[int, float],
        spool_radius_mm: Mapping[int, float],
        position_limits_deg: Mapping[int, tuple[float, float]],
        chain_order: Sequence[str],
        sections: Mapping[str, SectionActuation],
    ) -> None:
        self.motor_ids = tuple(int(value) for value in motor_ids)
        if len(self.motor_ids) != 6 or len(set(self.motor_ids)) != 6:
            raise ActuatorMappingError("exactly six unique motor ids are required")

        self.motor_to_tendon = {
            int(mid): int(_value(motor_to_tendon, mid))
            for mid in self.motor_ids
        }
        tendon_ids = tuple(self.motor_to_tendon[mid] for mid in self.motor_ids)
        if set(tendon_ids) != set(range(1, 7)):
            raise ActuatorMappingError(
                "motor_to_tendon must be a bijection onto tendon ids 1..6"
            )
        self.tendon_to_motor = {
            tendon_id: motor_id
            for motor_id, tendon_id in self.motor_to_tendon.items()
        }

        self.motor_sign = {
            int(mid): float(_value(motor_sign, mid))
            for mid in self.motor_ids
        }
        self.spool_radius_mm = {
            int(mid): float(_value(spool_radius_mm, mid))
            for mid in self.motor_ids
        }
        self.position_limits_deg = {
            int(mid): (
                float(_value(position_limits_deg, mid)[0]),
                float(_value(position_limits_deg, mid)[1]),
            )
            for mid in self.motor_ids
        }

        for motor_id in self.motor_ids:
            sign = self.motor_sign[motor_id]
            radius = self.spool_radius_mm[motor_id]
            low, high = self.position_limits_deg[motor_id]
            if not math.isfinite(sign) or abs(sign) < 1.0e-12:
                raise ActuatorMappingError(
                    f"motor {motor_id}: sign must be finite and nonzero"
                )
            if not math.isfinite(radius) or radius <= 0.0:
                raise ActuatorMappingError(
                    f"motor {motor_id}: spool radius must be positive"
                )
            if not (math.isfinite(low) and math.isfinite(high) and low < high):
                raise ActuatorMappingError(
                    f"motor {motor_id}: invalid position limits"
                )

        self.chain_order = tuple(str(value) for value in chain_order)
        if len(self.chain_order) != 2 or len(set(self.chain_order)) != 2:
            raise ActuatorMappingError(
                "two unique sections in base-to-tip chain order are required"
            )

        self.sections = {}
        used_tendons: list[int] = []
        for section_name in self.chain_order:
            if section_name not in sections:
                raise ActuatorMappingError(
                    f"missing section mapping: {section_name}"
                )
            section = sections[section_name]
            ids = tuple(int(value) for value in section.tendon_ids)
            signs = tuple(float(value) for value in section.dl_signs)
            if len(ids) != 3 or len(set(ids)) != 3:
                raise ActuatorMappingError(
                    f"{section_name}: three unique tendon ids are required"
                )
            if len(signs) != 3 or any(
                not math.isfinite(value) or abs(value) < 1.0e-12
                for value in signs
            ):
                raise ActuatorMappingError(
                    f"{section_name}: three finite nonzero dl_signs are required"
                )
            self.sections[section_name] = SectionActuation(
                name=section_name,
                tendon_ids=ids,
                dl_signs=signs,
            )
            used_tendons.extend(ids)

        if set(used_tendons) != set(range(1, 7)) or len(used_tendons) != 6:
            raise ActuatorMappingError(
                "section PCC mappings must use every tendon id exactly once"
            )

    @classmethod
    def from_config(cls, cfg: Mapping) -> "ContinuumActuatorMapping":
        """Build the mapping from ``robot_calibration.yaml`` contents."""

        motor_cfg = cfg["motor"]
        tendon_cfg = cfg["tendon"]
        pcc_cfg = cfg["pcc_model"]
        chain_order = [str(value) for value in pcc_cfg["chain_order"]]

        sections = {}
        for section_name in chain_order:
            section_cfg = pcc_cfg["sections"][section_name]
            sections[section_name] = SectionActuation(
                name=section_name,
                tendon_ids=tuple(
                    int(value) for value in section_cfg["tendon_ids"]
                ),
                dl_signs=tuple(
                    float(value)
                    for value in section_cfg.get("dl_signs", [1.0, 1.0, 1.0])
                ),
            )

        return cls(
            motor_ids=motor_cfg["motor_ids"],
            motor_to_tendon=tendon_cfg["motor_to_tendon"],
            motor_sign=motor_cfg["sign"],
            spool_radius_mm=motor_cfg["spool_radius_mm"],
            position_limits_deg=motor_cfg["position_limit_deg"],
            chain_order=chain_order,
            sections=sections,
        )

    @staticmethod
    def _vector(values: Sequence[float], size: int, name: str) -> np.ndarray:
        array = np.asarray(values, dtype=float).reshape(-1)
        if array.shape != (size,):
            raise ActuatorMappingError(
                f"{name} must have shape ({size},), got {array.shape}"
            )
        if not np.all(np.isfinite(array)):
            raise ActuatorMappingError(f"{name} contains non-finite values")
        return array

    def motor_deg_to_tendon_mm(
        self,
        motor_position_deg: Sequence[float],
    ) -> np.ndarray:
        """Convert zero-referenced motor angles to physical tendon changes."""

        motor = self._vector(motor_position_deg, 6, "motor_position_deg")
        tendon = np.zeros(6, dtype=float)
        for index, motor_id in enumerate(self.motor_ids):
            tendon_id = self.motor_to_tendon[motor_id]
            tendon[tendon_id - 1] = (
                self.motor_sign[motor_id]
                * self.spool_radius_mm[motor_id]
                * math.radians(float(motor[index]))
            )
        return tendon

    def tendon_mm_to_motor_deg(
        self,
        tendon_length_mm: Sequence[float],
        *,
        enforce_limits: bool = True,
    ) -> np.ndarray:
        """Convert physical tendon changes to zero-referenced motor angles."""

        tendon = self._vector(tendon_length_mm, 6, "tendon_length_mm")
        motor = np.zeros(6, dtype=float)
        for index, motor_id in enumerate(self.motor_ids):
            tendon_id = self.motor_to_tendon[motor_id]
            denominator = (
                self.motor_sign[motor_id] * self.spool_radius_mm[motor_id]
            )
            angle = math.degrees(float(tendon[tendon_id - 1]) / denominator)
            if enforce_limits:
                low, high = self.position_limits_deg[motor_id]
                if angle < low or angle > high:
                    raise ActuatorMappingError(
                        f"motor {motor_id}: mapped target {angle:.6f} deg "
                        f"outside [{low:.6f}, {high:.6f}] deg"
                    )
            motor[index] = angle
        return motor

    def pcc_sections_from_tendon_mm(
        self,
        tendon_length_mm: Sequence[float],
    ) -> dict[str, np.ndarray]:
        """Return PCC DL vectors in each section's configured slot order."""

        tendon = self._vector(tendon_length_mm, 6, "tendon_length_mm")
        result = {}
        for section_name in self.chain_order:
            section = self.sections[section_name]
            result[section_name] = np.array(
                [
                    tendon[tendon_id - 1] * sign
                    for tendon_id, sign in zip(
                        section.tendon_ids,
                        section.dl_signs,
                    )
                ],
                dtype=float,
            )
        return result

    def independent_to_tendon_velocity_mm_s(
        self,
        independent_velocity_mm_s: Sequence[float],
    ) -> np.ndarray:
        """Map four base-to-tip independent rates to six physical tendons."""

        independent = self._vector(
            independent_velocity_mm_s,
            4,
            "independent_velocity_mm_s",
        )
        tendon = np.zeros(6, dtype=float)
        for section_index, section_name in enumerate(self.chain_order):
            first = float(independent[2 * section_index])
            second = float(independent[2 * section_index + 1])
            model_rates = (first, second, -(first + second))
            section = self.sections[section_name]
            for tendon_id, sign, model_rate in zip(
                section.tendon_ids,
                section.dl_signs,
                model_rates,
            ):
                tendon[tendon_id - 1] = model_rate / sign
        return tendon

    def independent_to_tendon_matrix(self) -> np.ndarray:
        """Return the constant 6x4 differential actuation matrix.

        Multiplying this matrix by the four independent section rates produces
        the six physical tendon rates.  The column order follows
        ``chain_order`` and the row order is physical tendon id 1..6.
        """

        matrix = np.zeros((6, 4), dtype=float)
        for column in range(4):
            basis = np.zeros(4, dtype=float)
            basis[column] = 1.0
            matrix[:, column] = (
                self.independent_to_tendon_velocity_mm_s(basis)
            )
        return matrix

    def tendon_position_bounds_mm(
        self,
        *,
        motor_limit_margin_deg: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Convert calibrated motor limits into physical tendon bounds.

        A positive margin tightens both sides of every motor limit.  Direction
        signs are handled before the lower and upper tendon bounds are sorted.
        """

        margin = float(motor_limit_margin_deg)
        if not math.isfinite(margin) or margin < 0.0:
            raise ActuatorMappingError(
                "motor_limit_margin_deg must be finite and nonnegative"
            )

        lower = np.zeros(6, dtype=float)
        upper = np.zeros(6, dtype=float)
        for motor_id in self.motor_ids:
            motor_low, motor_high = self.position_limits_deg[motor_id]
            tightened_low = motor_low + margin
            tightened_high = motor_high - margin
            if tightened_low >= tightened_high:
                raise ActuatorMappingError(
                    f"motor {motor_id}: margin removes the position range"
                )

            scale = (
                self.motor_sign[motor_id]
                * self.spool_radius_mm[motor_id]
                * math.pi
                / 180.0
            )
            first = scale * tightened_low
            second = scale * tightened_high
            tendon_id = self.motor_to_tendon[motor_id]
            lower[tendon_id - 1] = min(first, second)
            upper[tendon_id - 1] = max(first, second)
        return lower, upper

    def project_tendon_velocity_zero_sum(
        self,
        tendon_velocity_mm_s: Sequence[float],
    ) -> np.ndarray:
        """Orthogonally remove each section's modeled common-mode rate."""

        tendon = self._vector(
            tendon_velocity_mm_s,
            6,
            "tendon_velocity_mm_s",
        ).copy()
        for section_name in self.chain_order:
            section = self.sections[section_name]
            model_rates = np.array(
                [
                    tendon[tendon_id - 1] * sign
                    for tendon_id, sign in zip(
                        section.tendon_ids,
                        section.dl_signs,
                    )
                ],
                dtype=float,
            )
            model_rates -= float(np.mean(model_rates))
            for tendon_id, sign, model_rate in zip(
                section.tendon_ids,
                section.dl_signs,
                model_rates,
            ):
                tendon[tendon_id - 1] = model_rate / sign
        return tendon

    def modeled_section_sums(
        self,
        tendon_values: Sequence[float],
    ) -> dict[str, float]:
        """Return diagnostic three-tendon sums after PCC sign mapping."""

        sections = self.pcc_sections_from_tendon_mm(tendon_values)
        return {
            section_name: float(np.sum(sections[section_name]))
            for section_name in self.chain_order
        }
