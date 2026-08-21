"""Pure PCC-to-virtual-joint mapping used by the MuJoCo viewer."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class VirtualJointState:
    bend_x_rad: float
    bend_y_rad: float
    slide_z_m: float
    raw_slide_z_m: float


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(value, limit))


def distribute_pcc_section(
    theta_rad: float,
    phi_rad: float,
    arc_length_m: float,
    reference_length_m: float,
    segment_lengths_m: Sequence[float],
    bend_limit_rad: float,
    slide_limit_m: float,
) -> list[VirtualJointState]:
    """Discretize one constant-curvature section by actual unit lengths."""

    scalar_values = (
        theta_rad,
        phi_rad,
        arc_length_m,
        reference_length_m,
        bend_limit_rad,
        slide_limit_m,
    )
    if not all(math.isfinite(float(value)) for value in scalar_values):
        raise ValueError("PCC section values must be finite")
    if bend_limit_rad <= 0.0 or slide_limit_m <= 0.0:
        raise ValueError("virtual joint limits must be positive")

    lengths = [float(value) for value in segment_lengths_m]
    if not lengths or any(
        not math.isfinite(value) or value <= 0.0 for value in lengths
    ):
        raise ValueError("segment lengths must be finite and positive")

    total_xml_length = sum(lengths)
    total_extension = float(arc_length_m) - float(reference_length_m)
    result = []
    for segment_length in lengths:
        weight = segment_length / total_xml_length
        unit_theta = float(theta_rad) * weight
        qx_raw = -unit_theta * math.sin(float(phi_rad))
        qy_raw = unit_theta * math.cos(float(phi_rad))
        qz_raw = total_extension * weight
        result.append(
            VirtualJointState(
                bend_x_rad=_clip(qx_raw, float(bend_limit_rad)),
                bend_y_rad=_clip(qy_raw, float(bend_limit_rad)),
                slide_z_m=_clip(qz_raw, float(slide_limit_m)),
                raw_slide_z_m=qz_raw,
            )
        )
    return result
