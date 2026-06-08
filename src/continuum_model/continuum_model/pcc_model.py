import math


def compute_section_curvature(tendon_lengths, tendon_angles_deg, tendon_radius_mm):
    """
    根据一段连续体的多根线缆长度变化，估计该段的曲率分量 kx, ky。

    tendon_lengths:
        dict，例如 {1: 0.5, 3: -0.2, 5: 0.1}

    tendon_angles_deg:
        dict，例如 {1: 300.0, 3: 180.0, 5: 60.0}

    tendon_radius_mm:
        线缆到中心距离，单位 mm
    """

    if tendon_radius_mm <= 0:
        return 0.0, 0.0

    sum_x = 0.0
    sum_y = 0.0

    for tendon_id, length_mm in tendon_lengths.items():
        angle_deg = tendon_angles_deg[tendon_id]
        angle_rad = math.radians(angle_deg)

        sum_x += length_mm * math.cos(angle_rad)
        sum_y += length_mm * math.sin(angle_rad)

    kx = sum_x / tendon_radius_mm
    ky = sum_y / tendon_radius_mm

    return float(kx), float(ky)


def compute_bending_magnitude(kx, ky):
    """
    计算总弯曲强度。
    """
    return float(math.sqrt(kx * kx + ky * ky))


def compute_bending_direction(kx, ky):
    """
    计算弯曲方向 phi，单位 rad。
    """
    return float(math.atan2(ky, kx))
