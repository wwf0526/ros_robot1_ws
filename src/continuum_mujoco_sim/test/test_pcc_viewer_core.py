import math

import pytest

from continuum_mujoco_sim.pcc_viewer_core import distribute_pcc_section


def test_unequal_segments_receive_length_weighted_motion():
    states = distribute_pcc_section(
        theta_rad=0.3,
        phi_rad=0.0,
        arc_length_m=0.06,
        reference_length_m=0.05,
        segment_lengths_m=[0.02, 0.03],
        bend_limit_rad=0.6,
        slide_limit_m=0.03,
    )
    assert states[0].bend_y_rad == pytest.approx(0.12)
    assert states[1].bend_y_rad == pytest.approx(0.18)
    assert states[0].slide_z_m == pytest.approx(0.004)
    assert states[1].slide_z_m == pytest.approx(0.006)


def test_direction_decomposition_and_limits():
    states = distribute_pcc_section(
        theta_rad=2.0,
        phi_rad=math.pi / 2.0,
        arc_length_m=0.2,
        reference_length_m=0.1,
        segment_lengths_m=[0.05, 0.05],
        bend_limit_rad=0.4,
        slide_limit_m=0.02,
    )
    assert all(state.bend_x_rad == pytest.approx(-0.4) for state in states)
    assert all(abs(state.bend_y_rad) < 1.0e-12 for state in states)
    assert all(state.slide_z_m == pytest.approx(0.02) for state in states)


def test_invalid_segment_lengths_are_rejected():
    with pytest.raises(ValueError, match="segment lengths"):
        distribute_pcc_section(
            theta_rad=0.1,
            phi_rad=0.0,
            arc_length_m=0.1,
            reference_length_m=0.1,
            segment_lengths_m=[0.05, 0.0],
            bend_limit_rad=0.6,
            slide_limit_m=0.03,
        )
