import pytest

from motor_can_driver.motor_protocol import (
    build_feedback_request,
    build_position_cmd,
    build_stop_cmd,
    parse_feedback,
)


def test_relative_position_frame_positive_and_negative():
    assert build_position_cmd(12.3, 0.4, 32) == bytes([
        0x02, 0x00, 32, 0x00, 123, 0x00, 4,
    ])
    assert build_position_cmd(-12.3, 0.4, 32) == bytes([
        0x02, 0x01, 32, 0x00, 123, 0x00, 4,
    ])


def test_feedback_signed_position_decoding():
    state = parse_feedback(bytes([
        2, 1, 0, 10, 0xFF, 0xFF, 0xFF, 0xF6,
    ]))
    assert state.motor_id == 2
    assert state.reached is True
    assert state.speed_rad_s == 1.0
    assert state.raw_deg == -1.0


def test_frame_lengths_and_out_of_range_rejection():
    assert len(build_feedback_request()) == 7
    assert len(build_stop_cmd(32)) == 7
    with pytest.raises(ValueError):
        build_position_cmd(7000.0, 0.3, 32)
    with pytest.raises(ValueError):
        parse_feedback(b"short")
