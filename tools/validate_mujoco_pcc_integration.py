#!/usr/bin/env python3
"""Validate the PCC-to-MuJoCo model and launch wiring without ROS/MuJoCo."""

from __future__ import annotations

import math
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml


import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/continuum_mujoco_sim"))

from continuum_mujoco_sim.pcc_viewer_core import distribute_pcc_section

CALIBRATION = ROOT / "src/robot_bringup/config/robot_calibration.yaml"
MODEL = (
    ROOT
    / "src/continuum_mujoco_sim/models/generated"
    / "continuum_kinematic_generated.xml"
)
STATE_MSG = ROOT / "src/robot_interfaces/msg/ContinuumState.msg"
LAUNCH = ROOT / "src/robot_bringup/launch/motor_pcc_mujoco.launch.py"


def expected_segment_lengths(cfg, section_name):
    section = cfg["sections"][section_name]
    thickness = float(section["disk_thickness_mm"])
    gaps = [float(value) for value in section["disk_gap_lengths_mm"]]
    return [(thickness + gap) / 1000.0 for gap in gaps]


def main() -> None:
    cfg = yaml.safe_load(CALIBRATION.read_text(encoding="utf-8"))
    root = ET.parse(MODEL).getroot()

    chain_order = list(cfg["pcc_model"]["chain_order"])
    if chain_order != ["section2", "section1"]:
        raise AssertionError(f"unexpected PCC chain order: {chain_order}")

    prefixes = {"section2": "sec2", "section1": "sec1"}
    for section_name in chain_order:
        prefix = prefixes[section_name]
        lengths = expected_segment_lengths(cfg, section_name)
        for index, expected_length in enumerate(lengths, start=1):
            body_name = f"{prefix}_u{index}"
            body = root.find(f".//body[@name='{body_name}']")
            if body is None:
                raise AssertionError(f"missing MuJoCo body: {body_name}")
            actual_length = float(body.attrib["pos"].split()[2])
            if not math.isclose(actual_length, expected_length, abs_tol=1e-9):
                raise AssertionError(
                    f"{body_name}: XML length {actual_length} != "
                    f"calibration {expected_length}"
                )
            for suffix in ("bend_x", "bend_y", "slide_z"):
                joint_name = f"{body_name}_{suffix}"
                if root.find(f".//joint[@name='{joint_name}']") is None:
                    raise AssertionError(f"missing MuJoCo joint: {joint_name}")

    unequal = distribute_pcc_section(
        theta_rad=0.3,
        phi_rad=0.0,
        arc_length_m=0.06,
        reference_length_m=0.05,
        segment_lengths_m=[0.02, 0.03],
        bend_limit_rad=0.6,
        slide_limit_m=0.03,
    )
    if not math.isclose(unequal[0].bend_y_rad, 0.12, abs_tol=1e-12):
        raise AssertionError("PCC bend is not weighted by segment length")
    if not math.isclose(unequal[1].bend_y_rad, 0.18, abs_tol=1e-12):
        raise AssertionError("PCC bend is not weighted by segment length")
    if not math.isclose(unequal[0].slide_z_m, 0.004, abs_tol=1e-12):
        raise AssertionError("PCC extension is not weighted by segment length")
    if not math.isclose(unequal[1].slide_z_m, 0.006, abs_tol=1e-12):
        raise AssertionError("PCC extension is not weighted by segment length")

    mesh = root.find("./asset/mesh[@name='hex_disk']")
    if mesh is None:
        raise AssertionError("hex_disk mesh declaration is missing")
    mesh_path = (MODEL.parent / mesh.attrib["file"]).resolve()
    if not mesh_path.is_file():
        raise AssertionError(f"mesh file does not exist: {mesh_path}")

    state_fields = STATE_MSG.read_text(encoding="utf-8")
    required_fields = []
    for section in ("section2", "section1"):
        required_fields.extend([
            f"{section}_model_theta_rad",
            f"{section}_model_phi_rad",
            f"{section}_model_arc_length_m",
            f"{section}_model_l0_m",
        ])
    missing_fields = [
        field for field in required_fields if field not in state_fields
    ]
    if missing_fields:
        raise AssertionError(f"ContinuumState fields missing: {missing_fields}")

    launch_text = LAUNCH.read_text(encoding="utf-8")
    for executable in (
        "mock_motor_hardware_node",
        "state_estimator_node",
        "mujoco_pcc_viewer",
    ):
        if executable == "mock_motor_hardware_node":
            # The mock executable lives in the included motor launch.
            motor_launch = (
                ROOT / "src/robot_bringup/launch/motor_control.launch.py"
            ).read_text(encoding="utf-8")
            if executable not in motor_launch:
                raise AssertionError(f"launch executable missing: {executable}")
        elif executable not in launch_text:
            raise AssertionError(f"launch executable missing: {executable}")

    print("PASS: calibration segment lengths match generated MuJoCo XML")
    print("PASS: 30 PCC virtual joints and mesh asset are present")
    print("PASS: unequal PCC units use length-weighted bend and extension")
    print("PASS: ContinuumState fields and integrated launch wiring are present")


if __name__ == "__main__":
    main()
