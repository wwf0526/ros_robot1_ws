#!/usr/bin/env python3

import numpy as np
import yaml
import time
import csv

from pathlib import Path

from continuum_mpc.pcc_control_model import PccControlModel
from continuum_mpc.pcc_nmpc_solver import (
    PccNmpcSolver,
    PccNmpcReference,
    PccNmpcConfig,
)


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def save_csv(path, data, header):

    with open(
        path,
        "w",
        newline=""
    ) as f:

        writer = csv.writer(f)

        writer.writerow(header)

        for row in data:
            writer.writerow(row)


def main():

    # ==============================
    # 1. Load robot calibration
    # ==============================

    ws = Path.home() / "ros_robot1_ws"

    cfg_path = (
        ws
        / "src"
        / "robot_bringup"
        / "config"
        / "robot_calibration.yaml"
    )

    cfg = load_yaml(cfg_path)


    # ==============================
    # 2. Create PCC model
    # ==============================

    # NMPC阶段降低采样数量，提高求解速度
    pcc_model = PccControlModel.from_config(
        cfg,
        samples_per_section=2,
    )


    # ==============================
    # 3. Create NMPC solver
    # ==============================

    nmpc_cfg = PccNmpcConfig()

    solver = PccNmpcSolver(
        pcc_model,
        config=nmpc_cfg,
    )


    # ==============================
    # 4. Target position
    # ==============================

    # 单位:m
    # 当前直杆:
    # (0,0,340mm)
    #
    # 测试目标:
    # (30,0,335mm)

    target_position = np.array(
        [
            0.030,
            0.000,
            0.335,
        ],
        dtype=float,
    )


    reference = PccNmpcReference(
        tip_position_m=target_position
    )


    # ==============================
    # 5. Initial tendon state
    # ==============================

    current_tendon = np.zeros(
        6,
        dtype=float
    )


    dt = nmpc_cfg.control_period_sec


    previous_u = None


    # ==============================
    # logging buffers
    # ==============================

    error_history = []
    trajectory_history = []
    tendon_history = []
    control_history = []
    solve_time_history = []


    # ==============================
    # 6. Closed loop
    # ==============================

    max_steps = 200


    for step in range(max_steps):

        result = solver.solve(
            current_tendon,
            reference,
            previous_independent_velocity_mm_s=previous_u,
        )


        if not result.valid:

            print(
                f"[{step}] NMPC failed:",
                result.status
            )

            break


        # NMPC output
        tendon_velocity = (
            result.first_tendon_velocity_mm_s
        )


        # tendon state update
        current_tendon += (
            tendon_velocity * dt
        )


        # PCC forward kinematics
        output = (
            pcc_model.evaluate_from_tendon_mm(
                current_tendon
            )
        )


        tip_position = (
            output.tip_transform[:3, 3]
        )


        # position error
        error = np.linalg.norm(
            target_position - tip_position
        )


        # ==============================
        # record data
        # ==============================

        error_history.append(
            error
        )

        trajectory_history.append(
            tip_position.copy()
        )

        tendon_history.append(
            current_tendon.copy()
        )

        control_history.append(
            result.first_independent_velocity_mm_s.copy()
        )

        solve_time_history.append(
            result.solve_time_ms
        )


        print(
            f"step={step:03d} "
            f"error={error*1000:.3f} mm "
            f"solve={result.solve_time_ms:.2f} ms"
        )


        # convergence condition
        if error < 0.002:

            print(
                "NMPC converged"
            )

            break


        previous_u = (
            result.first_independent_velocity_mm_s
        )


        time.sleep(0.01)


    # ==============================
    # final result
    # ==============================

    print("\nFinished")

    print("Final tendon:")
    print(current_tendon)


    print("Final tip:")
    print(tip_position)


    # ==============================
    # save csv
    # ==============================

    result_dir = (
        Path(__file__).parent.parent
        / "results"
    )

    result_dir.mkdir(
        exist_ok=True
    )


    save_csv(
        result_dir / "tip_error.csv",
        [[e] for e in error_history],
        [
            "error_m"
        ]
    )


    save_csv(
        result_dir / "tip_trajectory.csv",
        trajectory_history,
        [
            "x_m",
            "y_m",
            "z_m",
        ]
    )


    save_csv(
        result_dir / "tendon_history.csv",
        tendon_history,
        [
            "l1",
            "l2",
            "l3",
            "l4",
            "l5",
            "l6",
        ]
    )


    save_csv(
        result_dir / "control_history.csv",
        control_history,
        [
            "u1",
            "u2",
            "u3",
            "u4",
        ]
    )


    save_csv(
        result_dir / "solver_time.csv",
        [[t] for t in solve_time_history],
        [
            "solve_time_ms"
        ]
    )


if __name__ == "__main__":

    main()
