#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def main():

    result_dir = (
        Path(__file__).parent.parent
        / "results"
    )


    # ======================
    # Load data
    # ======================

    error = np.loadtxt(
        result_dir / "tip_error.csv",
        delimiter=",",
        skiprows=1
    )


    trajectory = np.loadtxt(
        result_dir / "tip_trajectory.csv",
        delimiter=",",
        skiprows=1
    )


    control = np.loadtxt(
        result_dir / "control_history.csv",
        delimiter=",",
        skiprows=1
    )


    solver_time = np.loadtxt(
        result_dir / "solver_time.csv",
        delimiter=",",
        skiprows=1
    )


    plot_dir = result_dir / "plots"
    plot_dir.mkdir(
        exist_ok=True
    )


    # ======================
    # 1 error curve
    # ======================

    plt.figure()

    plt.plot(
        error * 1000
    )

    plt.xlabel(
        "Step"
    )

    plt.ylabel(
        "Position Error (mm)"
    )

    plt.title(
        "PCC-NMPC Convergence"
    )

    plt.grid()

    plt.savefig(
        plot_dir /
        "error_curve.png",
        dpi=300
    )


    # ======================
    # 2 trajectory
    # ======================

    fig = plt.figure()

    ax = fig.add_subplot(
        111,
        projection="3d"
    )

    ax.plot(
        trajectory[:,0],
        trajectory[:,1],
        trajectory[:,2],
        marker="."
    )


    ax.scatter(
        trajectory[0,0],
        trajectory[0,1],
        trajectory[0,2],
        label="start"
    )


    ax.scatter(
        trajectory[-1,0],
        trajectory[-1,1],
        trajectory[-1,2],
        label="end"
    )


    ax.set_xlabel("X(m)")
    ax.set_ylabel("Y(m)")
    ax.set_zlabel("Z(m)")


    ax.legend()

    plt.title(
        "Tip Trajectory"
    )


    plt.savefig(
        plot_dir /
        "tip_trajectory.png",
        dpi=300
    )


    # ======================
    # 3 control input
    # ======================

    plt.figure()


    for i in range(4):

        plt.plot(
            control[:,i],
            label=f"u{i+1}"
        )


    plt.xlabel(
        "Step"
    )

    plt.ylabel(
        "Velocity(mm/s)"
    )


    plt.title(
        "NMPC Control Input"
    )


    plt.legend()

    plt.grid()


    plt.savefig(
        plot_dir /
        "control_input.png",
        dpi=300
    )


    # ======================
    # 4 solver time
    # ======================

    plt.figure()


    plt.plot(
        solver_time
    )


    plt.xlabel(
        "Step"
    )

    plt.ylabel(
        "Solve Time(ms)"
    )


    plt.title(
        "NMPC Solver Time"
    )


    plt.grid()


    plt.savefig(
        plot_dir /
        "solver_time.png",
        dpi=300
    )


    print(
        "Plots saved:"
    )

    print(plot_dir)


if __name__ == "__main__":
    main()
