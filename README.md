# ROS2 Continuum Robot NMPC Control Framework


## Overview

A ROS2-based control framework for a two-section tendon-driven continuum robot.

Features:

- PCC kinematic modeling
- NMPC trajectory tracking control
- Actuator mapping
- IMU and vision based state estimation
- MuJoCo visualization


## Software

- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.12
- MuJoCo


## Architecture


robot_bringup
    |
    |
continuum_pccmodel
    |
continuum_mpc
    |
continuum_state_estimator
    |
continuum_mujoco_sim


## Current Status

Completed:

- PCC forward kinematics
- NMPC solver
- MuJoCo validation
- State estimation framework


Next:

- Real robot closed-loop control
- MPC+RL hybrid compensation
