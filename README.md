# ROS 2 Demonstration-Based Trajectory Tracking for a 2-DOF Planar Robot

A compact ROS 2 Python package demonstrating trajectory replay, feedback control, simulated robot dynamics, disturbance injection, rosbag logging, and tracking-error analysis for a 2-DOF planar robot.

This repository is structured as a ROS 2 Python package following standard node-based design (publisher, controller, dynamics simulator, and logger).

## System Overview

```text
/demo_trajectory_publisher → /desired_joint_states
/pd_controller → /control_torque
/planar_robot_dynamics → /actual_joint_states
/tracking_error_logger → tracking_results.csv
```

## Key Features

- ROS 2 Python nodes (rclpy)
- Topic-based communication
- Demonstration trajectory replay from CSV
- PD control for joint tracking
- Simulated rigid-body planar dynamics
- Disturbance injection (bounded stochastic + sinusoidal)
- rosbag recording support
- Tracking-error logging and visualization

## Dynamics Model

The system simulates a simplified 2-DOF planar robot:

```text
M(q) qdd + C(q, qdot) + B qdot = tau + disturbance
```

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/mhfakouri/ROS-simple-project.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch ros2_demo_based_tracking demo_tracking.launch.py
```

## Data Output

- CSV logging of tracking results
- Optional rosbag recording
- Post-processing plots via provided scripts

## Verification Status

- Designed for ROS 2 Humble/Jazzy-style Python package structure
- Users should verify execution on their local ROS 2 setup
- No official OS-specific benchmark is claimed in this repository

## Author

Mohammad Hossein Fakouri
M.Sc. Mechanical Engineering – Applied Design
Research: robotics, control, reinforcement learning, simulation
