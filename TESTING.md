# ROS 2 Planar Robot – Verification Checklist

This document defines a minimal checklist to verify that the ROS 2 trajectory tracking pipeline runs correctly.

## 1. Environment

- Ubuntu 22.04 or 24.04
- ROS 2 Humble or Jazzy
- Python 3.10+

## 2. Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/mhfakouri/ROS-simple-project.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 3. Run system

Launch full pipeline:

```bash
ros2 launch ros2_demo_based_tracking demo_tracking.launch.py
```

## 4. Expected behavior

- `/demo_trajectory_publisher` publishes reference trajectory
- `/pd_controller` generates control torques
- `/planar_robot_dynamics` simulates motion
- `/tracking_error_logger` writes CSV log

## 5. Outputs

- `tracking_results.csv`
- optional rosbag files if enabled
- plots generated via scripts in `/scripts`

## 6. Notes

- Disturbances are enabled by default
- This is a simulation-only system
- No hardware validation is included
