# ROS 2 Planar Robot Trajectory Tracking

A compact ROS 2 Python project demonstrating closed-loop trajectory tracking for a simulated 2-DOF planar robot.

The repository combines reference-trajectory replay, joint-space PD control, nonlinear robot dynamics, disturbance injection, CSV logging, rosbag support, and reproducible tracking analysis. It is intentionally small and transparent so the complete ROS 2 control pipeline can be inspected end to end.

> **Scope:** software and simulation only. The trajectory is replayed as a reference; this project does not implement imitation learning, reinforcement learning, or physical-robot validation.

## Highlights

- ROS 2 Python nodes built with `rclpy`
- Topic-based closed-loop control
- CSV reference-trajectory replay
- Saturated joint-space PD control
- Nonlinear 2-link horizontal planar-robot dynamics
- Stochastic + sinusoidal disturbance injection
- CSV logging and reproducible quantitative analysis
- Disturbed and disturbance-free launch configurations
- Archived original and verified tuning results

## Reference Trajectory

The included reference is stored in [`config/demo_trajectory.csv`](config/demo_trajectory.csv) with columns:

```text
time,q1,q2,q1_dot,q2_dot
```

It contains 2,000 points and is replayed at 100 Hz by default.

![Reference trajectory](results/reference_trajectory/demo_trajectory.png)

Output: [`results/reference_trajectory/demo_trajectory.png`](results/reference_trajectory/demo_trajectory.png)

## Verified Tracking Results

The verified run was recorded with disturbance injection enabled using the default controller and disturbance settings below. Metrics are computed from the complete archived CSV, including the initial convergence transient.

| Metric | Measured value |
|---|---:|
| Joint 1 RMSE | `0.03397 rad` |
| Joint 2 RMSE | `0.06120 rad` |
| Mean joint-error norm | `0.01873 rad` |
| Logged samples | `3,554` |
| Logged time span | `~71.08 s` |

### Desired vs. actual joint trajectories

![Verified desired vs. actual joint trajectories](results/verified_tuning/desired_vs_actual.png)

### Tracking error

![Verified joint tracking error](results/verified_tuning/tracking_error.png)

Verified outputs:

- [`tracking_results.csv`](results/verified_tuning/tracking_results.csv)
- [`desired_vs_actual.png`](results/verified_tuning/desired_vs_actual.png)
- [`tracking_error.png`](results/verified_tuning/tracking_error.png)

Reproduce the verified plots and metrics without overwriting the archive:

```bash
python3 scripts/plot_tracking_results.py \
  --csv results/verified_tuning/tracking_results.csv \
  --output-dir results/reproduced_verified_tuning
```

## System Architecture

```text
                    /desired_joint_states
/demo_trajectory_publisher --------------------+
                                               |
                                               v
                                      /pd_controller
                                               |
                                               | /control_torque
                                               v
                                  /planar_robot_dynamics
                                               |
                                               | /actual_joint_states
                                               +-------------------+
                                               |                   |
                                               +----> controller    |
                                                                   v
                                                    /tracking_error_logger
                                                                   |
                                                                   v
                                                        tracking CSV file
```

| Topic | Message type | Purpose |
|---|---|---|
| `/desired_joint_states` | `sensor_msgs/JointState` | Reference joint positions and velocities |
| `/actual_joint_states` | `sensor_msgs/JointState` | Simulated robot state |
| `/control_torque` | `std_msgs/Float64MultiArray` | Two-joint torque command |

## Controller and Simulation

The `demo_trajectory_publisher` replays `config/demo_trajectory.csv`. The `pd_controller` computes a saturated joint-space PD command:

```text
tau_i = Kp_i * (q_des_i - q_i) + Kd_i * (qdot_des_i - qdot_i)
```

The simulated horizontal 2-link robot follows:

```text
M(q) q_ddot + C(q, q_dot) + B q_dot = tau + d
```

where `d` is the injected disturbance. The dynamics are integrated with a semi-implicit Euler step.

| Parameter | Value |
|---|---:|
| Trajectory publish rate | `100 Hz` |
| Simulation rate | `100 Hz` |
| Controller rate | `100 Hz` |
| Logger rate | `50 Hz` |
| `Kp` | `[30.0, 12.0]` |
| `Kd` | `[5.0, 1.5]` |
| Torque limit | `20.0 N.m` |
| Disturbance enabled | `true` |
| Disturbance std. dev. | `0.15` |
| Sinusoidal disturbance amplitude | `0.25` |
| Random seed | `7` |

The logger records:

```text
time,desired_q1,desired_q2,actual_q1,actual_q2,error_q1,error_q2,error_norm
```

## Build and Run

Requirements: ROS 2 Jazzy on Linux with Python 3. ROS dependencies include `rclpy`, `sensor_msgs`, and `std_msgs`; plotting uses NumPy, pandas, and Matplotlib.

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mhfakouri/ros2-planar-robot-trajectory-tracking.git

python3 -m pip install -r ros2-planar-robot-trajectory-tracking/requirements.txt

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch ros2_demo_based_tracking demo_tracking.launch.py
```

Run without disturbance:

```bash
ros2 launch ros2_demo_based_tracking demo_tracking.launch.py \
  disturbance_enabled:=false
```

For a persistent new log instead of `/tmp`, use:

```bash
ros2 launch ros2_demo_based_tracking demo_tracking.launch.py \
  output_csv:=$HOME/ros2_ws/src/ros2-planar-robot-trajectory-tracking/results/latest_run/tracking_results.csv
```

Then plot it with:

```bash
cd ~/ros2_ws/src/ros2-planar-robot-trajectory-tracking
python3 scripts/plot_tracking_results.py \
  --csv results/latest_run/tracking_results.csv \
  --output-dir results/latest_run
```

## Original Tuning Archive

The repository also preserves an earlier controller-tuning run as development history. It is intentionally kept separate from the verified result above.

### Original desired vs. actual trajectories

![Original tuning desired vs. actual trajectories](results/original_tuning/desired_vs_actual.png)

### Original tracking error

![Original tuning tracking error](results/original_tuning/tracking_error.png)

Original-tuning outputs:

- [`tracking_results.csv`](results/original_tuning/tracking_results.csv)
- [`desired_vs_actual.png`](results/original_tuning/desired_vs_actual.png)
- [`tracking_error.png`](results/original_tuning/tracking_error.png)

## Results Layout

```text
results/
├── original_tuning/
│   ├── desired_vs_actual.png
│   ├── tracking_error.png
│   └── tracking_results.csv
├── reference_trajectory/
│   └── demo_trajectory.png
└── verified_tuning/
    ├── desired_vs_actual.png
    ├── tracking_error.png
    └── tracking_results.csv
```

## Repository Structure

```text
ros2-planar-robot-trajectory-tracking/
├── config/
│   └── demo_trajectory.csv
├── launch/
│   └── demo_tracking.launch.py
├── results/
├── ros2_demo_based_tracking/
│   ├── demo_trajectory_publisher.py
│   ├── pd_controller.py
│   ├── planar_robot_dynamics.py
│   └── tracking_error_logger.py
├── scripts/
│   ├── plot_demo_trajectory.py
│   ├── plot_tracking_results.py
│   └── record_rosbag.sh
├── package.xml
├── requirements.txt
├── setup.cfg
└── setup.py
```

## Rosbag Recording

With the system running:

```bash
bash scripts/record_rosbag.sh
```

This records `/desired_joint_states`, `/actual_joint_states`, and `/control_torque`.

## Scope and Limitations

- Simulation only; no physical-robot validation.
- Conventional PD control, not reinforcement learning.
- The demonstration trajectory is a reference, not imitation learning.
- The dynamics are intentionally compact and represent a horizontal 2-link manipulator.
- Reported metrics describe the archived simulated run under the stated configuration, not hardware performance or optimal control.

## Possible Extensions

- Compare disturbed and disturbance-free runs quantitatively
- Log and visualize control torque
- Add model-parameter uncertainty experiments
- Add Cartesian end-effector visualization
- Replace the compact plant with Gazebo, MuJoCo, or another higher-fidelity simulator
- Add residual learning while retaining PD control as the nominal baseline
- Validate the same ROS 2 interfaces on a physical robot

## Author

**Mohammad Hossein Fakouri**  
M.Sc. in Mechanical Engineering - Applied Design  
Interests: robotic control, learning-based control, robot simulation, and autonomous systems

## License

This project is released under the [MIT License](LICENSE).
