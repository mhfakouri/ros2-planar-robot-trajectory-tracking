# ROS 2 Planar Robot Trajectory Tracking

A compact ROS 2 project for demonstration-trajectory replay, joint-space feedback control, simulated 2-DOF robot dynamics, disturbance injection, CSV logging, and quantitative tracking-error analysis.

The project is intentionally small and transparent. Its purpose is to demonstrate a complete ROS 2 control workflow rather than a high-fidelity robot or reinforcement-learning system.

## Highlights

- ROS 2 Python nodes built with `rclpy`
- Topic-based closed-loop control architecture
- Demonstration trajectory replay from CSV
- Joint-space PD control with torque saturation
- 2-link horizontal planar robot dynamics
- Stochastic and sinusoidal disturbance injection
- Desired/actual state logging to CSV
- Reproducible tracking-error plots and metrics
- Launch-file configuration for disturbed and disturbance-free runs

## Verified Tracking Results

The current controller tuning was evaluated with disturbance injection enabled. The recorded run includes the initial convergence transient and the subsequent repeated tracking motion.

| Metric | Measured value |
|---|---:|
| Joint 1 RMSE | `0.03397 rad` |
| Joint 2 RMSE | `0.06120 rad` |
| Mean joint-error norm | `0.01873 rad` |

The run used the public default controller and disturbance settings documented below. The logger recorded 3,554 samples over approximately 71.08 s.

The plotting script computes these values directly from the logged CSV:

```bash
python3 scripts/plot_tracking_results.py \
  --csv /tmp/ros2_demo_tracking_log.csv
```

Generated plots:

```text
results/demo_trajectory.png
results/desired_vs_actual.png
results/tracking_error.png
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

### ROS 2 interfaces

| Topic | Message type | Purpose |
|---|---|---|
| `/desired_joint_states` | `sensor_msgs/JointState` | Demonstration/reference trajectory |
| `/actual_joint_states` | `sensor_msgs/JointState` | Simulated robot state |
| `/control_torque` | `std_msgs/Float64MultiArray` | Two-joint torque command |

## Nodes

### `demo_trajectory_publisher`

Loads `config/demo_trajectory.csv` and publishes the desired joint positions and velocities.

The trajectory file contains:

```text
time,q1,q2,q1_dot,q2_dot
```

The included reference contains 2,000 points and is replayed at 100 Hz by default.

### `pd_controller`

Subscribes to the desired and actual joint states and computes a saturated joint-space PD command:

```text
tau_i = Kp_i * (q_des_i - q_i) + Kd_i * (qdot_des_i - qdot_i)
```

Current tested gains:

```text
Kp = [30.0, 12.0]
Kd = [5.0, 1.5]
torque limit = +/-20.0 N.m
```

### `planar_robot_dynamics`

Simulates a lightweight 2-link robot moving in the horizontal plane:

```text
M(q) q_ddot + C(q, q_dot) + B q_dot = tau + d
```

where `d` is the injected disturbance. The dynamics are integrated with a semi-implicit Euler step.

Default disturbance configuration:

```text
stochastic disturbance standard deviation = 0.15
sinusoidal disturbance amplitude          = 0.25
random seed                               = 7
```

### `tracking_error_logger`

Subscribes to desired and actual joint states and writes:

```text
time,desired_q1,desired_q2,actual_q1,actual_q2,error_q1,error_q2,error_norm
```

The default output path is:

```text
/tmp/ros2_demo_tracking_log.csv
```

## Default Experiment Configuration

| Parameter | Value |
|---|---:|
| Trajectory publish rate | 100 Hz |
| Simulation rate | 100 Hz |
| Controller rate | 100 Hz |
| Logger rate | 50 Hz |
| `Kp` | `[30.0, 12.0]` |
| `Kd` | `[5.0, 1.5]` |
| Torque limit | `20.0 N.m` |
| Disturbance enabled | `true` |
| Disturbance std. dev. | `0.15` |
| Sinusoidal disturbance amplitude | `0.25` |

## Repository Structure

```text
ros2-planar-robot-trajectory-tracking/
├── config/
│   └── demo_trajectory.csv
├── launch/
│   └── demo_tracking.launch.py
├── resource/
├── ros2_demo_based_tracking/
│   ├── __init__.py
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

## Requirements

The verified workflow used ROS 2 Jazzy on Linux with Python 3.

ROS dependencies declared by the package include:

- `rclpy`
- `sensor_msgs`
- `std_msgs`

The plotting utilities use NumPy, pandas, and Matplotlib through `requirements.txt`.

## Build

Create a clean ROS 2 workspace and clone the repository:

```bash
source /opt/ros/jazzy/setup.bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mhfakouri/ros2-planar-robot-trajectory-tracking.git
```

Install the plotting dependencies:

```bash
python3 -m pip install -r \
  ~/ros2_ws/src/ros2-planar-robot-trajectory-tracking/requirements.txt
```

Build and source the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Confirm that ROS 2 can find the package:

```bash
ros2 pkg prefix ros2_demo_based_tracking
```

## Run

Launch the full disturbed tracking pipeline:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ros2_demo_based_tracking demo_tracking.launch.py
```

Run without disturbance:

```bash
ros2 launch ros2_demo_based_tracking demo_tracking.launch.py \
  disturbance_enabled:=false
```

Choose a different log file:

```bash
ros2 launch ros2_demo_based_tracking demo_tracking.launch.py \
  output_csv:=/tmp/my_tracking_log.csv
```

## Inspect the ROS Graph

In another sourced terminal:

```bash
ros2 topic list
```

Expected project topics include:

```text
/actual_joint_states
/control_torque
/desired_joint_states
```

A single desired-state message can be inspected with:

```bash
ros2 topic echo /desired_joint_states --once
```

## Plot the Results

After collecting a run:

```bash
cd ~/ros2_ws/src/ros2-planar-robot-trajectory-tracking

python3 scripts/plot_tracking_results.py \
  --csv /tmp/ros2_demo_tracking_log.csv
```

The script generates:

```text
results/desired_vs_actual.png
results/tracking_error.png
```

It also prints the joint RMSE values and the mean error norm.

Plot the demonstration trajectory separately with:

```bash
python3 scripts/plot_demo_trajectory.py
```

## Record a Rosbag

With the tracking system running:

```bash
ros2 bag record \
  /desired_joint_states \
  /actual_joint_states \
  /control_torque \
  -o rosbag2_demo_tracking
```

or use:

```bash
bash scripts/record_rosbag.sh
```

## Scope and Limitations

This repository is a software and simulation portfolio project.

- The robot is simulated; there is no physical-robot validation.
- The controller is a conventional PD controller, not reinforcement learning.
- The demonstration trajectory is used as a reference trajectory; this is not demonstration learning or imitation learning.
- The dynamics model is intentionally compact and represents a horizontal 2-link manipulator.
- The reported metrics describe the recorded simulated run under the stated disturbance configuration; they should not be interpreted as hardware-performance results.

These limitations are intentional. The repository focuses on ROS 2 software structure, closed-loop communication, simulation, logging, and reproducible control analysis.

## Possible Extensions

Natural next steps include:

- compare disturbed and disturbance-free runs quantitatively
- log and visualize the control torque
- add model-parameter uncertainty experiments
- add Cartesian end-effector visualization
- replace the compact plant with Gazebo, MuJoCo, or another higher-fidelity simulator
- add a residual learning controller while retaining the PD controller as the nominal baseline
- validate the same ROS 2 interfaces on a physical robot

## Author

**Mohammad Hossein Fakouri**  
M.Sc. in Mechanical Engineering - Applied Design  
Interests: robotic control, learning-based control, robot simulation, and autonomous systems

## License

This project is released under the [MIT License](LICENSE).
