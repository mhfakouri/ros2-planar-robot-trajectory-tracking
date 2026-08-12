# ROS 2 Demonstration-Based Trajectory Tracking for a 2-DOF Planar Robot

A compact ROS 2 portfolio project demonstrating trajectory replay, feedback control, simulated robot dynamics, disturbance injection, rosbag logging, and tracking-error analysis for a 2-DOF planar robot.

This project is intentionally simple, but it is structured like a real robotics software pipeline. It is suitable for showing practical familiarity with ROS 2 nodes, topics, launch files, and data logging in a research application related to demonstration-initialized robotic control.

## Motivation

Many learning-based robotic manipulation methods use demonstration data to initialize or guide the robot's behavior. This repository implements a minimal version of that idea: a demonstrated joint-space trajectory is loaded from a CSV file and replayed as the desired motion for a simulated 2-DOF planar robot.

The robot is controlled using a PD controller. The simulated dynamics include bounded stochastic and sinusoidal disturbances, so the tracking-error plots show how the controller performs under imperfect conditions.

## What This Project Shows

- ROS 2 Python nodes using `rclpy`
- Topic-based communication between independent nodes
- Demonstration trajectory publishing
- Feedback controller node
- Simulated 2-DOF planar robot dynamics
- Disturbance injection
- rosbag recording workflow
- CSV logging and tracking-error plotting
- Connection to demonstration-initialized control research

## ROS Graph

```text
/demo_trajectory_publisher
        |
        |  /desired_joint_states  [sensor_msgs/JointState]
        v
/pd_controller  <---------------------- /actual_joint_states
        |
        |  /control_torque  [std_msgs/Float64MultiArray]
        v
/planar_robot_dynamics
        |
        |  /actual_joint_states  [sensor_msgs/JointState]
        v
/tracking_error_logger
        |
        v
tracking_results.csv
```

## Repository Structure

```text
ros2_demo_based_tracking/
├── README.md
├── LICENSE
├── package.xml
├── setup.py
├── setup.cfg
├── requirements.txt
├── config/
│   └── demo_trajectory.csv
├── launch/
│   └── demo_tracking.launch.py
├── ros2_demo_based_tracking/
│   ├── __init__.py
│   ├── demo_trajectory_publisher.py
│   ├── pd_controller.py
│   ├── planar_robot_dynamics.py
│   └── tracking_error_logger.py
└── scripts/
    ├── plot_demo_trajectory.py
    ├── plot_tracking_results.py
    └── record_rosbag.sh
```

## Nodes

### 1. `demo_trajectory_publisher`

Loads a demonstration trajectory from `config/demo_trajectory.csv` and publishes it as `/desired_joint_states`.

The CSV file contains:

```text
time, q1, q2, q1_dot, q2_dot
```

### 2. `pd_controller`

Subscribes to:

- `/desired_joint_states`
- `/actual_joint_states`

Publishes:

- `/control_torque`

The controller uses joint-position and joint-velocity feedback:

```text
tau = Kp * (q_des - q) + Kd * (qdot_des - qdot)
```

### 3. `planar_robot_dynamics`

Simulates a 2-link horizontal planar robot using a simplified rigid-body dynamics model:

```text
M(q) qddot + C(q, qdot) + B qdot = tau + disturbance
```

The node publishes the simulated robot state on `/actual_joint_states`.

### 4. `tracking_error_logger`

Subscribes to desired and actual joint states, computes tracking error, and saves the result to a CSV file.

## Installation

Tested conceptually for ROS 2 Humble/Jazzy-style Python packages.

Create a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone this repository:

```bash
git clone <your-repository-url>
cd ~/ros2_ws
```

Install Python plotting dependencies:

```bash
pip install -r src/ros2_demo_based_tracking/requirements.txt
```

Build the workspace:

```bash
colcon build
source install/setup.bash
```

## Run the Project

Launch the complete pipeline:

```bash
ros2 launch ros2_demo_based_tracking demo_tracking.launch.py
```

The logger writes results to:

```text
/tmp/ros2_demo_tracking_log.csv
```

You can also choose a custom output file:

```bash
ros2 launch ros2_demo_based_tracking demo_tracking.launch.py output_csv:=/tmp/my_tracking_log.csv
```

Run without disturbance:

```bash
ros2 launch ros2_demo_based_tracking demo_tracking.launch.py disturbance_enabled:=false
```

## Record a Rosbag

In a second terminal, after sourcing the workspace:

```bash
source ~/ros2_ws/install/setup.bash
ros2 bag record /desired_joint_states /actual_joint_states /control_torque -o rosbag2_demo_tracking
```

Or use the helper script:

```bash
bash src/ros2_demo_based_tracking/scripts/record_rosbag.sh
```

## Plot Results

After running the ROS 2 launch file for a few seconds, stop it with `Ctrl+C`, then run:

```bash
cd ~/ros2_ws/src/ros2_demo_based_tracking
python3 scripts/plot_tracking_results.py --csv /tmp/ros2_demo_tracking_log.csv
```

The script saves:

```text
results/desired_vs_actual.png
results/tracking_error.png
```

To plot only the included demonstration trajectory:

```bash
python3 scripts/plot_demo_trajectory.py
```

## Expected Output

The project should generate plots showing:

- desired vs. actual joint trajectories
- joint-level tracking errors
- Euclidean tracking-error norm

With disturbance enabled, the error should remain bounded but visibly larger than the no-disturbance case. This is useful for discussing robustness and the motivation for residual learning or demonstration-initialized reinforcement learning.

## Relevance to Demonstration-Initialized Control

This project is not a full reinforcement learning implementation. Instead, it demonstrates the software and control foundation needed before adding learning-based methods:

1. represent a demonstrated trajectory,
2. publish it through a ROS 2 interface,
3. track it with a controller,
4. simulate robot dynamics,
5. inject disturbances,
6. log data for analysis,
7. visualize tracking performance.

A natural extension would be to add a residual reinforcement learning policy that modifies the PD torque command:

```text
tau_total = tau_PD + tau_RL_residual
```

That extension would connect this simple project directly to residual reinforcement learning for robotic control.

## Possible Extensions

- Add a residual DDPG or TD3 controller
- Replace the simple dynamics with Gazebo, MuJoCo, or PyBullet
- Add Cartesian end-effector trajectory visualization
- Use real demonstration data from joystick or mouse input
- Add parameter uncertainty experiments
- Compare controller performance with and without disturbances
- Publish end-effector position using `geometry_msgs`


## Author

Mohammad Hossein  
MSc Mechanical Engineering, Robotics and Control  
Research interests: robot control, reinforcement learning, simulation, and rehabilitation robotics.
