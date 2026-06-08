#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR=${1:-rosbag2_demo_tracking}

ros2 bag record \
  /desired_joint_states \
  /actual_joint_states \
  /control_torque \
  -o "$OUTPUT_DIR"
