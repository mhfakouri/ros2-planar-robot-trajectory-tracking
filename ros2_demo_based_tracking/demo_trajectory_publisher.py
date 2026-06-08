"""Publish a demonstration trajectory as desired joint states.

The trajectory is loaded from a CSV file with columns:
time, q1, q2, q1_dot, q2_dot
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


TrajectoryRow = Tuple[float, float, float, float, float]


class DemoTrajectoryPublisher(Node):
    """Replay a demonstration trajectory for a 2-DOF robot."""

    def __init__(self) -> None:
        super().__init__('demo_trajectory_publisher')

        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('loop', True)

        trajectory_file = self.get_parameter('trajectory_file').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.loop = bool(self.get_parameter('loop').value)

        self.trajectory = self._load_trajectory(trajectory_file)
        self.index = 0

        self.publisher = self.create_publisher(JointState, '/desired_joint_states', 10)
        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._publish_next_point)

        self.get_logger().info(
            f'Loaded {len(self.trajectory)} demonstration points. '
            f'Publishing at {self.publish_rate_hz:.1f} Hz.'
        )

    def _load_trajectory(self, trajectory_file: str) -> List[TrajectoryRow]:
        if not trajectory_file:
            self.get_logger().warn('No trajectory_file parameter provided. Using built-in demo trajectory.')
            return self._default_trajectory()

        path = Path(trajectory_file).expanduser()
        if not path.exists():
            self.get_logger().warn(f'Trajectory file not found: {path}. Using built-in demo trajectory.')
            return self._default_trajectory()

        rows: List[TrajectoryRow] = []
        with path.open('r', newline='') as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                rows.append(
                    (
                        float(row['time']),
                        float(row['q1']),
                        float(row['q2']),
                        float(row['q1_dot']),
                        float(row['q2_dot']),
                    )
                )

        if not rows:
            raise ValueError(f'Trajectory file is empty: {path}')
        return rows

    @staticmethod
    def _default_trajectory() -> List[TrajectoryRow]:
        # Conservative smooth motion, useful when the CSV file is not available.
        import math

        dt = 0.01
        frequency = 0.15
        rows: List[TrajectoryRow] = []
        for k in range(1000):
            t = k * dt
            omega = 2.0 * math.pi * frequency
            q1 = 0.45 * math.sin(omega * t)
            q2 = 0.20 + 0.30 * math.cos(omega * t)
            q1_dot = 0.45 * omega * math.cos(omega * t)
            q2_dot = -0.30 * omega * math.sin(omega * t)
            rows.append((t, q1, q2, q1_dot, q2_dot))
        return rows

    def _publish_next_point(self) -> None:
        if self.index >= len(self.trajectory):
            if self.loop:
                self.index = 0
            else:
                self.get_logger().info('Finished trajectory replay.')
                self.timer.cancel()
                return

        _, q1, q2, q1_dot, q2_dot = self.trajectory[self.index]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2']
        msg.position = [q1, q2]
        msg.velocity = [q1_dot, q2_dot]
        msg.effort = []

        self.publisher.publish(msg)
        self.index += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
