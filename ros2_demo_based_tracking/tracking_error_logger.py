"""Log desired-vs-actual tracking error to CSV."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TrackingErrorLogger(Node):
    """Subscribe to desired and actual joint states and save tracking error."""

    def __init__(self) -> None:
        super().__init__('tracking_error_logger')

        self.declare_parameter('log_rate_hz', 50.0)
        self.declare_parameter('output_csv', '~/ros2_demo_tracking_log.csv')

        log_rate_hz = float(self.get_parameter('log_rate_hz').value)
        output_csv = Path(str(self.get_parameter('output_csv').value)).expanduser()
        output_csv.parent.mkdir(parents=True, exist_ok=True)

        self.desired_state: Optional[JointState] = None
        self.actual_state: Optional[JointState] = None
        self.start_time = self.get_clock().now()

        self.csv_file = output_csv.open('w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                'time',
                'desired_q1',
                'desired_q2',
                'actual_q1',
                'actual_q2',
                'error_q1',
                'error_q2',
                'error_norm',
            ]
        )

        self.create_subscription(JointState, '/desired_joint_states', self._desired_callback, 10)
        self.create_subscription(JointState, '/actual_joint_states', self._actual_callback, 10)

        period = 1.0 / max(log_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._log_error)
        self.get_logger().info(f'Tracking-error logger writing to: {output_csv}')

    def _desired_callback(self, msg: JointState) -> None:
        self.desired_state = msg

    def _actual_callback(self, msg: JointState) -> None:
        self.actual_state = msg

    def _log_error(self) -> None:
        if self.desired_state is None or self.actual_state is None:
            return
        if len(self.desired_state.position) < 2 or len(self.actual_state.position) < 2:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        desired_q1, desired_q2 = self.desired_state.position[:2]
        actual_q1, actual_q2 = self.actual_state.position[:2]
        error_q1 = desired_q1 - actual_q1
        error_q2 = desired_q2 - actual_q2
        error_norm = (error_q1**2 + error_q2**2) ** 0.5

        self.writer.writerow(
            [
                f'{elapsed:.6f}',
                f'{desired_q1:.8f}',
                f'{desired_q2:.8f}',
                f'{actual_q1:.8f}',
                f'{actual_q2:.8f}',
                f'{error_q1:.8f}',
                f'{error_q2:.8f}',
                f'{error_norm:.8f}',
            ]
        )
        self.csv_file.flush()

    def destroy_node(self) -> bool:
        try:
            self.csv_file.flush()
            self.csv_file.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackingErrorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
