"""PD controller node for 2-DOF trajectory tracking."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PDController(Node):
    """Subscribe to desired and actual joint states, then publish torque commands."""

    def __init__(self) -> None:
        super().__init__('pd_controller')

        self.declare_parameter('kp', [35.0, 28.0])
        self.declare_parameter('kd', [7.0, 6.0])
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('torque_limit', 20.0)

        self.kp = [float(v) for v in self.get_parameter('kp').value]
        self.kd = [float(v) for v in self.get_parameter('kd').value]
        self.torque_limit = float(self.get_parameter('torque_limit').value)
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.desired_state: Optional[JointState] = None
        self.actual_state: Optional[JointState] = None

        self.create_subscription(JointState, '/desired_joint_states', self._desired_callback, 10)
        self.create_subscription(JointState, '/actual_joint_states', self._actual_callback, 10)
        self.publisher = self.create_publisher(Float64MultiArray, '/control_torque', 10)

        period = 1.0 / max(control_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._control_step)
        self.get_logger().info('PD controller started.')

    def _desired_callback(self, msg: JointState) -> None:
        self.desired_state = msg

    def _actual_callback(self, msg: JointState) -> None:
        self.actual_state = msg

    def _control_step(self) -> None:
        if self.desired_state is None or self.actual_state is None:
            return

        if len(self.desired_state.position) < 2 or len(self.actual_state.position) < 2:
            return

        desired_velocity = self.desired_state.velocity if len(self.desired_state.velocity) >= 2 else [0.0, 0.0]
        actual_velocity = self.actual_state.velocity if len(self.actual_state.velocity) >= 2 else [0.0, 0.0]

        torque = []
        for i in range(2):
            position_error = self.desired_state.position[i] - self.actual_state.position[i]
            velocity_error = desired_velocity[i] - actual_velocity[i]
            tau = self.kp[i] * position_error + self.kd[i] * velocity_error
            tau = max(min(tau, self.torque_limit), -self.torque_limit)
            torque.append(tau)

        msg = Float64MultiArray()
        msg.data = torque
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
