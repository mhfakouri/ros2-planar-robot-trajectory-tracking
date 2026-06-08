"""Simple 2-DOF planar robot dynamics simulator with disturbance injection."""

from __future__ import annotations

import math
import random
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PlanarRobotDynamics(Node):
    """Integrate 2-link horizontal planar robot dynamics."""

    def __init__(self) -> None:
        super().__init__('planar_robot_dynamics')

        self.declare_parameter('simulation_rate_hz', 100.0)
        self.declare_parameter('disturbance_enabled', True)
        self.declare_parameter('disturbance_std', 0.15)
        self.declare_parameter('sinusoidal_disturbance_amplitude', 0.25)
        self.declare_parameter('viscous_friction', [0.10, 0.08])
        self.declare_parameter('random_seed', 7)

        self.simulation_rate_hz = float(self.get_parameter('simulation_rate_hz').value)
        self.dt = 1.0 / max(self.simulation_rate_hz, 1.0)
        self.disturbance_enabled = bool(self.get_parameter('disturbance_enabled').value)
        self.disturbance_std = float(self.get_parameter('disturbance_std').value)
        self.sin_dist_amp = float(self.get_parameter('sinusoidal_disturbance_amplitude').value)
        self.viscous_friction = [float(v) for v in self.get_parameter('viscous_friction').value]
        random_seed = int(self.get_parameter('random_seed').value)
        random.seed(random_seed)

        # Robot parameters for a lightweight horizontal 2-link manipulator.
        self.l1 = 0.45
        self.l2 = 0.35
        self.lc1 = self.l1 / 2.0
        self.lc2 = self.l2 / 2.0
        self.m1 = 1.2
        self.m2 = 0.9
        self.i1 = self.m1 * self.l1**2 / 12.0
        self.i2 = self.m2 * self.l2**2 / 12.0

        self.q = np.array([0.0, 0.0], dtype=float)
        self.q_dot = np.array([0.0, 0.0], dtype=float)
        self.latest_torque = np.array([0.0, 0.0], dtype=float)
        self.elapsed_time = 0.0

        self.create_subscription(Float64MultiArray, '/control_torque', self._torque_callback, 10)
        self.state_publisher = self.create_publisher(JointState, '/actual_joint_states', 10)

        self.timer = self.create_timer(self.dt, self._simulation_step)
        self.get_logger().info('2-DOF planar robot dynamics simulator started.')

    def _torque_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 2:
            self.latest_torque = np.array([float(msg.data[0]), float(msg.data[1])], dtype=float)

    def _mass_matrix(self, q: np.ndarray) -> np.ndarray:
        q2 = q[1]
        c2 = math.cos(q2)
        m11 = self.i1 + self.i2 + self.m1 * self.lc1**2 + self.m2 * (
            self.l1**2 + self.lc2**2 + 2.0 * self.l1 * self.lc2 * c2
        )
        m12 = self.i2 + self.m2 * (self.lc2**2 + self.l1 * self.lc2 * c2)
        m22 = self.i2 + self.m2 * self.lc2**2
        return np.array([[m11, m12], [m12, m22]], dtype=float)

    def _coriolis_vector(self, q: np.ndarray, q_dot: np.ndarray) -> np.ndarray:
        q2 = q[1]
        q1_dot, q2_dot = q_dot
        h = -self.m2 * self.l1 * self.lc2 * math.sin(q2)
        c1 = h * (2.0 * q1_dot * q2_dot + q2_dot**2)
        c2 = -h * q1_dot**2
        return np.array([c1, c2], dtype=float)

    def _disturbance(self) -> np.ndarray:
        if not self.disturbance_enabled:
            return np.zeros(2, dtype=float)

        stochastic = np.array(
            [random.gauss(0.0, self.disturbance_std), random.gauss(0.0, self.disturbance_std)],
            dtype=float,
        )
        sinusoidal = self.sin_dist_amp * np.array(
            [
                math.sin(2.0 * math.pi * 0.7 * self.elapsed_time),
                math.cos(2.0 * math.pi * 0.5 * self.elapsed_time),
            ],
            dtype=float,
        )
        return stochastic + sinusoidal

    def _simulation_step(self) -> None:
        mass_matrix = self._mass_matrix(self.q)
        coriolis = self._coriolis_vector(self.q, self.q_dot)
        friction = np.array(self.viscous_friction, dtype=float) * self.q_dot
        disturbance = self._disturbance()

        q_ddot = np.linalg.solve(mass_matrix, self.latest_torque + disturbance - coriolis - friction)

        # Semi-implicit Euler integration.
        self.q_dot = self.q_dot + q_ddot * self.dt
        self.q = self.q + self.q_dot * self.dt
        self.elapsed_time += self.dt

        self._publish_state(self.latest_torque.tolist())

    def _publish_state(self, torque: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2']
        msg.position = self.q.tolist()
        msg.velocity = self.q_dot.tolist()
        msg.effort = torque
        self.state_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanarRobotDynamics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
