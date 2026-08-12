"""Launch the complete demonstration-based trajectory tracking pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'ros2_demo_based_tracking'

    trajectory_file = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'demo_trajectory.csv']
    )

    output_csv_arg = DeclareLaunchArgument(
        'output_csv',
        default_value='/tmp/ros2_demo_tracking_log.csv',
        description='CSV file used by the tracking-error logger.',
    )

    disturbance_arg = DeclareLaunchArgument(
        'disturbance_enabled',
        default_value='true',
        description='Enable or disable disturbance injection in the simulated dynamics.',
    )

    return LaunchDescription(
        [
            output_csv_arg,
            disturbance_arg,
            Node(
                package=package_name,
                executable='demo_trajectory_publisher',
                name='demo_trajectory_publisher',
                output='screen',
                parameters=[
                    {
                        'trajectory_file': trajectory_file,
                        'publish_rate_hz': 100.0,
                        'loop': True,
                    }
                ],
            ),
            Node(
                package=package_name,
                executable='planar_robot_dynamics',
                name='planar_robot_dynamics',
                output='screen',
                parameters=[
                    {
                        'simulation_rate_hz': 100.0,
                        'disturbance_enabled': LaunchConfiguration('disturbance_enabled'),
                        'disturbance_std': 0.15,
                        'sinusoidal_disturbance_amplitude': 0.25,
                    }
                ],
            ),
            Node(
                package=package_name,
                executable='pd_controller',
                name='pd_controller',
                output='screen',
                parameters=[
                    {
                        'kp': [30.0, 12.0],
                        'kd': [5.0, 1.5],
                        'control_rate_hz': 100.0,
                        'torque_limit': 20.0,
                    }
                ],
            ),
            Node(
                package=package_name,
                executable='tracking_error_logger',
                name='tracking_error_logger',
                output='screen',
                parameters=[
                    {
                        'log_rate_hz': 50.0,
                        'output_csv': LaunchConfiguration('output_csv'),
                    }
                ],
            ),
        ]
    )
