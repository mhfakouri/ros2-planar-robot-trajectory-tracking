from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_demo_based_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohammad Hossein',
    maintainer_email='your.email@example.com',
    description='ROS 2 demonstration-based trajectory tracking for a 2-DOF planar robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_trajectory_publisher = ros2_demo_based_tracking.demo_trajectory_publisher:main',
            'pd_controller = ros2_demo_based_tracking.pd_controller:main',
            'planar_robot_dynamics = ros2_demo_based_tracking.planar_robot_dynamics:main',
            'tracking_error_logger = ros2_demo_based_tracking.tracking_error_logger:main',
        ],
    },
)
