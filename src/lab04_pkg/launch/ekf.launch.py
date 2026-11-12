"""
Start the EKF node for the simulation or the real robot.

ros2 launch lab04_pkg ekf.launch.py                  # simulation, Task 1
ros2 launch lab04_pkg ekf.launch.py fusion:=true     # simulation, Task 2
ros2 launch lab04_pkg ekf.launch.py robot:=real      # real robot, Task 3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIGS = {'sim': 'sim.yaml', 'real': 'real_robot.yaml'}


def make_node(context):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in CONFIGS:
        raise RuntimeError(f"robot must be one of {list(CONFIGS)}, got '{robot}'")
    fusion = LaunchConfiguration('fusion').perform(context).lower() == 'true'
    params = os.path.join(get_package_share_directory('lab04_pkg'), 'config', CONFIGS[robot])
    return [Node(package='lab04_pkg', executable='ekf_node', name='ekf_node',
                 parameters=[params, {'fusion': fusion}], output='screen', emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', description='sim or real'),
        DeclareLaunchArgument('fusion', default_value='false', description='Task 2 state'),
        OpaqueFunction(function=make_node),
    ])
