"""
Start the EKF node for the simulation or the real robot.

ros2 launch lab04_pkg ekf.launch.py                  # simulation, Task 1
ros2 launch lab04_pkg ekf.launch.py fusion:=true     # simulation, Task 2
ros2 launch lab04_pkg ekf.launch.py robot:=real      # real robot, Task 3
ros2 launch lab04_pkg ekf.launch.py robot:=real use_sim_time:=true   # replay of a real-robot bag
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
    overrides = {'fusion': fusion}
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower()
    if sim_time in ('true', 'false'):   # replay: ros2 bag play --clock needs use_sim_time:=true
        overrides['use_sim_time'] = sim_time == 'true'
    return [Node(package='lab04_pkg', executable='ekf_node', name='ekf_node',
                 parameters=[params, overrides], output='screen', emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', description='sim or real'),
        DeclareLaunchArgument('fusion', default_value='false', description='Task 2 state'),
        DeclareLaunchArgument('use_sim_time', default_value='',
                              description='override the config: true when replaying a bag'),
        OpaqueFunction(function=make_node),
    ])
