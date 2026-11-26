# Copyright 2026 SESASR Group 203 (SIQUBro)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


"""
Start the DWA navigation node for simulation or the real robot.

ros2 launch lab07_pkg dwa.launch.py                             # simulation, Task 1 (goal)
ros2 launch lab07_pkg dwa.launch.py robot:=sim mode:=follow     # simulation, Task 2 (follow)
ros2 launch lab07_pkg dwa.launch.py robot:=real mode:=goal      # real robot, Task 3 (goal)
ros2 launch lab07_pkg dwa.launch.py robot:=real use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIGS = {'sim': 'sim.yaml', 'real': 'real_robot.yaml'}
MODES = ('goal', 'follow')


def make_node(context):
    """Build the DWA Node action from launch arguments."""
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in CONFIGS:
        raise RuntimeError(f"robot must be one of {list(CONFIGS)}, got '{robot}'")
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODES:
        raise RuntimeError(f"mode must be one of {list(MODES)}, got '{mode}'")

    params = os.path.join(get_package_share_directory('lab07_pkg'), 'config', CONFIGS[robot])
    overrides = {'mode': mode}
    goal_source = LaunchConfiguration('goal_source').perform(context).strip()
    if goal_source != '':
        if goal_source not in ('topic', 'landmark'):
            raise RuntimeError(
                f"goal_source must be 'topic' or 'landmark', got '{goal_source}'"
            )
        overrides['goal_source'] = goal_source
    target_id_str = LaunchConfiguration('target_id').perform(context).strip()
    if target_id_str != '':
        overrides['target_id'] = int(target_id_str)
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower()
    if sim_time in ('true', 'false'):
        overrides['use_sim_time'] = sim_time == 'true'

    return [
        Node(
            package='lab07_pkg',
            executable='dwa_node',
            name='dwa_node',
            parameters=[params, overrides],
            output='screen',
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    """Declare launch arguments and create the DWA node launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', description='sim or real'),
        DeclareLaunchArgument('mode', default_value='goal', description='goal or follow'),
        DeclareLaunchArgument(
            'goal_source',
            default_value='',
            description='override goal_source in goal mode (topic or landmark)',
        ),
        DeclareLaunchArgument(
            'target_id',
            default_value='',
            description='AprilTag ID to follow (required on real robot in follow mode)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='',
            description='override use_sim_time from config (true/false)',
        ),
        OpaqueFunction(function=make_node),
    ])
