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
Start the Bump&Go node with the parameters for the simulation or the real robot.

ros2 launch lab02_pkg bump_and_go.launch.py              # simulation (config/sim.yaml)
ros2 launch lab02_pkg bump_and_go.launch.py robot:=real  # real robot (config/real_robot.yaml)
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
    params = os.path.join(get_package_share_directory('lab02_pkg'), 'config', CONFIGS[robot])
    return [Node(package='lab02_pkg', executable='bump_and_go', name='bump_and_go',
                 parameters=[params], output='screen', emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', description='sim or real'),
        OpaqueFunction(function=make_node),
    ])
