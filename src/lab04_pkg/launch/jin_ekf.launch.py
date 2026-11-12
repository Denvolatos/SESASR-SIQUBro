"""Launch Jin's Task 1 EKF independently from the group's existing EKF.

ros2 launch lab04_pkg jin_ekf.launch.py
ros2 launch lab04_pkg jin_ekf.launch.py robot:=real
ros2 launch lab04_pkg jin_ekf.launch.py robot:=real output_topic:=/ekf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_node(context):
    """Select parameters and optional topic/time overrides."""
    robot = LaunchConfiguration("robot").perform(context)
    if robot not in ("sim", "real"):
        raise ValueError("robot must be 'sim' or 'real'")
    config = os.path.join(
        get_package_share_directory("lab04_pkg"), "config", f"jin_{robot}.yaml"
    )
    overrides = {
        "output_topic": LaunchConfiguration("output_topic").perform(context)
    }
    sim_time = LaunchConfiguration("use_sim_time").perform(context).lower()
    if sim_time in ("true", "false"):
        overrides["use_sim_time"] = sim_time == "true"
    return [
        Node(
            package="lab04_pkg",
            executable="jin_ekf_node",
            name="jin_ekf_node",
            parameters=[config, overrides],
            output="screen",
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    """Provide simulation, real-robot and bag-replay launch options."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="sim"),
            DeclareLaunchArgument("output_topic", default_value="/ekf_jin"),
            DeclareLaunchArgument("use_sim_time", default_value=""),
            OpaqueFunction(function=make_node),
        ]
    )
