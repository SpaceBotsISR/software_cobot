#!/usr/bin/env python3
"""Launch file for the simple 3D local planner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("teleop"), "config", "local_planner.yaml"
    )

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            Node(
                package="teleop",
                executable="local_planner_node",
                name="simple_local_planner",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
