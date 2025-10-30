#!/usr/bin/env python3
"""Launch teleop stack with Gazebo simulation and the collision map."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cobot_gz_share = get_package_share_directory("cobot_gz")
    cobot_launch_share = get_package_share_directory("cobot_launch")
    teleop_share = get_package_share_directory("teleop")

    cobot_gz_launch = PathJoinSubstitution(
        [cobot_gz_share, "launch", "cobot_gz.launch.py"]
    )
    octomap_launch = PathJoinSubstitution(
        [cobot_launch_share, "launch", "octomap.launch.py"]
    )
    local_planner_config = PathJoinSubstitution(
        [teleop_share, "config", "local_planner.yaml"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(cobot_gz_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(octomap_launch)),
            Node(
                package="teleop",
                executable="teleop_node",
                name="teleop_bridge",
                output="log",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="teleop",
                executable="local_planner_node",
                name="simple_local_planner",
                namespace="teleop",
                output="screen",
                parameters=[local_planner_config, {"use_sim_time": True}],
            ),
        ]
    )
