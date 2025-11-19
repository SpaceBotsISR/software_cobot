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
    nav6d_share = get_package_share_directory("nav6d")

    cobot_gz_launch = PathJoinSubstitution(
        [cobot_gz_share, "launch", "cobot_gz.launch.py"]
    )
    octomap_launch = PathJoinSubstitution(
        [cobot_launch_share, "launch", "octomap.launch.py"]
    )
    planner_launch = PathJoinSubstitution(
        [nav6d_share, "launch", "n6d_planner.launch.py"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cobot_gz_launch),
                launch_arguments={"vel_input_frame": "world"}.items(),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(octomap_launch)),
            Node(
                package="teleop",
                executable="teleop_node",
                name="teleop_bridge",
                output="log",
                parameters=[
                    {
                        "use_sim_time": True,
                        "default_input_frame": "world",
                        "command_frame": "body",  # Treat incoming UI commands as world-frame and rotate to body-frame
                        "allow_cmd_frame_override": True,
                    }
                ],
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(planner_launch)
            # ),
        ]
    )
