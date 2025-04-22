#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Include the existing aruco_opencv launch
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("aruco_opencv"), "launch", "aruco.launch.py"
            )
        )
    )

    # Launch the EKF‐SLAM node from your slam package
    slam_node = Node(
        package="slam",
        executable="ekf",
        name="ekf_slam_node",
        output="screen",
    )

    return LaunchDescription(
        [
            aruco_launch,
            slam_node,
        ]
    )
