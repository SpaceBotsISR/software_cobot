from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

ENABLE_ARUCO = False   


def generate_launch_description():
    jetson_cameras_dir = get_package_share_directory("jetson_cameras")
    camera_params = jetson_cameras_dir + "/config/camera_params.yaml"

    camera_node = Node(
        package="jetson_cameras",
        executable="camera_publisher",
        name="camera_publisher0",
        parameters=[{"camera_id": 0}, camera_params],
    )

    aruco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(jetson_cameras_dir + "/launch/aruco0_launch.py"),
    )

    if ENABLE_ARUCO:
        return LaunchDescription([camera_node, aruco_node])
    else:
        return LaunchDescription([camera_node])
