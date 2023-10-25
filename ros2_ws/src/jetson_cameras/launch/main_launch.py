from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    jetson_cameras_dir = get_package_share_directory('jetson_cameras')

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(jetson_cameras_dir + '/launch/camera0_launch.py'),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(jetson_cameras_dir + '/launch/camera1_launch.py'),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(jetson_cameras_dir + '/launch/camera2_launch.py'),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(jetson_cameras_dir + '/launch/camera3_launch.py'),
            ),
        ]
    )
