from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            # Include camera0_launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource("camera0_launch.py"),
            ),
            # Include camera1_launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource("camera1_launch.py"),
            ),
            # Include camera2_launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource("camera2_launch.py"),
            ),
            # Include camera3_launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource("camera3_launch.py"),
            ),
        ]
    )
