from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="jetson_cameras",
                executable="camera_publisher",
                name="camera_publisher2",
                output="screen",
                parameters=[{"camera_id": 2}],
            )
        ]
    )
