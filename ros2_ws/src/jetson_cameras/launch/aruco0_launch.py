from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

ENABLE_ARUCO = True


def generate_launch_description():
    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node0",
        output="screen",
        parameters=[
            {
                "marker_size": 0.052,
                "aruco_dictionary_id": "DICT_4X4_250",
                "image_topic": "/camera_0/image_raw",
                "camera_info_topic": "/camera_0/camera_info",
            }
        ],
    )

    return LaunchDescription([aruco_node])
