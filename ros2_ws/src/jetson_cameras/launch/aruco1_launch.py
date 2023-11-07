from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node1",
        output="screen",
        parameters=[
            {
                "marker_size": 0.052,
                "aruco_dictionary_id": "DICT_4X4_250",
                "image_topic": "/camera_1/image_raw",
                "camera_info_topic": "/camera_1/camera_info",
            }
        ],
        remappings=[
            ("/aruco_poses", "/camera_1/aruco_poses"),
            ("/aruco_markers", "/camera_1/aruco_markers"),
        ],
    )

    return LaunchDescription([aruco_node])
