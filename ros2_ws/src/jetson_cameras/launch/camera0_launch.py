from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

ENABLE_ARUCO = True

def generate_launch_description():
    jetson_cameras_dir = get_package_share_directory('jetson_cameras')
    camera_params = jetson_cameras_dir + '/config/camera_params.yaml'

    camera_node = Node(
        package='jetson_cameras',
        executable='camera_publisher',
        name='camera_publisher0',
        #output='screen',
        parameters=[{'camera_id': 0}, camera_params],
    )

    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node0',
        output='screen',
        parameters=[{
            'marker_size': 0.052,
            'aruco_dictionary_id': 'DICT_4X4_250',
            'image_topic': '/camera_0/image_raw', 
            'camera_info_topic': '/camera_0/camera_info'
        }],
        # remappings=[
        #     ('/image_raw', '/camera_0/image_raw'),
        #     ('/camera_info', '/camera_0/image_raw'),
        # ]
    )

    if ENABLE_ARUCO:
        return LaunchDescription([
                camera_node,
                aruco_node
        ])
    else:
        return LaunchDescription([
            camera_node
        ])

