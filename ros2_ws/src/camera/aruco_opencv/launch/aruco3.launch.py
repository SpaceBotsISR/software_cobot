from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('aruco_opencv'), 'config', 'aruco_tracker.yaml')
    board_descriptions = os.path.join(get_package_share_directory('aruco_opencv'), 'config', 'board_descriptions.yaml')
    
    return LaunchDescription([
        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name='aruco_node3',
            parameters=[config, {'cam_base_topic': 'camera_3/image_raw'}, {'output_frame': 'camera_frame_3'}, {'board_descriptions_path': ''}],
            output='screen'
        ),
    ])
