from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('aruco_opencv'), 'config', 'aruco_tracker.yaml')
    
    return LaunchDescription([
        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name='aruco_node',
            parameters=[config, {'cam_base_topic': '/zed/zed_node/left/image_rect_color/compressed'}, {'output_frame': 'zed_left_camera_frame'}, {'board_descriptions_path': ''}, {' image_sub_compressed' : True}],
            output='screen'
        ),
    ])