import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

CAMERA_ID = 0 # change this to the camera you want to calibrate

def generate_launch_description():
    id = str(CAMERA_ID)

    # Launch camera
    jetson_cameras_dir = get_package_share_directory('jetson_cameras')
    launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(jetson_cameras_dir, 'launch', 'camera' + id + '_launch.py'))
    )
    
    camera_calibrator_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='camera_calibrator',
        output='screen',
        parameters=[],
        remappings=[
            ('image', '/camera_' + id + '/image_raw'),
        ],
        arguments=['--size', '7x10', '--square', '0.02', 'image:=/camera_' + id + '/image_raw', 'camera:=/camera_' + id],
    )

    return LaunchDescription([
        launch_camera,
        camera_calibrator_node
    ])
