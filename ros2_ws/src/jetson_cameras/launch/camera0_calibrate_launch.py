import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    jetson_cameras_dir = get_package_share_directory('jetson_cameras')

    launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(jetson_cameras_dir, 'launch', 'camera0_launch.py'))
    )

    output_file = jetson_cameras_dir + '/config/calibration_c0_data.yaml'

    # Define the camera calibration node
    camera_calibration_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='camera_calibration',
        output="screen",
        parameters=[
            {'image': "/camera_0/image_raw"},
            {'camera': "/camera_0"},
            {'output': output_file},
            {'size': '8x11'}, 
            {'square': '0.02'},   
        ]
    )

    # Return the launch description with all nodes and configurations
    return LaunchDescription([
        launch_camera,
        camera_calibration_node
    ])
