from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gscam2',
            executable='gscam_main',
            name='gscam_node',
            parameters=[{'camera_info_url': 'file:///path_to_camera_info_file', 
                         'gscam_config': 'tcpclientsrc host=172.17.0.1 port=5000 ! jpegdec ! videoconvert'}],
            output='screen'
        ),
    ])

