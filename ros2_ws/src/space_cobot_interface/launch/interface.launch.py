from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description(): 
    return LaunchDescription([
        Node (
            package='space_cobot_interface', 
            namespace='interface', 
            name='interface_slave', 
            executable='spaceCobotInterface', 
            output='screen', 
            emulate_tty=True, 
        ), 
        Node(
            package='space_cobot_interface', 
            namespace='interface', 
            name='interface_master', 
            executable='interface_master', 
            output='screen', 
            emulate_tty=True, 
        )
    ])


def main():
    generate_launch_description()


if __name__ == '__main__': 
    main()
