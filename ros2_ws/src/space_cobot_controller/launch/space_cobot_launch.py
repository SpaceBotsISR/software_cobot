from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction

import os 

from  ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch  =  os.path.join (
        get_package_share_directory ('space_cobot_controller'),
        'config',
        'autox.yml'
    )
    
    return LaunchDescription([
        Node (
            package='space_cobot_interface', 
            namespace='Space_Cobot_basic',
            name='space_cobot_interface', 
            executable='spaceCobotInterface', 
            output='screen', 
        ), 
        Node (
            package='space_cobot_controller',
            namespace='Space_Cobot_basic',
            name='space_cobot_controller',
            executable='spaceCobotController',
            output='screen',
        )
    ])

def main(): 
    generate_launch_description()

if __name__ == '__main__':
    main()