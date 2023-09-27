from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    foxglove_bridge = Node ( 
            package='foxglove_bridge',
            namespace="Foxglove", 
            executable='foxglove_bridge',
            name="foxglove_control_node"
        )
    
    ld.add_action(foxglove_bridge)
    return ld