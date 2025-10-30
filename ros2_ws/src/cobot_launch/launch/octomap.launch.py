from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    octomap = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        remappings=[("cloud_in", "/main_camera/points")],
        parameters=[{"resolution": 0.1, "max_range": 6.0, "frame_id": "map"}],
    )

    return LaunchDescription([octomap])
