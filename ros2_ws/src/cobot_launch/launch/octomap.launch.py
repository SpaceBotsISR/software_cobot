from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    octomap = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        remappings=[("cloud_in", "/main_camera/points")],
        parameters=[
            {
                "resolution": 0.20,
                "max_range": 6.0,
                "frame_id": "map",
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription([octomap])
