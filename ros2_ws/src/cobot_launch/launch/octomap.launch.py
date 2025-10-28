from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    cobot_gz_launch = PathJoinSubstitution(
        [get_package_share_directory("cobot_gz"), "launch", "cobot_gz.launch.py"]
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(cobot_gz_launch))

    octomap = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        remappings=[("cloud_in", "/main_camera/points")],
        parameters=[{"resolution": 0.05, "max_range": 6.0, "frame_id": "map"}],
    )

    return LaunchDescription([gazebo, octomap])
