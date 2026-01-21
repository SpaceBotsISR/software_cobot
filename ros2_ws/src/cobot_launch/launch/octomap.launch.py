from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cobot_gz_share = get_package_share_directory("cobot_gz")
    iss_map_path = PathJoinSubstitution(
        [cobot_gz_share, "maps", LaunchConfiguration("iss_map_file")]
    )
    maybe_iss_map = PythonExpression(
        [
            "'",
            iss_map_path,
            "' if '",
            LaunchConfiguration("use_iss_map"),
            "' == 'true' else ''",
        ]
    )

    args = [
        DeclareLaunchArgument("use_iss_map", default_value="false"),
        DeclareLaunchArgument("iss_map_file", default_value="iss.bt"),
    ]

    octomap = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        remappings=[("cloud_in", "/main_camera/points")],
        parameters=[
            {
                "resolution": 0.10,
                "max_range": 6.0,
                "frame_id": "world",
                "use_sim_time": True,
                "octomap_path": maybe_iss_map,
            }
        ],
    )

    return LaunchDescription([*args, octomap])
