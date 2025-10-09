from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

# Default values
DEFAULT_WORLD = "world.sdf"
DEFAULT_BRIDGE = "ros_gz_bridge"
DEFAULT_CONFIG = ""
DEFAULT_USE_COMPOSITION = True
DEFAULT_CREATE_CONTAINER = True


def generate_launch_description():
    cobot_gz_path = get_package_share_directory("cobot_gz")
    ros_gz_sim_path = get_package_share_directory("ros_gz_sim")

    args = [
        DeclareLaunchArgument("world_sdf_file", default_value=DEFAULT_WORLD),
        DeclareLaunchArgument("bridge_name", default_value=DEFAULT_BRIDGE),
        DeclareLaunchArgument("config_file", default_value=DEFAULT_CONFIG),
        DeclareLaunchArgument(
            "use_composition", default_value=str(DEFAULT_USE_COMPOSITION)
        ),
        DeclareLaunchArgument(
            "create_own_container", default_value=str(DEFAULT_CREATE_CONTAINER)
        ),
    ]

    world_path = PathJoinSubstitution(
        [cobot_gz_path, "urdf", LaunchConfiguration("world_sdf_file")]
    )

    gz_launch = PathJoinSubstitution([ros_gz_sim_path, "launch", "gz_sim.launch.py"])

    return LaunchDescription(
        [
            *args,
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([cobot_gz_path, "meshes"])
            ),
            SetEnvironmentVariable(
                "GZ_SIM_PLUGIN_PATH", PathJoinSubstitution([cobot_gz_path, "plugins"])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    "gz_args": world_path,
                    "on_exit_shutdown": "True",
                }.items(),
            ),
            Node(
                package=LaunchConfiguration("bridge_name"),
                executable="parameter_bridge",
                arguments=["/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU"],
                remappings=[("/example_imu_topic", "/remapped_imu_topic")],
                output="screen",
            ),
        ]
    )
