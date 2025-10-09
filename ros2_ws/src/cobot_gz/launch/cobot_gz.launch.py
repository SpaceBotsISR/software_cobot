import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


# === Default Values ===
DEFAULT_WORLD = "world.sdf"
DEFAULT_BRIDGE = "ros_gz_bridge"
DEFAULT_CONFIG = ""
DEFAULT_USE_COMPOSITION = True
DEFAULT_CREATE_CONTAINER = True

# --- Bridge topics ---
BRIDGE_TOPICS = [
    "/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU",
    "/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
    "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
    "/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
]


def generate_launch_description():
    # --- Paths ---
    cobot_gz_path = get_package_share_directory("cobot_gz")
    ros_gz_sim_path = get_package_share_directory("ros_gz_sim")

    # --- Launch Arguments ---
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

    # --- Resolve world path ---
    world_path = PathJoinSubstitution(
        [cobot_gz_path, "sdf", LaunchConfiguration("world_sdf_file")]
    )

    # --- Gazebo sim launch (from ros_gz_sim) ---
    gz_launch = PathJoinSubstitution([ros_gz_sim_path, "launch", "gz_sim.launch.py"])

    # --- Environment setup ---
    resource_path = os.pathsep.join(
        [
            cobot_gz_path,
            os.path.join(cobot_gz_path, "sdf"),
            os.path.join(cobot_gz_path, "meshes"),
        ]
    )

    # --- Return Launch Description ---
    return LaunchDescription(
        [
            *args,
            # Gazebo resource and plugin paths
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable(
                "GZ_SIM_PLUGIN_PATH", PathJoinSubstitution([cobot_gz_path, "plugins"])
            ),
            # === Launch Gazebo ===
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    # Compose lazy so final shell command receives "-r <path>"
                    "gz_args": [TextSubstitution(text="-r "), world_path],
                    "on_exit_shutdown": "True",
                }.items(),
            ),
            # === Bridge sensors ===
            Node(
                package=LaunchConfiguration("bridge_name"),
                executable="parameter_bridge",
                name="sensor_bridge",
                arguments=BRIDGE_TOPICS,
                output="screen",
            ),
        ]
    )
