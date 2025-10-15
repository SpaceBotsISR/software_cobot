import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
)
from launch_ros.actions import Node


# === Default Values ===
DEFAULT_WORLD = "world.sdf"
DEFAULT_BRIDGE = "ros_gz_bridge"
DEFAULT_USE_COMPOSITION = True
DEFAULT_CREATE_CONTAINER = True


def generate_launch_description():
    # --- Paths ---
    cobot_gz_path = get_package_share_directory("cobot_gz")
    ros_gz_sim_path = get_package_share_directory("ros_gz_sim")
    default_bridge_config = PathJoinSubstitution(
        [cobot_gz_path, "config", "ros_gz_bridge.yaml"]
    )

    # --- Launch Arguments ---
    args = [
        DeclareLaunchArgument("world_sdf_file", default_value=DEFAULT_WORLD),
        DeclareLaunchArgument("bridge_name", default_value=DEFAULT_BRIDGE),
        DeclareLaunchArgument("config_file", default_value=default_bridge_config),
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
    runtime_dir = os.path.join("/tmp", f"runtime-{os.getuid()}")

    # Ensure Gazebo's Qt dependencies have a writable runtime directory.
    try:
        os.makedirs(runtime_dir, exist_ok=True)
        os.chmod(runtime_dir, 0o700)
    except OSError as err:
        raise RuntimeError(
            f"Failed to prepare XDG runtime directory at '{runtime_dir}': {err}"
        ) from err

    # --- Return Launch Description ---
    return LaunchDescription(
        [
            *args,
            # Gazebo resource and plugin paths
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable(
                "GZ_SIM_PLUGIN_PATH", PathJoinSubstitution([cobot_gz_path, "plugins"])
            ),
            SetEnvironmentVariable("XDG_RUNTIME_DIR", runtime_dir),
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
                parameters=[
                    {"config_file": LaunchConfiguration("config_file")},
                ],
                output="screen",
            ),
        ]
    )
