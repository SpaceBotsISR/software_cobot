from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    jetson_cameras_dir = get_package_share_directory('jetson_cameras')
    camera_params = jetson_cameras_dir + "/config/camera0_params.yaml"

    return LaunchDescription(
        [
            Node(
                package="jetson_cameras",
                executable="camera_publisher",
                name="camera_publisher0",
                #output="screen",
                parameters=[{"camera_id": 0}, camera_params],
            )
        ]
    )
