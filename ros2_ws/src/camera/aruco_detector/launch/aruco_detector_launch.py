from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="camera_info_publisher",
                executable="camera_info_publisher",
                name="camera_info_publisher",
                output="screen",
                parameters=[
                    {
                        "config_file_path": "/cobot/src/camera/camera_info_publisher/config/camera_calibration.yaml"
                    },  # Set the path to your YAML file
                    {"rate": 1.0},  # Set the desired rate
                    {"camera_namespace": "/camera"},
                ],
            ),
            Node(
                package="aruco_opencv",
                executable="aruco_tracker",
                name="aruco_tracker",
                output="screen",
                parameters=[
                    {"cam_base_topic": "/camera/image_raw"},
                    {"marker_size": 0.037},
                    {"cam_info_topic": "/camera/camera_info"},
                ],
            )
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
