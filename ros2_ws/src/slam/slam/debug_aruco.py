import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
)
from visualization_msgs.msg import Marker
from aruco_opencv_msgs.msg import ArucoDetection


class EkfSlamNode(Node):
    """A node that visualizes detected ArUco markers in the 'map' frame."""

    def __init__(self):
        super().__init__("ekf_slam_node")

        self.prev_zed_pose: Optional[PoseStamped] = None

        # Publishers and subscribers
        self.marker_pub = self.create_publisher(Marker, "ekf_slam/aruco_markers", 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "ekf_slam/pose", 10
        )
        self.path_pub = self.create_publisher(
            PoseWithCovarianceStamped, "ekf_slam/path", 10
        )

        self.create_subscription(
            PoseStamped, "/zed/zed_node/pose", self.pose_callback, 10
        )
        self.create_subscription(
            ArucoDetection, "/aruco_detections", self.aruco_callback, 10
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        """Cache the latest camera→map pose."""
        self.prev_zed_pose = msg

    def aruco_callback(self, msg: ArucoDetection) -> None:
        """Convert each detected marker into the map frame and publish as a sphere."""
        if self.prev_zed_pose is None:
            self.get_logger().warning("No camera pose yet; skipping ArUco processing")
            return

        for m in msg.markers:
            marker_in_map = self.transform_pose_to_map(m.pose)
            visualization = self.make_marker(
                msg.header.stamp, m.marker_id, marker_in_map
            )
            self.marker_pub.publish(visualization)

    def transform_pose_to_map(self, camera_pose: Pose) -> Pose:
        """
        Transform a pose from the camera frame into the map frame.

        Uses the cached self._latest_camera_pose as the camera→map transform.
        """
        cam_to_map = self.prev_zed_pose.pose

        # Convert positions to arrays
        cam_pos = np.array(
            [camera_pose.position.x, camera_pose.position.y, camera_pose.position.z]
        )
        map_pos = np.array(
            [cam_to_map.position.x, cam_to_map.position.y, cam_to_map.position.z]
        )

        # Convert orientations to Rotation objects
        cam_rot = R.from_quat(
            [
                camera_pose.orientation.x,
                camera_pose.orientation.y,
                camera_pose.orientation.z,
                camera_pose.orientation.w,
            ]
        )
        map_rot = R.from_quat(
            [
                cam_to_map.orientation.x,
                cam_to_map.orientation.y,
                cam_to_map.orientation.z,
                cam_to_map.orientation.w,
            ]
        )

        # Transform position and orientation
        world_pos = map_pos + map_rot.apply(cam_pos)
        world_rot = map_rot * cam_rot

        # Build a new Pose message
        out = Pose()
        out.position.x, out.position.y, out.position.z = world_pos
        qx, qy, qz, qw = world_rot.as_quat()
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw

        return out

    def make_marker(self, stamp, marker_id: int, pose: Pose) -> Marker:
        """Create a green sphere Marker for visualization in RViz."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.ns = "aruco_debug"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose = pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = EkfSlamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
