import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Pose,
    Point,
    Vector3,
    Quaternion,
)
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from aruco_opencv_msgs.msg import ArucoDetection


class EkfSlamNode(Node):
    """
    EKF-SLAM node: fuses 6-DOF ZED poses as motion updates
    and 3D ArUco landmark detections. Publishes robot pose,
    path, and debug markers for landmarks at 30ms.
    """

    def __init__(self):
        super().__init__("ekf_slam_node")

        # State: robot [x,y,z,roll,pitch,yaw] + landmarks [x1,y1,z1...]
        self.x = np.zeros(6)
        self.P = np.eye(6) * 1e-3  # Initial covariance
        self.landmarks = {}

        odom_std = [
            0.2,  # x translation std-dev (m)
            0.2,  # y translation std-dev (m)
            0.2,  # z translation std-dev (m)
            np.deg2rad(6),  # roll std-dev (rad)
            np.deg2rad(6),  # pitch std-dev (rad)
            np.deg2rad(6),  # yaw std-dev (rad)
        ]
        self.R = np.diag(odom_std) ** 2
        # Process noise
        self.Q = np.eye(3) * 0.01**2

        self.prev_odom = None

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/ekf_slam/pose", 10
        )
        self.path_pub = self.create_publisher(Path, "/ekf_slam/path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/ekf_slam/landmarks", 10)

        # Path message
        self.path = Path()
        self.path.header.frame_id = "map"

        # Subscribers
        self.create_subscription(
            PoseStamped, "/zed/zed_node/pose", self.odom_callback, 10
        )
        self.create_subscription(
            ArucoDetection, "/aruco_detections", self.aruco_callback, 10
        )

        # Timer for fixed-rate publishing
        self.publish_timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg: PoseStamped):
        # Motion update: 6-DOF delta
        pos = msg.pose.position
        orn = msg.pose.orientation
        curr = np.array(
            [
                pos.x,
                pos.y,
                pos.z,
                *R.from_quat([orn.x, orn.y, orn.z, orn.w]).as_euler("xyz"),
            ]
        )
        if self.prev_odom is None:
            self.prev_odom = curr
            return
        delta = curr - self.prev_odom
        delta[3:] = self._angle_wrap(delta[3:])
        self.prev_odom = curr
        self.x[:6] += delta
        self.P[:6, :6] += self.R

    def aruco_callback(self, msg: ArucoDetection):
        # Measurement update
        p = self.x[:3]
        eul = self.x[3:6]
        Rcw = R.from_euler("xyz", eul).as_matrix()
        for m in msg.markers:
            mid = m.marker_id
            z = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            if mid not in self.landmarks:
                idx = len(self.x)
                world_coord = p + Rcw @ z
                self._add_landmark(idx, world_coord, mid)
            else:
                idx = self.landmarks[mid]
                self._update_landmark(idx, z, p, Rcw)
            self.get_logger().debug(
                f"Landmarks={len(self.landmarks)} Pose={self.x[:6]}"
            )

    def timer_callback(self):
        # Timestamp
        now = self.get_clock().now().to_msg()
        # Pose
        self.publish_pose(now)
        # Path
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "map"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = self.x[0:3]
        q = R.from_euler("xyz", self.x[3:6]).as_quat()
        ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.path.header.stamp = now
        self.path.poses.append(ps)
        self.path_pub.publish(self.path)
        # Landmarks as cubes
        ma = MarkerArray()
        for mid, idx in self.landmarks.items():
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = "map"
            marker.ns = "landmarks"
            marker.id = mid
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            # Position as Point
            marker.pose = Pose()
            marker.pose.position = Point(
                x=float(self.x[idx]), y=float(self.x[idx + 1]), z=float(self.x[idx + 2])
            )
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            ma.markers.append(marker)
        self.marker_pub.publish(ma)

    def _add_landmark(self, idx: int, coords: np.ndarray, mid: int):
        self.landmarks[mid] = idx
        self.x = np.hstack((self.x, coords))
        P_aug = np.zeros((self.x.size, self.x.size))
        P_aug[: self.P.shape[0], : self.P.shape[1]] = self.P
        P_aug[idx : idx + 3, idx : idx + 3] = np.eye(3) * 1e3
        self.P = P_aug

    def _update_landmark(self, idx: int, z: np.ndarray, p: np.ndarray, Rcw: np.ndarray):
        lw = self.x[idx : idx + 3]
        z_hat = Rcw.T @ (lw - p)
        H = np.zeros((3, self.x.size))
        H[:, :3] = -Rcw.T
        H[:, idx : idx + 3] = Rcw.T
        y = z - z_hat
        S = H @ self.P @ H.T + self.Q
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(self.x.size) - K @ H) @ self.P

    def publish_pose(self, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])
        msg.pose.pose.position.z = float(self.x[2])
        q = R.from_euler("xyz", self.x[3:6]).as_quat()
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        cov6 = self.P[:6, :6]
        msg.pose.covariance = cov6.flatten().tolist()
        self.pose_pub.publish(msg)

    @staticmethod
    def _angle_wrap(angles: np.ndarray) -> np.ndarray:
        return (angles + np.pi) % (2 * np.pi) - np.pi


def main(args=None):
    rclpy.init(args=args)
    node = EkfSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
