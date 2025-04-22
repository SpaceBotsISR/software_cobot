import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from aruco_opencv_msgs.msg import ArucoDetection
from nav_msgs.msg import Path

PUBLISH_FREQ = 10.0  # Hz


class EkfSlamNode(Node):
    """A node that performs a simple EKF-SLAM using ArUco detections and visualizes results."""

    def __init__(self):
        super().__init__("ekf_slam_node")

        self.prev_zed_pose: Optional[np.ndarray] = None

        # State: robot pose [x,y,z,roll,pitch,yaw] + landmarks stacked [x1,y1,z1, x2,y2,z2, ...]
        self.state = np.zeros(6)
        self.cov = np.eye(6) * 1e-3  # initial covariance
        # Process noise for motion (robot-only)
        self.noise = (
            np.diag([0.05, 0.05, 0.05, np.deg2rad(2), np.deg2rad(2), np.deg2rad(2)])
            ** 2
        )
        # Measurement noise for landmark observations
        self.R = np.eye(3) * 0.05**2
        self.landmarks = {}  # id -> start index in state

        # For path visualization
        self.path_history: list[PoseStamped] = []

        # Publishers and subscribers
        self.marker_pub = self.create_publisher(Marker, "ekf_slam/aruco_markers", 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "ekf_slam/pose", 10
        )
        self.path_pub = self.create_publisher(Path, "ekf_slam/path", 10)

        self.create_subscription(
            PoseStamped, "/zed/zed_node/pose", self.pose_callback, 10
        )
        self.create_subscription(
            ArucoDetection, "/aruco_detections", self.aruco_callback, 10
        )

        self.publish_timer = self.create_timer(
            1.0 / PUBLISH_FREQ, self.publish_timer_callback
        )

    @staticmethod
    def angle_wrap(angles: np.ndarray) -> np.ndarray:
        return (angles + np.pi) % (2 * np.pi) - np.pi

    @staticmethod
    def pose_to_numpy(pose: Pose) -> np.ndarray:
        pos = pose.position
        orn = pose.orientation
        return np.array(
            [
                pos.x,
                pos.y,
                pos.z,
                *R.from_quat([orn.x, orn.y, orn.z, orn.w]).as_euler("xyz"),
            ]
        )

    @staticmethod
    def numpy_to_pose(pose: np.ndarray) -> Pose:
        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = pose[:3]
        quat = R.from_euler("xyz", pose[3:6]).as_quat()
        (
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w,
        ) = quat
        return pose_msg

    def pose_callback(self, msg: PoseStamped) -> None:
        curr = self.pose_to_numpy(msg.pose)
        if self.prev_zed_pose is None:
            self.prev_zed_pose = curr
            return

        # Compute motion increment
        delta = curr - self.prev_zed_pose
        delta[3:] = self.angle_wrap(delta[3:])
        self.prev_zed_pose = curr

        # Prediction step: update robot pose and covariance
        self.state[:6] += delta
        self.state[3:6] = self.angle_wrap(self.state[3:6])
        F = np.eye(self.state.size)
        F[:6, :6] = np.eye(6)
        # propagate covariance for robot block
        self.cov[:6, :6] = F[:6, :6] @ self.cov[:6, :6] @ F[:6, :6].T + self.noise

    def aruco_callback(self, msg: ArucoDetection) -> None:
        # For each detected ArUco marker, get its pose relative to map via current state
        for m in msg.markers:
            landmark_id = m.marker_id
            measurement = self.transform_pose_to_map(m.pose)  # [x,y,z,roll,pitch,yaw]
            z = measurement[:3]

            if landmark_id not in self.landmarks:
                # initialize landmark in state
                idx = self.state.size
                self.add_landmark(landmark_id, idx, z)
            else:
                idx = self.landmarks[landmark_id]
                self.update_landmark(idx, z)

    def add_landmark(self, id: int, index: int, z: np.ndarray) -> None:
        """Augment state and covariance for a new landmark."""
        # register index
        self.landmarks[id] = index
        # extend state vector
        self.state = np.hstack((self.state, z))
        # augment covariance
        n = self.state.size
        P_aug = np.zeros((n, n))
        P_aug[: self.cov.shape[0], : self.cov.shape[1]] = self.cov
        P_aug[index : index + 3, index : index + 3] = np.eye(3) * 1e-3
        self.cov = P_aug

    def update_landmark(self, index: int, z: np.ndarray) -> None:
        """EKF measurement update for a known landmark."""
        # measurement model h(x) = landmark position in map = state[index:index+3]
        # Jacobian H: zeros except identity at landmark block
        n = self.state.size
        H = np.zeros((3, n))
        H[:, index : index + 3] = np.eye(3)

        # Innovation
        z_pred = self.state[index : index + 3]
        y = z - z_pred

        # Innovation covariance
        S = H @ self.cov @ H.T + self.R
        K = self.cov @ H.T @ np.linalg.inv(S)

        # State and covariance update
        self.state = self.state + K @ y
        self.cov = (np.eye(n) - K @ H) @ self.cov

    def transform_pose_to_map(self, aruco_pose: any) -> np.ndarray:
        """Transform a Pose (in camera frame) to map frame using current robot state."""
        # ensure numpy form
        if not isinstance(aruco_pose, (np.ndarray, Pose)):
            raise TypeError("Expected Pose or numpy.ndarray for aruco_pose")
        if isinstance(aruco_pose, Pose):
            aruco_pose = self.pose_to_numpy(aruco_pose)

        # robot pose
        pos_r = self.state[:3]
        orn_r = self.state[3:6]
        Rm = R.from_euler("xyz", orn_r).as_matrix()
        # measurement in camera
        t_c = aruco_pose[:3]
        R_c = R.from_euler("xyz", aruco_pose[3:6]).as_matrix()

        # transform to map: p_m = p_r + Rm * t_c
        p_m = pos_r + Rm.dot(t_c)
        # orientation composition
        R_m = Rm.dot(R_c)
        euler_m = R.from_matrix(R_m).as_euler("xyz")
        return np.hstack((p_m, euler_m))

    def make_marker(self, stamp, marker_id: int, pose: Pose) -> Marker:
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

    def publish_timer_callback(self) -> None:
        """Publish markers, current pose with covariance, and path."""
        now = self.get_clock().now().to_msg()

        # Publish landmarks as markers
        for lm_id, idx in self.landmarks.items():
            pos_l = self.state[idx : idx + 3]
            pose_msg = Pose()
            pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = pos_l
            marker = self.make_marker(now, lm_id, pose_msg)
            self.marker_pub.publish(marker)

        # Publish current robot pose with covariance
        pwc = PoseWithCovarianceStamped()
        pwc.header.stamp = now
        pwc.header.frame_id = "map"
        pwc.pose.pose = self.numpy_to_pose(self.state[:6])
        # flatten covariance (row-major) into 6x6
        cov6 = self.cov[:6, :6]
        pwc.pose.covariance = tuple(cov6.flatten().tolist())
        self.pose_pub.publish(pwc)

        # Append to path and publish Path
        ps = PoseStamped()
        ps.header = pwc.header
        ps.pose = pwc.pose.pose
        self.path_history.append(ps)
        path_msg = Path()
        path_msg.header = pwc.header
        path_msg.poses = list(self.path_history)
        self.path_pub.publish(path_msg)


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
