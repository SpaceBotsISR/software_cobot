import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from aruco_opencv_msgs.msg import ArucoDetection

# GTSAM imports
import gtsam
from gtsam import (
    ISAM2,
    NonlinearFactorGraph,
    Values,
    PriorFactorPose3,
    BetweenFactorPose3,
    symbol,
)
from gtsam import noiseModel


class GtsamNode(Node):
    def __init__(self) -> None:
        super().__init__("gtsam_node")

        # ISAM2 setup
        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(0.01)
        self.isam = ISAM2(params)
        self.graph = NonlinearFactorGraph()
        self.initial = Values()

        # Noise models
        self.prior_noise = noiseModel.Diagonal.Variances([1e-6] * 6)
        self.odom_noise = noiseModel.Diagonal.Variances([0.1] * 6)
        self.aruco_noise = noiseModel.Diagonal.Variances([0.05] * 6)

        # Track step and keys
        self.step = 0
        self.pose_keys = []
        self.marker_keys = {}

        # Subscriptions
        self.create_subscription(
            PoseStamped,
            "/zed/zed_node/pose",
            self.odom_callback,
            10,
        )
        self.create_subscription(
            ArucoDetection,
            "/aruco_detections",
            self.aruco_callback,
            10,
        )

        # Publisher
        self.pub = self.create_publisher(PoseStamped, "/gtsam/pose", 10)

    def odom_callback(self, msg: PoseStamped) -> None:
        key = symbol("x", self.step)
        # Convert ROS PoseStamped to GTSAM Pose3
        p = msg.pose.position
        o = msg.pose.orientation
        pose3 = gtsam.Pose3(
            gtsam.Rot3.Quaternion(o.w, o.x, o.y, o.z),
            gtsam.Point3(p.x, p.y, p.z),
        )

        if self.step == 0:
            # Anchor origin with a prior
            self.graph.add(PriorFactorPose3(key, pose3, self.prior_noise))
        else:
            prev_key = symbol("x", self.step - 1)
            # Get the last optimized pose from ISAM2
            prev_est = self.isam.calculateEstimate().atPose3(prev_key)
            rel = prev_est.between(pose3)
            self.graph.add(BetweenFactorPose3(prev_key, key, rel, self.odom_noise))

        # Insert this pose as an initial estimate
        self.initial.insert(key, pose3)
        self.pose_keys.append(key)
        self.step += 1

        # Update ISAM2 and publish fused pose
        self.update_and_publish(key, msg.header)

    def aruco_callback(self, msg: ArucoDetection) -> None:
        # Use latest robot pose key
        key = symbol("x", self.step - 1)

        # Loop through all detected markers
        for marker in msg.markers:
            marker_id = marker.marker_id
            # Extract marker pose in camera frame
            pm = marker.pose.position
            om = marker.pose.orientation

            # Initialize landmark if seen first time
            if marker_id not in self.marker_keys:
                lkey = symbol("l", marker_id)
                self.marker_keys[marker_id] = lkey
                # Estimate initial marker pose from current camera pose
                cam_est = (
                    self.isam.calculateEstimate().atPose3(key)
                    if self.step > 0
                    else gtsam.Pose3()
                )
                meas = gtsam.Pose3(
                    gtsam.Rot3.Quaternion(om.w, om.x, om.y, om.z),
                    gtsam.Point3(pm.x, pm.y, pm.z),
                )
                init_marker = cam_est.compose(meas)
                self.initial.insert(lkey, init_marker)

            # Add measurement factor between pose and marker
            lkey = self.marker_keys[marker_id]
            meas = gtsam.Pose3(
                gtsam.Rot3.Quaternion(om.w, om.x, om.y, om.z),
                gtsam.Point3(pm.x, pm.y, pm.z),
            )
            self.graph.add(BetweenFactorPose3(key, lkey, meas, self.aruco_noise))

        # After factors, update ISAM2 and publish
        self.update_and_publish(key, msg.header)

    def update_and_publish(self, key, header) -> None:
        # Run the ISAM2 update
        self.isam.update(self.graph, self.initial)
        est = self.isam.calculateEstimate().atPose3(key)

        # Clear the temporary graph and estimates
        self.graph.resize(0)
        self.initial.clear()

        # Build the output PoseStamped
        out = PoseStamped()
        out.header = header

        # Extract translation array and quaternion
        t = est.translation()  # numpy.ndarray [x, y, z]
        q = est.rotation().toQuaternion()  # GTSAM Quaternion

        # Fill position fields
        out.pose.position.x = float(t[0])
        out.pose.position.y = float(t[1])
        out.pose.position.z = float(t[2])

        # Fill orientation fields
        out.pose.orientation.x = float(q.x())
        out.pose.orientation.y = float(q.y())
        out.pose.orientation.z = float(q.z())
        out.pose.orientation.w = float(q.w())

        # Publish fused pose
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GtsamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
