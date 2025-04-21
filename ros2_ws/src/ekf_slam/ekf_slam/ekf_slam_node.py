import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from aruco_opencv_msgs.msg import ArucoDetection

class EkfSlamNode(Node):
    def __init__(self) -> None:
        super().__init__('ekf_slam_node')
        
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_slam/pose', 10)
        self.zed_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/zed/zed_node/pose_with_covariance',
            self.zed_pose_callback,
            10
        )
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            10
        )
        

    def zed_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        return
        self.get_logger().info(f'Received ZED pose: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}')
    
    def aruco_callback(self, msg: ArucoDetection) -> None:
        for marker in msg.markers:
            self.get_logger().info(f'Detected marker ID: {marker.marker_id}, position: {marker.pose.position.x}, {marker.pose.position.y}, {marker.pose.position.z}')
        



def main(args=None) -> None:
    rclpy.init(args=args)

    node = EkfSlamNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()