import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from aruco_opencv_msgs.msg import ArucoDetection
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R
from stl import mesh


class RobotOverlayNode(Node):

    def __init__(self):
        super().__init__('robot_overlay_node')
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.camera_info_subscriber = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.aruco_subscriber = self.create_subscription(ArucoDetection, '/aruco_detections', self.aruco_callback, 10)
        self.image_publisher = self.create_publisher(Image, '/overlay_image', 10)

        self.follow_ids = [18, 11]
        self.bridge = CvBridge()
        self.current_poses = {18: None, 11: None}
        self.last_known_poses = {18: None, 11: None}

        self.camera_matrix = None
        self.dist_coeffs = None

    def aruco_callback(self, msg): 
        for marker in msg.markers: 
            if marker.marker_id in self.follow_ids:
                self.current_poses[marker.marker_id] = marker.pose
                self.last_known_poses[marker.marker_id] = marker.pose

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn('Camera calibration data is not available yet.')
            return

        # Check if any Aruco marker is detected
        detected_ids = [marker_id for marker_id, pose in self.current_poses.items() if pose is not None]
        
        if not detected_ids:
            self.get_logger().warn('No Aruco markers detected.')
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = frame.shape

            if len(detected_ids) == 1:
                return
            
            pos1 = self.current_poses[18].position
            pos2 = self.current_poses[11].position
            avg_position = np.array([(pos1.x + pos2.x) / 2, (pos1.y + pos2.y) / 2, (pos1.z + pos2.z) / 2])
            print(pos1)
            print(pos2)
            print(avg_position)

            print('-----')

            ori1 = self.current_poses[18].orientation
            ori2 = self.current_poses[11].orientation
            avg_orientation = np.array([
                (ori1.w + ori2.w) / 2, 
                (ori1.x + ori2.x) / 2, 
                (ori1.y + ori2.y) / 2, 
                (ori1.z + ori2.z) / 2
            ])

            rvec, _ = cv2.Rodrigues(self.quaternion_to_rotation_matrix(avg_orientation))
            tvec = avg_position.reshape((3, 1))

            # Define the axes length
            axis_length = 0.1  # Adjust as needed

            # Define the points for the axes
            axes_points = np.float32([
                [0, 0, 0], 
                [axis_length, 0, 0], 
                [0, axis_length, 0], 
                [0, 0, axis_length]
            ]).reshape(-1, 3)

            # Project the 3D points to the 2D image plane
            img_points, _ = cv2.projectPoints(axes_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)

            # Draw the axes on the image
            origin = tuple(img_points[0].ravel().astype(int))
            x_axis = tuple(img_points[1].ravel().astype(int))
            y_axis = tuple(img_points[2].ravel().astype(int))
            z_axis = tuple(img_points[3].ravel().astype(int))

            cv2.line(frame, origin, x_axis, (0, 0, 255), 2)  # X axis in red
            cv2.line(frame, origin, y_axis, (0, 255, 0), 2)  # Y axis in green
            cv2.line(frame, origin, z_axis, (255, 0, 0), 2)  # Z axis in blue

            overlay_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_publisher.publish(overlay_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def quaternion_to_rotation_matrix(self, quaternion):
        w, x, y, z = quaternion
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])

def main(args=None):
    rclpy.init(args=args)
    node = RobotOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
