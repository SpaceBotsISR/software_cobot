import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.publisher_ = self.create_publisher(Image, "camera/image_raw", 10)
        self.timer = self.create_timer(1 / 30.0, self.timer_callback)  # 30Hz
        self.cap = cv2.VideoCapture(
            2
        )  # Change the argument if you want to use a different camera

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 26)

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert the frame to a ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()

            # Publish the raw image
            self.publisher_.publish(msg)

    def __del__(self):
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
