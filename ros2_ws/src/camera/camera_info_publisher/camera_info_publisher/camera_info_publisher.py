import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml


class CameraInfoPublisher(Node):
    """
    ROS2 Node to publish CameraInfo messages at a fixed rate.
    """

    def __init__(self):
        # Initialize the node with the name "camera_info_publisher"
        super().__init__("camera_info_publisher")

        # Declare and get parameters
        self.declare_parameter("camera_namespace", "/camera")
        self.declare_parameter("config_file_path", "default_path")
        self.declare_parameter("rate", 1.0)

        config_file_path = (
            self.get_parameter("config_file_path").get_parameter_value().string_value
        )
        rate = self.get_parameter("rate").get_parameter_value().double_value

        self.topic_name = (
            self.get_parameter("camera_namespace").get_parameter_value().string_value
            + "/camera_info"
        )

        # Create a publisher for the CameraInfo messages on the "/camera/camera_info" topic with a queue size of 10
        self.publisher_ = self.create_publisher(CameraInfo, self.topic_name, 10)

        # Set a timer to call the timer_callback function at the specified rate
        timer_period = 1.0 / rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load camera calibration data from a YAML file
        self.camera_info_msg = self.load_camera_info(config_file_path)

    def load_camera_info(self, config_file_path):
        """
        Load camera calibration data from a YAML file and populate a CameraInfo message.
        """
        # Open and read the YAML file

        file = None

        try:
            file = open(config_file_path, "r")
            self.get_logger().info("Loaded camera configuration file")
            self.node_ok = True
        except IOError:
            self.get_logger().error("Error loading camera configuration file")
            self.node_ok = False
            return

        calibration_data = yaml.safe_load(file)

        # Create a CameraInfo message and populate it with the calibration data
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calibration_data["image_width"]
        camera_info_msg.height = calibration_data["image_height"]
        camera_info_msg.k = calibration_data["camera_matrix"]["data"]
        camera_info_msg.d = calibration_data["distortion_coefficients"]["data"]
        camera_info_msg.r = calibration_data["rectification_matrix"]["data"]
        camera_info_msg.p = calibration_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calibration_data["distortion_model"]

        return camera_info_msg

    def timer_callback(self):
        """
        Timer callback function to publish the CameraInfo message.
        """
        # Publish the CameraInfo message
        self.publisher_.publish(self.camera_info_msg)
        # Log the publishing event


def main(args=None):
    """
    Main function to initialize the ROS2 system, create the node, and spin it.
    """
    # Initialize ROS2 with the given arguments
    rclpy.init(args=args)
    # Create an instance of the CameraInfoPublisher node
    node = CameraInfoPublisher()
    # Keep the node running until interrupted

    if hasattr(node, "node_ok"):
        if node.node_ok:
            rclpy.spin(node)
        else:
            node.destroy_node()


if __name__ == "__main__":
    # Execute the main function when the script is run
    main()
