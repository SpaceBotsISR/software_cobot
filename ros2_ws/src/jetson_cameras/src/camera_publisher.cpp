#include <arpa/inet.h>
<<<<<<< HEAD
=======
#include <cv_bridge/cv_bridge.h>
>>>>>>> camera ros2
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

<<<<<<< HEAD
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

=======
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#define FRAME_SIZE 1555200
>>>>>>> camera ros2
#define SENSOR_ID 0
#define CAPTURE_WIDTH 1920
#define CAPTURE_HEIGHT 1080
#define DISPLAY_WIDTH 960
#define DISPLAY_HEIGHT 540
#define FRAMERATE 30
#define FLIP_METHOD 0
<<<<<<< HEAD
#define FRAME_SIZE DISPLAY_WIDTH* DISPLAY_HEIGHT * 3

class CameraPublisher : public rclcpp::Node {
   public:
    CameraPublisher() : Node("camera_publisher"), frame_pub(nullptr) {
        frame_pub =
            this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/compressed", 10);
=======

class CameraPublisher : public rclcpp::Node {
   public:
    CameraPublisher() : Node("video_receiver"), frame_pub(nullptr) {
        frame_pub = this->create_publisher<sensor_msgs::msg::Image>("video_frame", 10);
>>>>>>> camera ros2

        // Socket setup
        client_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (client_socket < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error creating socket!");
            exit(1);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(8080);
        server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

        if (connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error connecting to server!");
            exit(1);
        }

        RCLCPP_INFO(this->get_logger(), "Connected to server.");

<<<<<<< HEAD
        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(30),
                                    std::bind(&CameraPublisher::image_publisher_callback, this));
=======
        // Use a timer to control the rate of frame reading
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / FRAMERATE),
            std::bind(&CameraPublisher::read_and_publish, this, FRAME_SIZE));
>>>>>>> camera ros2
    }

    ~CameraPublisher() { close(client_socket); }

   private:
    int client_socket;
    struct sockaddr_in server_addr;
<<<<<<< HEAD
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr frame_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    void image_publisher_callback() {
        ssize_t msg_size = FRAME_SIZE;
        cv::Mat image(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3);
=======
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    void read_and_publish(ssize_t msg_size) {
        cv::Mat frame(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3);
        char* buffer = new char[msg_size];
>>>>>>> camera ros2

        ssize_t total_bytes_received = 0;
        ssize_t offset = 0;
        ssize_t bytes_received;

        while (total_bytes_received < msg_size) {
            bytes_received =
<<<<<<< HEAD
                recv(client_socket, image.data + offset, msg_size - total_bytes_received, 0);
=======
                recv(client_socket, buffer + offset, msg_size - total_bytes_received, 0);

>>>>>>> camera ros2
            if (bytes_received <= 0) {
                std::cerr << "Connection closed or error" << std::endl;
                exit(1);
            }
<<<<<<< HEAD
            total_bytes_received += bytes_received;
            offset += bytes_received;
        }

        std::vector<uchar> buffer;
        cv::imencode(".jpg", image, buffer);

        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->format = "jpeg";  // Specify the format here
        msg->data.assign(buffer.begin(), buffer.end());

        frame_pub->publish(*msg);
=======

            total_bytes_received += bytes_received;
            offset += bytes_received;  // Update the offset by the number of bytes received
        }
        std::memcpy(frame.data, buffer, total_bytes_received);

        // Convert the cv::Mat to a sensor_msgs::Image and publish it
        sensor_msgs::msg::Image::SharedPtr img_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        frame_pub->publish(*img_msg);

        delete[] buffer;
>>>>>>> camera ros2
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
