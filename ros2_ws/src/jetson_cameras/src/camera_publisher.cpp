#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#define SENSOR_ID 0
#define CAPTURE_WIDTH 1920
#define CAPTURE_HEIGHT 1080
#define DISPLAY_WIDTH 960
#define DISPLAY_HEIGHT 540
#define FRAMERATE 30
#define FLIP_METHOD 0
#define FRAME_SIZE DISPLAY_WIDTH* DISPLAY_HEIGHT * 3

class CameraPublisher : public rclcpp::Node {
   public:
    CameraPublisher() : Node("camera_publisher"), frame_pub(nullptr) {
        frame_pub =
            this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/compressed", 10);

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

        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(30),
                                    std::bind(&CameraPublisher::image_publisher_callback, this));
    }

    ~CameraPublisher() { close(client_socket); }

   private:
    int client_socket;
    struct sockaddr_in server_addr;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr frame_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    void image_publisher_callback() {
        ssize_t msg_size = FRAME_SIZE;
        cv::Mat image(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3);

        ssize_t total_bytes_received = 0;
        ssize_t offset = 0;
        ssize_t bytes_received;

        while (total_bytes_received < msg_size) {
            bytes_received =
                recv(client_socket, image.data + offset, msg_size - total_bytes_received, 0);
            if (bytes_received <= 0) {
                std::cerr << "Connection closed or error" << std::endl;
                exit(1);
            }
            total_bytes_received += bytes_received;
            offset += bytes_received;
        }

        std::vector<uchar> buffer;
        cv::imencode(".jpg", image, buffer);

        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->format = "jpeg";  // Specify the format here
        msg->data.assign(buffer.begin(), buffer.end());

        frame_pub->publish(*msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
