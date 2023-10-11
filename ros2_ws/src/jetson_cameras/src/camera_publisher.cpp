#include <arpa/inet.h>
// #include <cv_bridge/cv_bridge.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// #include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#define FRAME_SIZE 1555200
#define SENSOR_ID 0
#define CAPTURE_WIDTH 1920
#define CAPTURE_HEIGHT 1080
#define DISPLAY_WIDTH 960
#define DISPLAY_HEIGHT 540
#define FRAMERATE 30
#define FLIP_METHOD 0

class CameraPublisher : public rclcpp::Node {
   public:
    CameraPublisher() : Node("video_receiver"), frame_pub(nullptr) {
        frame_pub = this->create_publisher<sensor_msgs::msg::Image>("video_frame", 10);

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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    void image_publisher_callback() {
        ssize_t msg_size = FRAME_SIZE;
        auto msg = std::make_unique<sensor_msgs::msg::Image>();

        // The image is in BGR8 format
        msg->encoding = "bgr8";
        msg->width = DISPLAY_WIDTH;    // Use defined constants for width
        msg->height = DISPLAY_HEIGHT;  // Use defined constants for height
        msg->step = msg->width * 3;    // 3 bytes per pixel for BGR8

        // Resize data to fit our image content
        msg->data.resize(msg_size);

        ssize_t total_bytes_received = 0;
        ssize_t offset = 0;
        ssize_t bytes_received;

        while (total_bytes_received < msg_size) {
            bytes_received =
                recv(client_socket, msg->data.data() + offset, msg_size - total_bytes_received, 0);
            if (bytes_received <= 0) {
                std::cerr << "Connection closed or error" << std::endl;
                exit(1);
            }
            total_bytes_received += bytes_received;
            offset += bytes_received;
        }

        frame_pub->publish(*msg);  // Use frame_pub here
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
