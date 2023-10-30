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
#include <sensor_msgs/msg/camera_info.hpp>


#define CAPTURE_WIDTH 1920
#define CAPTURE_HEIGHT 1080
#define DISPLAY_WIDTH 960
#define DISPLAY_HEIGHT 540
#define FRAMERATE 30
#define FLIP_METHOD 0
#define FRAME_SIZE DISPLAY_WIDTH* DISPLAY_HEIGHT * 3

class CameraPublisher : public rclcpp::Node {
   public:
    CameraPublisher() : Node("camera_publisher") {
        init_parameters();
        init_publishers();
        init_socket();
        init_timers();
    }

    ~CameraPublisher() { close(client_socket); }

   private:
    int client_socket;
    struct sockaddr_in server_addr;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_frame_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_frame_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    int camera_id;

    void init_parameters() {
        this->declare_parameter("camera_id", 0);
        this->get_parameter("camera_id", camera_id);
    }

    void init_publishers() {
        std::string compressed_topic_name =
            "/camera_" + std::to_string(camera_id) + "/image_raw/compressed";
        compressed_frame_pub =
            this->create_publisher<sensor_msgs::msg::CompressedImage>(compressed_topic_name, 10);

        std::string raw_topic_name = "/camera_" + std::to_string(camera_id) + "/image_raw";
        raw_frame_pub = this->create_publisher<sensor_msgs::msg::Image>(raw_topic_name, 10);

        std::string camera_info_topic_name =
            "/camera_" + std::to_string(camera_id) + "/camera_info";
        camera_info_pub =
            this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_name, 1);
    }

    void init_socket() {
        int port = camera_id + 8080;

        client_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (client_socket < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error creating socket!");
            exit(1);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

        if (connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error connecting to server!");
            exit(1);
        }

        RCLCPP_INFO(this->get_logger(), "Connected to server.");
    }

    void init_timers() {
        timer1_ =
            this->create_wall_timer(std::chrono::milliseconds(30),
                                    std::bind(&CameraPublisher::image_publisher_callback, this));

        timer2_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&CameraPublisher::camera_info_publisher_callback, this));
    }
    void camera_info_publisher_callback() {}

    void image_publisher_callback() {
        cv::Mat image = receive_image_from_socket();
        publish_raw_image(image);
        publish_compressed_image(image);
    }

    cv::Mat receive_image_from_socket() {
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

        return image;
    }

    void publish_raw_image(const cv::Mat& image) {
        auto raw_msg = std::make_unique<sensor_msgs::msg::Image>();
        raw_msg->header.stamp = this->now();
        raw_msg->height = image.rows;
        raw_msg->width = image.cols;
        raw_msg->encoding = "bgr8";
        raw_msg->step = image.cols * image.elemSize();
        size_t size = image.rows * image.step;
        raw_msg->data.resize(size);
        std::memcpy(&raw_msg->data[0], image.data, size);
        raw_frame_pub->publish(*raw_msg);
    }

    void publish_compressed_image(const cv::Mat& image) {
        std::vector<uchar> buffer;
        cv::imencode(".jpg", image, buffer);

        auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        compressed_msg->format = "jpeg";
        compressed_msg->data.assign(buffer.begin(), buffer.end());
        compressed_frame_pub->publish(*compressed_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
