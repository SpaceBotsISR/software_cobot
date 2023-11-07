#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

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
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_;
    int camera_id;

    void init_parameters() {
        // Camera id parameters created in the launchfile 
        this->declare_parameter("camera_id", 0);
        this->get_parameter("camera_id", camera_id);


        // Declaring params
        std::string param_name = "camera_publisher" + std::to_string(camera_id) + ".";
        camera_info_msg_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
        
        this->declare_parameter("image.width", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("image.height", rclcpp::PARAMETER_INTEGER);

        this->declare_parameter("camera_matrix.data", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("distortion_model", rclcpp::PARAMETER_STRING);
        this->declare_parameter("distortion_coefficients.data", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("rectification_matrix.data", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("projection_matrix.data", rclcpp::PARAMETER_DOUBLE_ARRAY);


        // Getting params from yaml
        int width, height;
        this->get_parameter("image.width", width);
        this->get_parameter("image.height", height);
        camera_info_msg_->width = width;
        camera_info_msg_->height = height;

        std::string distortion_model;
        std::vector<double> camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix;
        this->get_parameter("camera_matrix.data", camera_matrix);
        this->get_parameter("distortion_model", distortion_model);
        this->get_parameter("distortion_coefficients.data", distortion_coefficients);
        this->get_parameter("rectification_matrix.data", rectification_matrix);
        this->get_parameter("projection_matrix.data", projection_matrix);

        // Copying the params to the info msg
        camera_info_msg_->distortion_model = distortion_model;
        for(auto v : distortion_coefficients){
            camera_info_msg_->d.push_back(v);
        }
        std::copy(camera_matrix.begin(), camera_matrix.end(), camera_info_msg_->k.begin());
        std::copy(rectification_matrix.begin(), rectification_matrix.end(), camera_info_msg_->r.begin());
        std::copy(projection_matrix.begin(), projection_matrix.end(), camera_info_msg_->p.begin());
    }

    void init_publishers() {
        // Quality of service change
        // auto sensor_qos = rclcpp::SensorDataQoS();
        // sensor_qos.keep_last(5);

        std::string compressed_topic_name = 
            "/camera_" + std::to_string(camera_id) + "/image_raw/compressed";
        compressed_frame_pub =
            this->create_publisher<sensor_msgs::msg::CompressedImage>(compressed_topic_name, 5);

        std::string raw_topic_name = "/camera_" + std::to_string(camera_id) + "/image_raw";
        raw_frame_pub = this->create_publisher<sensor_msgs::msg::Image>(raw_topic_name, 5);

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
            std::chrono::milliseconds(500),
            std::bind(&CameraPublisher::camera_info_publisher_callback, this));
    }
    void camera_info_publisher_callback() {
        camera_info_msg_->header.stamp = this->now();
        camera_info_msg_->header.frame_id = "camera_frame_" + std::to_string(camera_id);
        camera_info_pub->publish(*camera_info_msg_);
    }

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
        raw_msg->header.frame_id = "camera_frame_" + std::to_string(camera_id);
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
        compressed_msg->header.stamp = this->now();
        compressed_msg->header.frame_id = "camera_frame_" + std::to_string(camera_id);
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
