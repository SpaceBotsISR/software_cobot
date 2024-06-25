#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher"), cap_(2)
    {
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        cap_.set(cv::CAP_PROP_FPS, 60);

        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/image_raw/compressed", 10);
        timer_ = this->create_wall_timer(33ms, std::bind(&CameraPublisher::timer_callback, this));

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
            rclcpp::shutdown();
        }
    }

private:
    void timer_callback()
    {
        static auto last_time = std::chrono::steady_clock::now();
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame");
            return;
        }

        std::vector<uchar> buffer;
        cv::imencode(".jpg", frame, buffer);

        auto now_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now_time - last_time;
        last_time = now_time;
        RCLCPP_INFO(this->get_logger(), "Capture frame rate: %.2f Hz", 1.0 / elapsed.count());

        auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->now();
        msg->format = "jpeg";
        msg->data = buffer;

        publisher_->publish(*msg);

        static auto last_pub_time = this->now();
        auto now_pub_time = this->now();
        auto elapsed_pub_time = now_pub_time - last_pub_time;
        last_pub_time = now_pub_time;
        RCLCPP_INFO(this->get_logger(), "Publishing rate: %.2f Hz", 1.0 / elapsed_pub_time.seconds());
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
