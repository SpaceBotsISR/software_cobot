/*
 * Copyright 2024.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <sstream>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <zmq.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
// TF2
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

namespace pt = boost::property_tree;
using namespace std::chrono_literals;

namespace {
constexpr char kDefaultSensorEndpoint[] = "tcp://*:5556";
constexpr char kDefaultImageEndpoint[] = "tcp://*:5560";
constexpr char kDefaultCommandEndpoint[] = "tcp://*:5557";

std::string base64_encode(const uint8_t* data, size_t length) {
    static const char* base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    std::string encoded;
    encoded.reserve(((length + 2) / 3) * 4);

    size_t index = 0;
    while (length - index >= 3) {
        const uint32_t triple = (static_cast<uint32_t>(data[index]) << 16) |
                                (static_cast<uint32_t>(data[index + 1]) << 8) |
                                static_cast<uint32_t>(data[index + 2]);

        encoded.push_back(base64_chars[(triple >> 18) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 12) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 6) & 0x3F]);
        encoded.push_back(base64_chars[triple & 0x3F]);

        index += 3;
    }

    const size_t remaining = length - index;
    if (remaining == 1) {
        const uint32_t triple = static_cast<uint32_t>(data[index]) << 16;
        encoded.push_back(base64_chars[(triple >> 18) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 12) & 0x3F]);
        encoded.push_back('=');
        encoded.push_back('=');
    } else if (remaining == 2) {
        const uint32_t triple = (static_cast<uint32_t>(data[index]) << 16) |
                                (static_cast<uint32_t>(data[index + 1]) << 8);
        encoded.push_back(base64_chars[(triple >> 18) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 12) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 6) & 0x3F]);
        encoded.push_back('=');
    }

    return encoded;
}

template <typename T, size_t N>
pt::ptree array_to_ptree(const std::array<T, N>& array) {
    pt::ptree tree;
    for (const auto& value : array) {
        pt::ptree child;
        child.put("", value);
        tree.push_back(std::make_pair("", child));
    }
    return tree;
}

pt::ptree time_to_ptree(const builtin_interfaces::msg::Time& time_msg) {
    pt::ptree tree;
    tree.put("sec", time_msg.sec);
    tree.put("nanosec", time_msg.nanosec);
    return tree;
}

pt::ptree header_to_ptree(const std_msgs::msg::Header& header) {
    pt::ptree tree;
    tree.add_child("stamp", time_to_ptree(header.stamp));
    tree.put("frame_id", header.frame_id);
    return tree;
}

pt::ptree quaternion_to_ptree(const geometry_msgs::msg::Quaternion& quat) {
    pt::ptree tree;
    tree.put("x", quat.x);
    tree.put("y", quat.y);
    tree.put("z", quat.z);
    tree.put("w", quat.w);
    return tree;
}

pt::ptree vector3_to_ptree(const geometry_msgs::msg::Vector3& vec) {
    pt::ptree tree;
    tree.put("x", vec.x);
    tree.put("y", vec.y);
    tree.put("z", vec.z);
    return tree;
}

pt::ptree pose_to_ptree(const geometry_msgs::msg::Pose& pose) {
    pt::ptree tree;
    pt::ptree position;
    position.put("x", pose.position.x);
    position.put("y", pose.position.y);
    position.put("z", pose.position.z);
    tree.add_child("position", position);
    tree.add_child("orientation", quaternion_to_ptree(pose.orientation));
    return tree;
}

std::string ptree_to_compact_json(const pt::ptree& tree) {
    std::ostringstream oss;
    pt::write_json(oss, tree, false);
    std::string json = oss.str();
    if (!json.empty() && json.back() == '\n') {
        json.pop_back();
    }
    return json;
}

}  // namespace

class TeleopNode : public rclcpp::Node {
   public:
    TeleopNode()
        : Node("teleop_node"),
          context_(1),
          sensor_pub_(context_, ZMQ_PUB),
          image_pub_(context_, ZMQ_PUB),
          cmd_sub_(context_, ZMQ_SUB),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_) {
        declare_parameter<std::string>("teleop_bridge_sensor_endpoint", kDefaultSensorEndpoint);
        declare_parameter<std::string>("teleop_bridge_image_endpoint", kDefaultImageEndpoint);
        declare_parameter<std::string>("teleop_cmd_endpoint", kDefaultCommandEndpoint);
        declare_parameter<int>("jpeg_quality", 80);
        declare_parameter<std::string>("command_frame", "body");
        declare_parameter<std::string>("default_input_frame", "body");
        declare_parameter<bool>("allow_cmd_frame_override", true);

        const auto sensor_endpoint = get_parameter("teleop_bridge_sensor_endpoint").as_string();
        const auto image_endpoint = get_parameter("teleop_bridge_image_endpoint").as_string();
        const auto cmd_endpoint = get_parameter("teleop_cmd_endpoint").as_string();
        jpeg_quality_ = get_parameter("jpeg_quality").as_int();
        jpeg_quality_ = std::max(1, std::min(100, jpeg_quality_));

        configure_socket(sensor_pub_, sensor_endpoint, 100);
        configure_socket(image_pub_, image_endpoint, 3);
        configure_cmd_socket(cmd_sub_, cmd_endpoint);

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/space_cobot/cmd_vel", 10);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { handle_imu(msg); });

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/space_cobot/pose", 10,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { handle_pose(msg); });

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/main_camera/image", 10,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { handle_image(msg); });

        cmd_timer_ = create_wall_timer(10ms, [this]() { poll_command_bus(); });

        command_frame_ = get_parameter("command_frame").as_string();
        default_input_frame_ = get_parameter("default_input_frame").as_string();
        allow_cmd_frame_override_ = get_parameter("allow_cmd_frame_override").as_bool();

        RCLCPP_INFO(get_logger(),
                    "Teleop bridge PUB sensors on %s; images on %s; commands SUB on %s (cmd frame=%s, default input=%s)",
                    sensor_endpoint.c_str(), image_endpoint.c_str(), cmd_endpoint.c_str(),
                    command_frame_.c_str(), default_input_frame_.c_str());
    }

    ~TeleopNode() override {
        try {
            cmd_sub_.close();
            image_pub_.close();
            sensor_pub_.close();
            context_.close();
        } catch (const zmq::error_t& e) {
            RCLCPP_WARN(get_logger(), "Failed to close ZMQ sockets cleanly: %s", e.what());
        }
    }

   private:
    void configure_socket(zmq::socket_t& socket, const std::string& endpoint, int hwm) {
        socket.set(zmq::sockopt::linger, 0);
        socket.set(zmq::sockopt::sndhwm, hwm);
        socket.bind(endpoint);
    }

    void configure_cmd_socket(zmq::socket_t& socket, const std::string& endpoint) {
        socket.set(zmq::sockopt::linger, 0);
        socket.set(zmq::sockopt::rcvhwm, 1000);
        socket.set(zmq::sockopt::rcvtimeo, 1);
        socket.set(zmq::sockopt::subscribe, "");
        socket.bind(endpoint);
    }

    void publish_payload(const std::string& topic, const pt::ptree& payload, bool is_image) {
        pt::ptree envelope;
        envelope.put("topic", topic);
        envelope.add_child("data", payload);
        const auto message = ptree_to_compact_json(envelope);

        try {
            auto& socket = is_image ? image_pub_ : sensor_pub_;
            zmq::send_flags flags = zmq::send_flags::dontwait;
            socket.send(zmq::buffer(message), flags);
        } catch (const zmq::error_t& e) {
            RCLCPP_WARN(get_logger(), "Failed to publish teleop payload for topic %s: %s",
                        topic.c_str(), e.what());
        }
    }

    void handle_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        pt::ptree payload;
        payload.add_child("header", header_to_ptree(msg->header));
        payload.add_child("orientation", quaternion_to_ptree(msg->orientation));
        payload.add_child("orientation_covariance", array_to_ptree(msg->orientation_covariance));
        payload.add_child("angular_velocity", vector3_to_ptree(msg->angular_velocity));
        payload.add_child("angular_velocity_covariance",
                          array_to_ptree(msg->angular_velocity_covariance));
        payload.add_child("linear_acceleration", vector3_to_ptree(msg->linear_acceleration));
        payload.add_child("linear_acceleration_covariance",
                          array_to_ptree(msg->linear_acceleration_covariance));

        publish_payload("/imu/data", payload, false);
    }

    void handle_pose(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        pt::ptree payload;
        payload.add_child("header", header_to_ptree(msg->header));
        payload.add_child("pose", pose_to_ptree(msg->pose));
        publish_payload("/space_cobot/pose", payload, false);
    }

    void handle_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        auto payload = pt::ptree{};
        payload.put("width", msg->width);
        payload.put("height", msg->height);
        payload.put("step", msg->step);
        payload.put("is_bigendian", msg->is_bigendian);

        const auto& encoding = msg->encoding;
        try {
            const auto jpeg = encode_to_jpeg(msg);
            payload.put("encoding", "jpeg");
            payload.put("data", base64_encode(jpeg.data(), jpeg.size()));
        } catch (const std::exception& e) {
            payload.put("encoding", encoding);
            payload.put("data", base64_encode(msg->data.data(), msg->data.size()));
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Falling back to raw image payload: %s", e.what());
        }

        publish_payload("/main_camera/image", payload, true);
    }

    std::vector<uint8_t> encode_to_jpeg(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        if (!msg || msg->data.empty()) {
            throw std::runtime_error("empty image buffer");
        }

        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        cv::Mat converted;

        if (msg->encoding == "bgr8") {
            converted = cv_ptr->image;
        } else if (msg->encoding == "rgb8") {
            cv::cvtColor(cv_ptr->image, converted, cv::COLOR_RGB2BGR);
        } else if (msg->encoding == "rgba8") {
            cv::cvtColor(cv_ptr->image, converted, cv::COLOR_RGBA2BGR);
        } else if (msg->encoding == "bgra8") {
            cv::cvtColor(cv_ptr->image, converted, cv::COLOR_BGRA2BGR);
        } else if (msg->encoding == "mono8") {
            converted = cv_ptr->image;
        } else {
            throw std::runtime_error("unsupported encoding: " + msg->encoding);
        }

        std::vector<uint8_t> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        if (!cv::imencode(".jpg", converted, buffer, params)) {
            throw std::runtime_error("cv::imencode failed");
        }
        return buffer;
    }

    void poll_command_bus() {
        while (rclcpp::ok()) {
            zmq::message_t message;
            auto result = cmd_sub_.recv(message, zmq::recv_flags::dontwait);
            if (!result) {
                break;
            }

            const std::string payload(static_cast<const char*>(message.data()), message.size());
            process_command(payload);
        }
    }

    void process_command(const std::string& payload) {
        try {
            std::istringstream iss(payload);
            pt::ptree root;
            pt::read_json(iss, root);
            const auto topic = root.get<std::string>("topic", "");
            if (topic != "/space_cobot/cmd_vel") {
                return;
            }

            const auto data = root.get_child_optional("data");
            if (!data) {
                return;
            }

            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = data->get<double>("linear.x", 0.0);
            twist.linear.y = data->get<double>("linear.y", 0.0);
            twist.linear.z = data->get<double>("linear.z", 0.0);
            twist.angular.x = data->get<double>("angular.x", 0.0);
            twist.angular.y = data->get<double>("angular.y", 0.0);
            twist.angular.z = data->get<double>("angular.z", 0.0);

            // Determine input frame: optional override from JSON, else default param.
            std::string input_frame = default_input_frame_;
            if (allow_cmd_frame_override_) {
                // Check at root level or inside data
                const auto root_frame = root.get_optional<std::string>("frame_id");
                const auto data_frame = data->get_optional<std::string>("frame_id");
                if (root_frame && !root_frame->empty()) {
                    input_frame = *root_frame;
                } else if (data_frame && !data_frame->empty()) {
                    input_frame = *data_frame;
                }
            }

            if (!command_frame_.empty() && !input_frame.empty() && input_frame != command_frame_) {
                // Rotate vectors from input_frame -> command_frame using TF
                const auto now_ros = this->get_clock()->now();
                try {
                    const auto tf = tf_buffer_.lookupTransform(
                        command_frame_, input_frame, tf2::TimePointZero);

                    geometry_msgs::msg::Vector3Stamped lin_in, lin_out;
                    lin_in.header.frame_id = input_frame;
                    lin_in.header.stamp = now_ros;
                    lin_in.vector = twist.linear;
                    tf2::doTransform(lin_in, lin_out, tf);
                    twist.linear = lin_out.vector;

                    geometry_msgs::msg::Vector3Stamped ang_in, ang_out;
                    ang_in.header.frame_id = input_frame;
                    ang_in.header.stamp = now_ros;
                    ang_in.vector = twist.angular;
                    tf2::doTransform(ang_in, ang_out, tf);
                    twist.angular = ang_out.vector;
                } catch (const tf2::TransformException& ex) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "cmd_vel transform %s->%s missing: %s (publishing untransformed)",
                                         input_frame.c_str(), command_frame_.c_str(), ex.what());
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (last_cmd_time_) {
                const auto dt = std::chrono::duration<double>(now - *last_cmd_time_).count();
                if (twist.linear.x > 0.0) {
                    RCLCPP_INFO(get_logger(), "Received /space_cobot/cmd_vel (dt=%.3fs)", dt);
                }
            }
            last_cmd_time_ = now;
            cmd_vel_pub_->publish(twist);
        } catch (const pt::json_parser_error& e) {
            RCLCPP_WARN(get_logger(), "Failed to parse teleop command JSON: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to process teleop command: %s", e.what());
        }
    }

    zmq::context_t context_;
    zmq::socket_t sensor_pub_;
    zmq::socket_t image_pub_;
    zmq::socket_t cmd_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;

    std::optional<std::chrono::steady_clock::time_point> last_cmd_time_;
    int jpeg_quality_{80};

    // Frames and TF
    std::string command_frame_;
    std::string default_input_frame_;
    bool allow_cmd_frame_override_{true};
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
