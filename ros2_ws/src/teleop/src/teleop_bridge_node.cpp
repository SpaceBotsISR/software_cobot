/*
 * Copyright 2024.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <sstream>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav6d/msg/path_execution_summary.hpp>
#include <nav6d/msg/path_quality.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <zmq.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
// TF2

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

builtin_interfaces::msg::Time ptree_to_time(const pt::ptree& time_tree) {
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = time_tree.get<int32_t>("sec", 0);
    time_msg.nanosec = time_tree.get<uint32_t>("nanosec", 0);
    return time_msg;
}

std_msgs::msg::Header ptree_to_header(const pt::ptree& header_tree) {
    std_msgs::msg::Header header;
    if (const auto stamp_tree = header_tree.get_child_optional("stamp")) {
        header.stamp = ptree_to_time(*stamp_tree);
    }
    header.frame_id = header_tree.get<std::string>("frame_id", header.frame_id);
    return header;
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

pt::ptree pose_stamped_to_ptree(const geometry_msgs::msg::PoseStamped& pose_stamped) {
    pt::ptree tree;
    tree.add_child("header", header_to_ptree(pose_stamped.header));
    tree.add_child("pose", pose_to_ptree(pose_stamped.pose));
    return tree;
}

pt::ptree path_to_ptree(const nav_msgs::msg::Path& path_msg) {
    pt::ptree tree;
    tree.add_child("header", header_to_ptree(path_msg.header));
    pt::ptree poses_tree;
    for (const auto& pose : path_msg.poses) {
        poses_tree.push_back(std::make_pair("", pose_stamped_to_ptree(pose)));
    }
    tree.add_child("poses", poses_tree);
    return tree;
}

pt::ptree path_quality_to_ptree(const nav6d::msg::PathQuality& msg) {
    pt::ptree tree;
    tree.put("clearance_score", msg.clearance_score);
    tree.put("narrow_score", msg.narrow_score);
    tree.put("turn_score", msg.turn_score);
    tree.put("efficiency_score", msg.efficiency_score);
    tree.put("heuristic", msg.heuristic);
    return tree;
}

pt::ptree execution_summary_to_ptree(const nav6d::msg::PathExecutionSummary& msg) {
    pt::ptree tree;
    tree.put("completed", msg.completed);
    tree.put("planned_length", msg.planned_length);
    tree.put("executed_length", msg.executed_length);
    tree.put("rms_tracking_error", msg.rms_tracking_error);
    return tree;
}

geometry_msgs::msg::Pose ptree_to_pose(const pt::ptree& pose_tree) {
    const pt::ptree* source = &pose_tree;
    if (auto nested = pose_tree.get_child_optional("pose")) {
        source = &nested.get();
    }

    auto get_coord = [source](const std::string& key, const std::string& alt_key,
                              double default_value) {
        if (const auto first = source->get_optional<double>(key)) {
            return *first;
        }
        if (!alt_key.empty()) {
            if (const auto second = source->get_optional<double>(alt_key)) {
                return *second;
            }
        }
        return default_value;
    };

    geometry_msgs::msg::Pose pose;
    pose.position.x = get_coord("position.x", "x", 0.0);
    pose.position.y = get_coord("position.y", "y", 0.0);
    pose.position.z = get_coord("position.z", "z", 0.0);
    pose.orientation.x = get_coord("orientation.x", "qx", 0.0);
    pose.orientation.y = get_coord("orientation.y", "qy", 0.0);
    pose.orientation.z = get_coord("orientation.z", "qz", 0.0);
    pose.orientation.w = get_coord("orientation.w", "qw", 1.0);
    return pose;
}

geometry_msgs::msg::PoseStamped ptree_to_pose_stamped(const pt::ptree& pose_tree) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    if (const auto header_tree = pose_tree.get_child_optional("header")) {
        pose_stamped.header = ptree_to_header(*header_tree);
    }
    pose_stamped.pose = ptree_to_pose(pose_tree);
    return pose_stamped;
}

nav_msgs::msg::Path ptree_to_path(const pt::ptree& path_tree) {
    nav_msgs::msg::Path path_msg;

    if (const auto header_tree = path_tree.get_child_optional("header")) {
        path_msg.header = ptree_to_header(*header_tree);
    }
    path_msg.header.frame_id = path_tree.get<std::string>("frame_id", path_msg.header.frame_id);
    if (path_msg.header.frame_id.empty()) {
        path_msg.header.frame_id = "world";
    }

    if (const auto poses_tree = path_tree.get_child_optional("poses")) {
        for (const auto& pose_node : *poses_tree) {
            path_msg.poses.push_back(ptree_to_pose_stamped(pose_node.second));
        }
    }
    return path_msg;
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
          cmd_sub_(context_, ZMQ_SUB) {
        declare_parameter<std::string>("teleop_bridge_sensor_endpoint", kDefaultSensorEndpoint);
        declare_parameter<std::string>("teleop_bridge_image_endpoint", kDefaultImageEndpoint);
        declare_parameter<std::string>("teleop_cmd_endpoint", kDefaultCommandEndpoint);
        declare_parameter<int>("jpeg_quality", 80);
        declare_parameter<std::string>("command_frame", "body");
        declare_parameter<std::string>("default_input_frame", "body");
        // Teleop inputs are always interpreted in the default frame.
        declare_parameter<bool>("allow_cmd_frame_override", false);
        declare_parameter<int>("one_way_delay_ms", 1500);

        const auto sensor_endpoint = get_parameter("teleop_bridge_sensor_endpoint").as_string();
        const auto image_endpoint = get_parameter("teleop_bridge_image_endpoint").as_string();
        const auto cmd_endpoint = get_parameter("teleop_cmd_endpoint").as_string();
        jpeg_quality_ = get_parameter("jpeg_quality").as_int();
        jpeg_quality_ = std::max(1, std::min(100, jpeg_quality_));
        const int delay_ms =
            std::max<int64_t>(0, get_parameter("one_way_delay_ms").as_int());
        one_way_delay_ = std::chrono::milliseconds(delay_ms);

        configure_socket(sensor_pub_, sensor_endpoint, 100);
        configure_socket(image_pub_, image_endpoint, 3);
        configure_cmd_socket(cmd_sub_, cmd_endpoint);

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/space_cobot/cmd_vel", 10);
        nav_path_pub_ = create_publisher<nav_msgs::msg::Path>("/nav6d/planner/path", 10);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { handle_imu(msg); });

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/space_cobot/pose", 10,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { handle_pose(msg); });

        nav_path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/nav6d/planner/path", 10,
            [this](nav_msgs::msg::Path::ConstSharedPtr msg) { handle_nav6d_path(msg); });

        path_quality_sub_ = create_subscription<nav6d::msg::PathQuality>(
            "/nav6d/path_quality", 10,
            [this](nav6d::msg::PathQuality::ConstSharedPtr msg) { handle_path_quality(msg); });

        velocity_summary_sub_ = create_subscription<nav6d::msg::PathExecutionSummary>(
            "/nav6d/velocity_controller/path_execution_summary", 10,
            [this](nav6d::msg::PathExecutionSummary::ConstSharedPtr msg) {
                handle_execution_summary("/nav6d/velocity_controller/path_execution_summary", msg);
            });

        force_summary_sub_ = create_subscription<nav6d::msg::PathExecutionSummary>(
            "/nav6d/force_controller/path_execution_summary", 10,
            [this](nav6d::msg::PathExecutionSummary::ConstSharedPtr msg) {
                handle_execution_summary("/nav6d/force_controller/path_execution_summary", msg);
            });

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/space_cobot/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { handle_cmd_vel(msg); });

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/main_camera/image", 10,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { handle_image(msg); });

        cmd_timer_ = create_wall_timer(10ms, [this]() {
            poll_command_bus();
            flush_delayed_commands();
            flush_delayed_payloads();
        });
        nav_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/nav6d/goal", 10);

        command_frame_ = get_parameter("command_frame").as_string();
        default_input_frame_ = get_parameter("default_input_frame").as_string();
        allow_cmd_frame_override_ = false;

        RCLCPP_INFO(get_logger(),
                    "Teleop bridge PUB sensors on %s; images on %s; commands SUB on %s (cmd frame=%s, default input=%s, one-way delay=%dms)",
                    sensor_endpoint.c_str(), image_endpoint.c_str(), cmd_endpoint.c_str(),
                    command_frame_.c_str(), default_input_frame_.c_str(), delay_ms);
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

        if (one_way_delay_.count() <= 0) {
            send_payload_message(topic, message, is_image);
            return;
        }
        const auto ready_at = std::chrono::steady_clock::now() + one_way_delay_;
        queued_payloads_.push_back({ready_at, topic, message, is_image});
    }

    void send_payload_message(const std::string& topic, const std::string& message,
                              bool is_image) {
        try {
            auto& socket = is_image ? image_pub_ : sensor_pub_;
            zmq::send_flags flags = zmq::send_flags::dontwait;
            socket.send(zmq::buffer(message), flags);
        } catch (const zmq::error_t& e) {
            RCLCPP_WARN(get_logger(), "Failed to publish teleop payload for topic %s: %s",
                        topic.c_str(), e.what());
        }
    }

    void flush_delayed_payloads() {
        if (queued_payloads_.empty()) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        while (!queued_payloads_.empty() && queued_payloads_.front().ready_at <= now) {
            auto item = std::move(queued_payloads_.front());
            queued_payloads_.pop_front();
            send_payload_message(item.topic, item.message, item.is_image);
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

    void handle_nav6d_path(nav_msgs::msg::Path::ConstSharedPtr msg) {
        if (!msg) {
            return;
        }
        const auto payload = path_to_ptree(*msg);
        publish_payload("/nav6d/planner/path", payload, false);
    }

    void handle_path_quality(nav6d::msg::PathQuality::ConstSharedPtr msg) {
        if (!msg) {
            return;
        }
        const auto payload = path_quality_to_ptree(*msg);
        publish_payload("/nav6d/path_quality", payload, false);
    }

    void handle_execution_summary(const std::string& topic,
                                  nav6d::msg::PathExecutionSummary::ConstSharedPtr msg) {
        if (!msg) {
            return;
        }
        const auto payload = execution_summary_to_ptree(*msg);
        publish_payload(topic, payload, false);
    }


    void handle_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!msg) {
            return;
        }
        pt::ptree payload;
        payload.put("linear.x", msg->linear.x);
        payload.put("linear.y", msg->linear.y);
        payload.put("linear.z", msg->linear.z);
        payload.put("angular.x", msg->angular.x);
        payload.put("angular.y", msg->angular.y);
        payload.put("angular.z", msg->angular.z);
        publish_payload("/space_cobot/cmd_vel", payload, false);
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
            if (one_way_delay_.count() <= 0) {
                process_command(payload);
                continue;
            }
            const auto ready_at = std::chrono::steady_clock::now() + one_way_delay_;
            queued_commands_.push_back({ready_at, payload});
        }
    }

    void flush_delayed_commands() {
        if (queued_commands_.empty()) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        while (!queued_commands_.empty() && queued_commands_.front().ready_at <= now) {
            auto item = std::move(queued_commands_.front());
            queued_commands_.pop_front();
            process_command(item.payload);
        }
    }

    void process_command(const std::string& payload) {
        try {
            std::istringstream iss(payload);
            pt::ptree root;
            pt::read_json(iss, root);
            const auto topic = root.get<std::string>("topic", "");
            const auto data = root.get_child_optional("data");
            if (!data) {
                return;
            }

            if (topic == "/space_cobot/cmd_vel") {
                handle_cmd_vel_command(root, *data);
            } else if (topic == "/nav6d/goal") {
                handle_nav6d_goal_command(*data);
            } else if (topic == "/nav6d/planner/path") {
                handle_nav6d_path_command(*data);
            }
        } catch (const pt::json_parser_error& e) {
            RCLCPP_WARN(get_logger(), "Failed to parse teleop command JSON: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to process teleop command: %s", e.what());
        }
    }

    void handle_cmd_vel_command(const pt::ptree& root, const pt::ptree& data) {
        try {
            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = data.get<double>("linear.x", 0.0);
            twist.linear.y = data.get<double>("linear.y", 0.0);
            twist.linear.z = data.get<double>("linear.z", 0.0);
            twist.angular.x = data.get<double>("angular.x", 0.0);
            twist.angular.y = data.get<double>("angular.y", 0.0);
            twist.angular.z = data.get<double>("angular.z", 0.0);

            // Determine input frame: optional override from JSON, else default param.
            std::string input_frame = default_input_frame_;
            if (allow_cmd_frame_override_) {
                // Check at root level or inside data
                const auto root_frame = root.get_optional<std::string>("frame_id");
                const auto data_frame = data.get_optional<std::string>("frame_id");
                if (root_frame && !root_frame->empty()) {
                    input_frame = *root_frame;
                } else if (data_frame && !data_frame->empty()) {
                    input_frame = *data_frame;
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
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to process /space_cobot/cmd_vel command: %s",
                        e.what());
        }
    }

    void handle_nav6d_goal_command(const pt::ptree& data) {
        try {
            geometry_msgs::msg::PoseStamped goal_msg;
            goal_msg.header.stamp = now();
            goal_msg.header.frame_id = "world";

            if (const auto header = data.get_child_optional("header")) {
                goal_msg.header.frame_id =
                    header->get<std::string>("frame_id", goal_msg.header.frame_id);
            } else if (const auto frame = data.get_optional<std::string>("frame_id")) {
                goal_msg.header.frame_id = *frame;
            }

            goal_msg.pose = ptree_to_pose(data);
            nav_goal_pub_->publish(goal_msg);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to process /nav6d/goal command: %s", e.what());
        }
    }

    void handle_nav6d_path_command(const pt::ptree& data) {
        try {
            auto path_msg = ptree_to_path(data);
            if (path_msg.header.stamp.sec == 0 && path_msg.header.stamp.nanosec == 0) {
                path_msg.header.stamp = now();
            }
            if (path_msg.header.frame_id.empty()) {
                path_msg.header.frame_id = "world";
            }
            nav_path_pub_->publish(path_msg);
            RCLCPP_INFO(get_logger(), "Forwarded /nav6d/planner/path (%zu poses)",
                        path_msg.poses.size());
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to process /nav6d/planner/path command: %s",
                        e.what());
        }
    }

    zmq::context_t context_;
    zmq::socket_t sensor_pub_;
    zmq::socket_t image_pub_;
    zmq::socket_t cmd_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_path_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr nav_path_sub_;
    rclcpp::Subscription<nav6d::msg::PathQuality>::SharedPtr path_quality_sub_;
    rclcpp::Subscription<nav6d::msg::PathExecutionSummary>::SharedPtr velocity_summary_sub_;
    rclcpp::Subscription<nav6d::msg::PathExecutionSummary>::SharedPtr force_summary_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;

    std::optional<std::chrono::steady_clock::time_point> last_cmd_time_;
    int jpeg_quality_{80};
    std::chrono::milliseconds one_way_delay_{0};

    struct QueuedPayload {
        std::chrono::steady_clock::time_point ready_at;
        std::string topic;
        std::string message;
        bool is_image{false};
    };

    struct QueuedCommand {
        std::chrono::steady_clock::time_point ready_at;
        std::string payload;
    };

    std::deque<QueuedPayload> queued_payloads_;
    std::deque<QueuedCommand> queued_commands_;

    // Frames and TF
    std::string command_frame_;
    std::string default_input_frame_;
    bool allow_cmd_frame_override_{true};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
