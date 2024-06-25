#ifndef __DELAYED_MESSAGE__
#define __DELAYED_MESSAGE__

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <future>
#include <memory>

class DelayedMessage : public std::enable_shared_from_this<DelayedMessage>
{
public:
    DelayedMessage(
        std::shared_ptr<rclcpp::SerializedMessage> message,
        rclcpp::GenericPublisher::SharedPtr publisher)
    {
        message_ = message;
        publisher_ = publisher;
    }

    ~DelayedMessage()
    {
        std::cout << "DelayedMessage::~DelayedMessage" << std::endl;
    };

    // Start the timer to publish the message after the delay time
    void publish_message()
    {
        const rclcpp::SerializedMessage msg_ = *message_;
        this->publisher_->publish(msg_);
    }

private:
    rclcpp::GenericPublisher::SharedPtr publisher_;
    std::shared_ptr<rclcpp::SerializedMessage> message_;
};

#endif