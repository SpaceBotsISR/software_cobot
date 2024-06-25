#ifndef __DELAYD_TOPIC_INFO_HPP__
#define __DELAYD_TOPIC_INFO_HPP__

#include <string>
#include <rclcpp/rclcpp.hpp>

class DelayedTopicInfo {
public: 
    DelayedTopicInfo(std::string topic_name, 
                    std::string topic_type,
                    std::string delayed_topic_name, 
                    rclcpp::GenericSubscription::SharedPtr subscription)
                    {
        topic_name_ = topic_name;
        topic_type_ = topic_type;
        delayed_topic_name_ = delayed_topic_name;
        subscription_ = subscription;
    }

    ~DelayedTopicInfo() = default;

    std::string getTopicName() {
        return topic_name_;
    }

    std::string getTopicType() {
        return topic_type_;
    }

    std::string getDelayedTopicName() {
        return delayed_topic_name_;
    }

    rclcpp::GenericSubscription::SharedPtr getSubscription() {
        return subscription_;
    }

private:
    std::string topic_name_;
    std::string topic_type_;
    std::string delayed_topic_name_;
    rclcpp::GenericSubscription::SharedPtr subscription_;
};

#endif