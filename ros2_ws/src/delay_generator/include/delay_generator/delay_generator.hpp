#ifndef __DELAY_GENERATOR__
#define __DELAY_GENERATOR__

// File: delay_generator.hpp

// c++ headers
#include <chrono>
#include <iostream>
#include <string>
#include <map>
#include <set>

// ros2 headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "std_msgs/msg/string.hpp"

// nlohmann json includes

#include <nlohmann/json.hpp>
// project headers
#include "delay_generator/delayd_topic_info.hpp"
#include "delay_generator/delayed_message.hpp"

using json = nlohmann::json;

class DelayGenerator : public rclcpp::Node
{
public:
  DelayGenerator(std::string node_name, std::string yaml_file_path);

  ~DelayGenerator() = default;

  bool create_all_subscriptions();

private:
  json j_;
  std::string node_name_;
  std::chrono::milliseconds delay_time_;
  std::string json_file_path_;
  std::map<std::string, std::string> topics;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr subscriber_callback_group_;

  std::map<std::shared_ptr<DelayedMessage>, rclcpp::TimerBase::SharedPtr> delayed_messages_map_;

  std::map<std::string, std::shared_ptr<DelayedTopicInfo>> delayed_topic_info_;
  std::map<std::string, rclcpp::GenericPublisher::SharedPtr> Publishers;
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> Subscribers;
};

#endif // __DELAY_GENERATOR__
