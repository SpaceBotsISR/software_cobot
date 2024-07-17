#ifndef __INTERFACE_CONTROLLER_HPP__
#define __INTERFACE_CONTROLLER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/wait_result.hpp>
#include <rclcpp/executor.hpp>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include <iostream>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <mavros_msgs/msg/rc_in.hpp>

using std::placeholders::_1;

class InterfaceController : public rclcpp::Node
{
public:
    InterfaceController();
    ~InterfaceController();

private:
    void rc_callback(const mavros_msgs::msg::RCIn::SharedPtr msg);
    void spin_node();
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_subscriber;

    std::thread spin_thread;
    rclcpp::TimerBase::SharedPtr spin_timer;
    rclcpp::TimerBase::SharedPtr controll_watch_dog; 

    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state;

    void change_state_callback(const rclcpp::Client <lifecycle_msgs::srv::ChangeState>::SharedFuture future);

    // Function to change interface state with 100 ms timeout
    bool change_state(std::uint8_t transition, std::chrono::milliseconds time_ut = std::chrono::milliseconds(100));

    // Function to get interface state with 100ms timeout
    unsigned int get_state(std::chrono::milliseconds timeout = std::chrono::milliseconds(100));
};

#endif