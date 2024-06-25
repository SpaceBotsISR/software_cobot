#ifndef __CONTROLLER_MASTER_HANDLE_HPP__
#define __CONTROLLER_MASTER_HANDLE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include <vector>
#include <utility>

#include "controller_master/AMatrix.hpp"
#include "controller_master/Actuation.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

#define NUM_MOTORS 4

class ControllerMasterHandle : public rclcpp::Node
{
public:
    ControllerMasterHandle(std::string node_name);
    ~ControllerMasterHandle();

private:
    void declare_publisher();
    void declare_subscribers();
    void declare_timer();

    void publish_desired_attuation();

    rclcpp::TimerBase::SharedPtr desired_atuation_timer;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_attitude_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_translation_sub;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_attuation_pub;

    void desired_attitude_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void desired_translation_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    std::vector<double> desired_attitude;
    std::vector<double> desired_translation;

};
#endif
