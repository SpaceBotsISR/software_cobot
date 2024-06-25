#ifndef __POINT_COLECTOR_HPP__
#define __POINT_COLECTOR_HPP__

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <sensor_msgs/msg/imu.hpp>

class PointColetor : public rclcpp::Node
{
public:
    PointColetor(std::string node_name, std::string file_name, bool use_intra_coms);
    ~PointColetor();

private:
    void random_actuation_generator();
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void actuate();

    void store_data_to_file();

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr actuation_pub;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    rclcpp::TimerBase::SharedPtr random_actuation_timer;
    rclcpp::TimerBase::SharedPtr actuate_timer;

    std::vector<double> random_actuation;

    std::string file_name;
    std::ofstream file;
};

#endif