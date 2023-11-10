#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

#include "AttitideControl.h"
#include "PositionControl.h"
#include "Actuation.h"

#include <std_msgs/msg/float64_multi_array.hpp>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/vector3_stamped.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/actuator_control.hpp>
#include <mavros_msgs/msg/rc_in.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include "AMatrix.h"

// #include <mav_msgs/Actuators.h>
#include <space_cobot_interface/msg/pwm_values.hpp>
#include <mocap_interface/msg/mocap_msg.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Scalar.h>

#include <thread>

#define pi 3.1415926535897932384
#define POSITION 0

using std::placeholders::_1;

class SpaceCobotController : public rclcpp::Node
{

public:
    SpaceCobotController();

private:
    void subscriber_and_parameter_declare();

    void run();

    void desiredPoseCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg);
    void desiredTwistCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg);
    void rc_controlCallback(const mavros_msgs::msg::RCIn::ConstPtr &msg);
    void IMUCallback(const sensor_msgs::msg::Imu::ConstPtr &msg);
    void mocapCallback(const mocap_interface::msg::MocapMsg &msg);

    void get_desired_orientation(const std_msgs::msg::Float64MultiArray &msg);

    std::thread run_thread;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_des_twist;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr fmode;
    rclcpp::Subscription<mocap_interface::msg::MocapMsg>::SharedPtr sub_current_position;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_des_pose;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_current_pose;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_desired_orientation;

    rclcpp::Publisher<space_cobot_interface::msg::PwmValues>::SharedPtr imp_values;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub;
};
