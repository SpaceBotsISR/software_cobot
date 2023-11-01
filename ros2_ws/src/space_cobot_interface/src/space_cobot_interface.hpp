#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

#include <eigen3/Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>

// including files for actuator control by ROS
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/actuator_control.hpp>
#include <mavros_msgs/msg/rc_in.hpp>

#include "space_cobot_interface/msg/pwm_values.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Scalar.h>

#include "AMatrix.h"
#include "Actuation.h"

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <vector>

#include <thread> 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

using std::placeholders::_1;


#define NUM_MOTORS 6
#define pi 3.1415926535897932384
#define FORCE_MODE 0
#define MOMENT_MODE 0

class Space_Cobot_Interface : public rclcpp::Node
{
public:
    Space_Cobot_Interface();

    void subscriber_and_parameter_declare();

    void run();

    rclcpp::TimerBase::SharedPtr run_timer;

    void state_cb (const mavros_msgs::msg::State::ConstPtr &msg);
    void rc_controlCallback (const mavros_msgs::msg::RCIn::ConstPtr &msg);
    void pwmValuesCallback(const space_cobot_interface::msg::PwmValues::ConstPtr &msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstPtr &msg);

private: 

  std::thread run_thread;
    
    std::vector<double> force_ol;
    std::vector<double> torque_ol;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<space_cobot_interface::msg::PwmValues>::SharedPtr pwm_values;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_values;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr fmode;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_torque;
    rclcpp::Publisher<mavros_msgs::msg::ActuatorControl>::SharedPtr actuator_controls_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_force;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
};