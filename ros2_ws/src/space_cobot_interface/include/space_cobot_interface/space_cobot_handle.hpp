#ifndef __SPACE_COBOT_INTERFACE_HPP__
#define __SPACE_COBOT_INTERFACE_HPP__

#include <iostream>
#include <vector>
#include <thread>
#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <eigen3/Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>

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

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

using std::placeholders::_1;

#define NUM_MOTORS 6
#define pi 3.1415926535897932384
#define FORCE_MODE 0
#define MOMENT_MODE 0

enum FlightModes
{
  BOARD,
  MANUAL_ORIENTATION_WORLD_INCREMENTAL,
  MANUAL_ORIENTATION_WORLD_ABSOLUTE,
  MANUAL_ORIENTATION_BODY_INCREMENTAL,
  MANUAL_ORIENTATION_BODY_ABSOLUTE,
  MANUAL_POSITION_INCREMENTAL,
  MANUAL_POSITION_ABSOLUTE,

  OPEN_LOOP_FORCE,
  OPEN_LOOP_TORQUE_WORLD,
  OPEN_LOOP_TORQUE_BODY
}; // 1099 is BOARD, 1500 is MANUAL and 1901 is OPEN_LOOP

class Space_Cobot_Interface : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Space_Cobot_Interface(std::string node_name, bool intra_process_coms = false);

private:
  // Lifecycle node callbacks declare
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
  rclcpp::Subscription<space_cobot_interface::msg::PwmValues>::SharedPtr pwm_values;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr fmode;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_torque;
  rclcpp::Publisher<mavros_msgs::msg::ActuatorControl>::SharedPtr actuator_controls_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_force;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;

  /// Declaring MAVROS State
  mavros_msgs::msg::State current_state;

  // Function to initialize all the parameters

  // Function to initialize all the publishers
  void declare_publishers();
  void reset_publishers();
  // Function to initialize all the subscribers
  void declare_subscribers();
  void reset_subscribers();

  // Function to initialize all the clients
  void declare_clients();
  void reset_clients();

  // Function where all code node will be run
  void run_node();

  void change_px4_custom_mode(std::string new_mode);
  void change_px4_arm_state(bool arm);

  void set_desired_orientation(std::vector<double> desired_orientation);

  // Subscriber Callbacks
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg);
  void pwmValuesCallback(const space_cobot_interface::msg::PwmValues::SharedPtr msg);

  void set_mode_px4();

  rclcpp::TimerBase::SharedPtr px4_arming_timer;
  rclcpp::Time last_arming_request;
};

#endif // __SPACE_COBOT_INTERFACE_HPP__