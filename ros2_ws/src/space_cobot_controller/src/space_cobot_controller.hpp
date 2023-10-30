#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

#include "AttitideControl.h"
#include "PositionControl.h"
#include "Actuation.h"

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

#define pi 3.1415926535897932384
#define POSITION 0

class SpaceCobotController_ : public rclcpp::Node
{

public:
    SpaceCobotController_();
};
