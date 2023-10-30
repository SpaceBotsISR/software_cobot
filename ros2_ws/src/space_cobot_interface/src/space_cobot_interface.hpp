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

#define NUM_MOTORS 6
#define pi 3.1415926535897932384
#define FORCE_MODE 0
#define MOMENT_MODE 0

class SpaceCobotController : public rclcpp::Node
{
public:
    SpaceCobotController();
};