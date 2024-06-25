#ifndef __ATTITUDE_CONTROLLER_HANDLE_HPP__
#define __ATTITUDE_CONTROLLER_HANDLE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/actuator_control.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Scalar.h>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <eigen3/Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "attitude_controller/euler_angles_input.hpp"

#include <map>
#include <numbers> // std::numbers::pi
#include <string>
#include <utility>
#include <vector>

using std::placeholders::_1;

#define NUM_MOTORS 6

#define PI 3.1415926535897932384

enum FlightModes
{
    BOARD,
    MANUAL_ORIENTATION_WORLD_INCREMENTAL,
    MANUAL_ORIENTATION_WORLD_ABSOLUTE,
    MANUAL_ORIENTATION_BODY_INCREMENTAL,
    MANUAL_ORIENTATION_BODY_ABSOLUTE,

    OPEN_LOOP_FORCE,
    OPEN_LOOP_TORQUE_WORLD,
    OPEN_LOOP_TORQUE_BODY
}; // 1099 is BOARD, 1500 is MANUAL and 1901 is OPEN_LOOP

class AttitudeControl : public rclcpp::Node
{
public:
    AttitudeControl(std::string node_name);
    ~AttitudeControl();

private:
    void declare_publisher();
    void declare_subscribers();
    void declare_timer();

    Eigen::Matrix3d rc_Rd;

    std::vector<double> quaternion_to_vector(Eigen::Quaterniond q);
    std::vector<double> quaternion_to_vector(tf2::Quaternion &tf_quaternion);
    std::vector<double> quaternion_to_vector(geometry_msgs::msg::Quaternion gm_quaternion);
    Eigen::Quaterniond gm_quaternion_to_eigen(const geometry_msgs::msg::Quaternion &imu_quat);

    void select_flight_mode();

    rclcpp::TimerBase::SharedPtr attitude_timer_;

    FlightModes flight_mode;
    RCControl *euler_angle_rc;

    std::vector<double> desired_attitude;

    void calculate_desired_attitude();
    void publish_desired_attitude();

    void rc_callback(const mavros_msgs::msg::RCIn::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr sub_remote;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_attitude_pub;

    sensor_msgs::msg::Imu::SharedPtr imu_data;
    geometry_msgs::msg::Quaternion imu_quaternion;

    std::vector<double> angular_velocity_current;
    std::vector<double> angular_velocity_desired;

    Eigen::Vector3d moment;

    unsigned int flight_mode_channel = 5;
    unsigned int joystick_mode_channel = 7;

    unsigned int joystick_mode_switch = 1500;
    unsigned int flight_mode_switch = 1500;

    unsigned int roll_input = 1500;
    unsigned int pitch_input = 1500;
    unsigned int yaw_input = 1500;

    unsigned int incremental = 1;
    unsigned int toggle_world_body_frames = 1100; // No longer updating this variable, must check what it is doing
    unsigned int toggle_position_orientation = 1500;

    unsigned int rc_flag = 1;

    // unsigned int toggle_world_body_frames_channel = 7;
    unsigned int toggle_position_orientation_channel = 2;

    void manual_orientation_world_incremental();
    void manual_orientation_body_incremental();
    void manual_orientation_body_absolute();
    void manual_orientation_world_absolute();
    void open_loop_torque_world();
    void open_loop_torque_body();

    Eigen::Vector3d attitudeController(Eigen::RowVector4d q_current, Eigen::RowVector4d q_des, Eigen::Vector3d omega_current, Eigen::Vector3d omega_des, Eigen::Vector3d omegaD_des);
    Eigen::Matrix3d getRotationMatrix(Eigen::RowVector4d q_current);
    Eigen::Matrix3d getDesiredRotationMatrix(Eigen::RowVector4d q_des);
    float trace(Eigen::Matrix3d mat);
    Eigen::Vector3d invskew(Eigen::Matrix3d mat);
    Eigen::Matrix3d skew(Eigen::Vector3d vec);
};

#endif