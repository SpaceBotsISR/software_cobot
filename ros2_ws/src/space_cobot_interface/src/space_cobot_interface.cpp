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

/// Defining the flight modes here
// TODO(ALG): The different flight_modes should be #1 BOARD, #2 MANUAL_POSITION, #3 MANUAL_ATTITUDE, #4
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

using namespace std;

/// Inputs from the transmitter-- default neutral value being 1499.. These are used by manual_control, torque_control, force_control
int roll_input = 1499, pitch_input = 1499, yaw_input = 1499;

/// Declaring flight_mode switch for enabling manual control
int pwm_flight_mode = 1099;
int toggle_position_orientation = 1100;
int toggle_world_body_frames = 1100;
// TODO(ALG): to be used as a rosparam
int incremental = 1;

/// bool values so that notification messages are printed only once
bool isaid_board = false, isaid_manual_orientation_world_incremental = false,
     isaid_manual_orientation_world_absolute = false, isaid_manual_orientation_body_incremental = false,
     isaid_manual_orientation_body_absolute = false, isaid_manual_position_incremental = false,
     isaid_manual_position_absolute = false, isaid_openloop_torque_world = false,
     isaid_openloop_torque_body = false, isaid_openloop_force = false;

/// Declaring value for kill switch to set the value of err_int and des_orientation_rpy when kill is deployed such that it doesnt bounce back when kill is switched off
/// Kill-OFF is 1099

int kill_switch = 1099;

Eigen::Quaterniond orientation_current(1, 0, 0, 0); /// =identity
vector<double> position_current(3), angular_velocity_current(3), velocity_current(3), acceleration_current(3);

/// The Eigen::Quaternion is (w, x, y, z)
Eigen::Quaterniond orientation_desired(1, 0, 0, 0);
vector<double> orientation_desired_rpy(3), angular_velocity_desired(3), position_desired(3), velocity_desired(3);

Eigen::Vector3d target_moment, target_force; // The values obtained from Callback function so that the continuity in the moments is maintained.
// Setting the force vector to zero in OPEN_LOOP_TORQUE
Eigen::Vector3d force(0, 0, 0);
Eigen::Vector3d moment(0, 0, 0);

/// rc flag for locking the current orientation for manual control when first switched.
int rc_flag = 0;

/// defining current matrix for rc control
Eigen::Matrix3d rc_Rd;

/// Declaring MAVROS State
mavros_msgs::msg::State current_state;

void state_cb(const mavros_msgs::msg::State::ConstPtr &msg)
{
    current_state = *msg;
}

void rc_controlCallback(const mavros_msgs::msg::RCIn::ConstPtr &msg)
{
    pwm_flight_mode = msg->channels[5];

    toggle_position_orientation = msg->channels[2];
    toggle_world_body_frames = msg->channels[7];

    roll_input = msg->channels[0];
    pitch_input = msg->channels[1];
    yaw_input = msg->channels[3];

    kill_switch = msg->channels[4];
}

void manualControl_OrientationWorldIncremental()
{
    double rc_x_change = ((roll_input - 1497) * 0.00225 * (pi / 180)); // converts change in rc PWM values into change in angles
    double rc_y_change = ((pitch_input - 1498) * 0.00225) * (pi / 180);
    double rc_z_change = ((yaw_input - 1499) * 0.00225) * (pi / 180);
    // cout<<"CHEcKPOINT 1"<<endl;

    Eigen::Matrix3d Ri;
    /// cv version of Ri and angle change for rodriguez formula
    cv::Mat cvRI(3, 3, CV_32FC1);
    cv::Mat cvChange(1, 3, CV_32FC1);
    // cout<<"CHECKPOINT 2 "<<endl;

    /// Assigning angle change values to cvChange for rodriguez formula
    cvChange.ptr<float>(0)[0] = rc_x_change;
    cvChange.ptr<float>(0)[1] = rc_y_change;
    cvChange.ptr<float>(0)[2] = rc_z_change;

    // cout<<"The CV matrix that goes into the Rodriguez formula"<<endl;
    // cout<<cvChange<<endl;

    // cout<<"CHECKPOINT 2.5"<<endl;

    /// The main Rodriguez Formula
    cv::Rodrigues(cvChange, cvRI);

    // cout<<"CHECKPOINT 3"<<endl;
    // cout<<"3X3 matrix after the Rodriguez Formula"<<endl;
    // cout<<cvRI<<endl;

    /// Need to convert cvRI to Ri i.e the eigen matrix for easier multiplication
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            Ri(i, j) = cvRI.ptr<float>(i)[j];
        }
    // cout<<"CHECKPOINT 4"<<endl;
    // cout<<"This is Ri-- the change matrix in Eigen form-----should be same as the 3x3 matrix after the rodriguez formula: "<<endl;
    // cout<<Ri<<endl;

    if (rc_flag != 1)
    { /// rc_flag more like rc_lock for orientation----- so that the rc_Rd is given the value of current orientation only when its flicked to manual mode-- after that it takes the value of the ri*rd not current orientation
        tf2::Quaternion q_temp_cur;
        q_temp_cur.setX(orientation_current.x());
        q_temp_cur.setY(orientation_current.y());
        q_temp_cur.setZ(orientation_current.z());
        q_temp_cur.setW(orientation_current.w());

        tf2::Matrix3x3 rc33(q_temp_cur); // transform the current quat to 3X3 --> use a tf function to change this

        // cout<<"Locked Initial Current Orientation: "<<endl;
        // cout<<rc33.getRow(0)<<endl;
        // cout<<rc33.getRow(1)<<endl;
        // cout<<rc33.getRow(2)<<endl;

        /// changing the current orientation to 3x3 matrix Eigen
        rc_Rd(0, 0) = rc33.getRow(0).x();
        rc_Rd(0, 1) = rc33.getRow(0).y();
        rc_Rd(0, 2) = rc33.getRow(0).z();
        rc_Rd(1, 0) = rc33.getRow(1).x();
        rc_Rd(1, 1) = rc33.getRow(1).y();
        rc_Rd(1, 2) = rc33.getRow(1).z();
        rc_Rd(2, 0) = rc33.getRow(2).x();
        rc_Rd(2, 1) = rc33.getRow(2).y();
        rc_Rd(2, 2) = rc33.getRow(2).z();
    }
    // cout<<"This is either the current orientation (when flicked for the first time) or changed orientation after applying orientations "<<endl;
    // cout<<rc_Rd<<endl;

    // cout<<"CHECKPOINT 5"<<endl;

    rc_flag = 1;

    /// This is causing all the problems -----> continuously multiplying and going to zero....
    rc_Rd = Ri * rc_Rd;
    // cout<<"The des_orientation 3x3 matrix---currently has no changes and IS EQUAL TO Ri"<<endl;
    // cout<<rc_Rd<<endl;

    Eigen::Quaterniond quatdes(rc_Rd);
    quatdes.normalize();
    orientation_desired.x() = quatdes.x();
    orientation_desired.y() = quatdes.y();
    orientation_desired.z() = quatdes.z();
    orientation_desired.w() = quatdes.w();

    // cout<<"Desired Orientation after changing from rc_Rd: "<<orientation_des.x()<<" "<<orientation_des.y()<<" "<<orientation_des.z()<<" "<<orientation_des.w()<<endl;
}

void manualControl_OrientationBodyIncremental()
{                                                                      // it is multiplied by a constant (90/400) because I want a full stick up(or down) to mean 90 degrees in angle change. Also, the range of stick movement in one side is 400 (1100-1500). Hence each unit of stick movement change should give stick_input*(90/400) in angle change. Hence Result!!!
    double rc_x_change = ((roll_input - 1497) * 0.00225 * (pi / 180)); // converts change in rc PWM values into change in angles in radians
    double rc_y_change = ((pitch_input - 1498) * 0.00225) * (pi / 180);
    double rc_z_change = ((yaw_input - 1499) * 0.00225) * (pi / 180);

    Eigen::Matrix3d Ri;

    cv::Mat cvRI(3, 3, CV_32FC1);
    cv::Mat cvChange(1, 3, CV_32FC1);

    /// Assigning angle change values to cvChange for rodriguez formula
    cvChange.ptr<float>(0)[0] = rc_x_change;
    cvChange.ptr<float>(0)[1] = rc_y_change;
    cvChange.ptr<float>(0)[2] = rc_z_change;

    /// The main Rodriguez Formula
    cv::Rodrigues(cvChange, cvRI);

    /// Need to convert cvRI to Ri i.e the eigen matrix for easier multiplication
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            Ri(i, j) = cvRI.ptr<float>(i)[j];
        }

    if (rc_flag != 1)
    { /// rc_flag more like rc_lock for orientation----- so that the rc_Rd is given the value of current orientation only when its flicked to manual mode-- after that it takes the value of the ri*rd not current orientation
        tf2::Quaternion q_temp_cur;
        q_temp_cur.setX(orientation_current.x());
        q_temp_cur.setY(orientation_current.y());
        q_temp_cur.setZ(orientation_current.z());
        q_temp_cur.setW(orientation_current.w());

        tf2::Matrix3x3 rc33(q_temp_cur); // transform the current quat to 3X3 --> use a tf function to change this

        /// changing the current orientation to 3x3 matrix Eigen
        rc_Rd(0, 0) = rc33.getRow(0).x();
        rc_Rd(0, 1) = rc33.getRow(0).y();
        rc_Rd(0, 2) = rc33.getRow(0).z();
        rc_Rd(1, 0) = rc33.getRow(1).x();
        rc_Rd(1, 1) = rc33.getRow(1).y();
        rc_Rd(1, 2) = rc33.getRow(1).z();
        rc_Rd(2, 0) = rc33.getRow(2).x();
        rc_Rd(2, 1) = rc33.getRow(2).y();
        rc_Rd(2, 2) = rc33.getRow(2).z();
    }

    rc_flag = 1;

    /// Rotation in body frame
    rc_Rd = rc_Rd * Ri;

    /// Now convert the New rotation into orientation desired quaternion
    // tf::Quaternion quatdes();

    Eigen::Quaterniond quatdes(rc_Rd);
    quatdes.normalize();
    orientation_desired.x() = quatdes.x();
    orientation_desired.y() = quatdes.y();
    orientation_desired.z() = quatdes.z();
    orientation_desired.w() = quatdes.w();
}

void manualControl_OrientationBodyAbsolute(Eigen::Quaterniond current_orientation_manual_absolute)
{

    /// We get change in desired angles (rpy) in euler angles (degrees)
    double roll_angle_change = ((roll_input - 1499) * 0.225); /// it is multiplied by a constant (90/400) because I want a full stick up(or down) to mean 90 degrees in angle change. Also, the range of stick movement in one side is 400 (1100-1500). Hence each unit of stick movement change should give stick_input*(90/400) in angle change. Hence Result!!!
    double pitch_angle_change = ((pitch_input - 1499) * 0.225);
    double yaw_angle_change = ((yaw_input - 1499) * 0.225);

    tf2::Quaternion q_temp_cur;
    q_temp_cur.setX(current_orientation_manual_absolute.x());
    q_temp_cur.setY(current_orientation_manual_absolute.y());
    q_temp_cur.setZ(current_orientation_manual_absolute.z());
    q_temp_cur.setW(current_orientation_manual_absolute.w());

    tf2::Matrix3x3 mat1(q_temp_cur);
    tf2Scalar roll_cur = 0, pitch_cur = 0, yaw_cur = 0;
    mat1.getEulerYPR(yaw_cur, pitch_cur, roll_cur);

    orientation_desired_rpy[0] = roll_cur * (180 / pi) + roll_angle_change; // roll_curr is changed to degrees before adding to change (also in degrees) and making orientation_desired (degrees)
    orientation_desired_rpy[1] = pitch_cur * (180 / pi) + pitch_angle_change;
    orientation_desired_rpy[2] = yaw_cur * (180 / pi) + yaw_angle_change;

    cout << "Orientaiton Desired: " << orientation_desired_rpy[0] << endl;
}

void manualControl_OrientationWorldAbsolute(Eigen::Quaterniond current_orientation_manual_absolute)
{ // Explanation: the reason to include desired orientation in the manual absolute in the world frame is because in the equation rc_Rd = Ri*rc_Rd, rd_Rd is constantly being updated thus defeating the purpose of using a central orientation around whichthe absolute values of rotation are applied

    double rc_x_change = ((roll_input - 1497) * 0.225 * (pi / 180)); // converts change in rc PWM values into change in angles in radians
    double rc_y_change = ((pitch_input - 1498) * 0.225) * (pi / 180);
    double rc_z_change = ((yaw_input - 1499) * 0.225) * (pi / 180);

    Eigen::Matrix3d Ri;

    cv::Mat cvRI(3, 3, CV_32FC1);
    cv::Mat cvChange(1, 3, CV_32FC1);

    /// Assigning angle change values to cvChange for rodriguez formula
    cvChange.ptr<float>(0)[0] = rc_x_change;
    cvChange.ptr<float>(0)[1] = rc_y_change;
    cvChange.ptr<float>(0)[2] = rc_z_change;

    /// The main Rodriguez Formula
    cv::Rodrigues(cvChange, cvRI);

    /// Need to convert cvRI to Ri i.e the eigen matrix for easier multiplication
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            Ri(i, j) = cvRI.ptr<float>(i)[j];
        }

    /// defining current matrix for rc control
    Eigen::Matrix3d rc_Rd;

    if (1)
    { /// rc_flag more like rc_lock for orientation----- so that the rc_Rd is given the value of current orientation only when its flicked to manual mode-- after that it takes the value of the ri*rd not current orientation
        tf2::Quaternion q_temp_cur;
        q_temp_cur.setX(current_orientation_manual_absolute.x());
        q_temp_cur.setY(current_orientation_manual_absolute.y());
        q_temp_cur.setZ(current_orientation_manual_absolute.z());
        q_temp_cur.setW(current_orientation_manual_absolute.w());

        tf2::Matrix3x3 rc33(q_temp_cur); // transform the current quat to 3X3 --> use a tf function to change this

        /// changing the current orientation to 3x3 matrix Eigen
        rc_Rd(0, 0) = rc33.getRow(0).x();
        rc_Rd(0, 1) = rc33.getRow(0).y();
        rc_Rd(0, 2) = rc33.getRow(0).z();
        rc_Rd(1, 0) = rc33.getRow(1).x();
        rc_Rd(1, 1) = rc33.getRow(1).y();
        rc_Rd(1, 2) = rc33.getRow(1).z();
        rc_Rd(2, 0) = rc33.getRow(2).x();
        rc_Rd(2, 1) = rc33.getRow(2).y();
        rc_Rd(2, 2) = rc33.getRow(2).z();
    }

    rc_flag = 1;

    // Rotation in world frame
    rc_Rd = Ri * rc_Rd; //<---- this second rc_Rd needs to be changed to fixed orientation given by the function

    Eigen::Quaterniond quatdes(rc_Rd);
    quatdes.normalize();
    orientation_desired.x() = quatdes.x();
    orientation_desired.y() = quatdes.y();
    orientation_desired.z() = quatdes.z();
    orientation_desired.w() = quatdes.w();
}

void manualControl_PositionIncremental()
{
    // get current position -- already known as a global variable

    // get change in position
    double x_change = ((roll_input - 1499) * 0.00225);
    double y_change = ((pitch_input - 1499) * 0.00225);

    // add the change to the current position to determine the final position
    position_desired[0] = position_current[0] + x_change;
    position_desired[1] = position_current[1] + y_change;
}

void manualControl_PositionAbsolute(Eigen::Vector3d current_position_absolute)
{
    // get current position -- already known as a global variable

    // get change in position
    double x_change = ((roll_input - 1499) * 0.00225);
    double y_change = ((pitch_input - 1499) * 0.00225);

    // add the change to the current position to determine the final position
    position_desired[0] = current_position_absolute[0] + x_change;
    position_desired[1] = current_position_absolute[1] + y_change;
}

/// GLOBAL VARIABLE FOR THE PWM VALUES .....................................
// TODO(ALG): Check the safety of this thang!!!

vector<double> actuator_pwm_values(6);

void pwmValuesCallback(const space_cobot_interface::msg::PwmValues::ConstPtr &msg)
{
    for (int i = 0; i < 6; i++)
        actuator_pwm_values[i] = msg->pwm[i];

    target_moment[0] = msg->moment[0];
    target_moment[1] = msg->moment[1];
    target_moment[2] = msg->moment[2];

    target_force[0] = msg->force[0];
    target_force[1] = msg->force[1];
    target_force[2] = msg->force[2];
}

void imuCallback(const sensor_msgs::msg::Imu::ConstPtr &msg)
{

    orientation_current.x() = msg->orientation.x;
    orientation_current.y() = msg->orientation.y;
    orientation_current.z() = msg->orientation.z;
    orientation_current.w() = msg->orientation.w;

    acceleration_current[0] = msg->linear_acceleration.x;
    acceleration_current[1] = msg->linear_acceleration.y;
    acceleration_current[2] = msg->linear_acceleration.z;

    angular_velocity_current[0] = msg->angular_velocity.x;
    angular_velocity_current[1] = msg->angular_velocity.y;
    angular_velocity_current[2] = msg->angular_velocity.z;
}

void setFlightMode(FlightModes &flight_mode)
{

    // TODO(ALG): Setting different flight modes here
    if (pwm_flight_mode == 1099)
        flight_mode = BOARD;

    // MANUAL MODES
    if (pwm_flight_mode == 1500)
    {
        if (toggle_position_orientation < 1500 &&
            toggle_world_body_frames < 1500 &&
            incremental == 1)
        {

            flight_mode = MANUAL_ORIENTATION_WORLD_INCREMENTAL;
        }

        if (toggle_position_orientation < 1500 &&
            toggle_world_body_frames < 1500 &&
            incremental == 0)
        {

            flight_mode = MANUAL_ORIENTATION_WORLD_ABSOLUTE;
        }

        if (toggle_position_orientation < 1500 &&
            toggle_world_body_frames > 1500 &&
            incremental == 1)
        {

            flight_mode = MANUAL_ORIENTATION_BODY_INCREMENTAL;
        }

        if (toggle_position_orientation < 1500 &&
            toggle_world_body_frames > 1500 &&
            incremental == 0)
        {

            flight_mode = MANUAL_ORIENTATION_BODY_ABSOLUTE;
        }

        if (toggle_position_orientation >= 1500 &&
            incremental == 1)
        {
            flight_mode = MANUAL_POSITION_INCREMENTAL;
        }

        if (toggle_position_orientation >= 1500 &&
            incremental == 0)
        {
            flight_mode = MANUAL_POSITION_ABSOLUTE;
        }
    }

    // OPEN LOOP MODES
    if (pwm_flight_mode == 1901)
    {
        if (toggle_position_orientation < 1500 && toggle_world_body_frames < 1500)
        {

            flight_mode = OPEN_LOOP_TORQUE_WORLD;
        }

        if (toggle_position_orientation < 1500 && toggle_world_body_frames >= 1500)
        {
            flight_mode = OPEN_LOOP_TORQUE_BODY;
        }

        if (toggle_position_orientation >= 1500)
        {
            flight_mode = OPEN_LOOP_FORCE;
        }
    }
}

void OpenLoopTorqueWorld(Eigen::Vector3d &M)
{

    double x_change = (roll_input - 1499) * 0.000125;
    double y_change = (pitch_input - 1499) * 0.000125;
    double z_change = (yaw_input - 1499) * 0.000125;

    M[0] = x_change;
    M[1] = y_change;
    M[2] = z_change;
}

void OpenLoopTorqueBody(Eigen::Vector3d &M)
{
    // TODO(ALG): Need to be checked if can be done and implemented... Here for the sake of completion

    double x_change = (roll_input - 1499) * 0.000125;
    double y_change = (pitch_input - 1499) * 0.000125;
    double z_change = (yaw_input - 1499) * 0.000125;

    // New code
    // Cancel very small values from the inputs
    if (x_change < 0.00008 && x_change > -0.00008)
    {
        x_change = 0;
    }
    if (y_change < 0.00008 && y_change > -0.00008)
    {
        y_change = 0;
    }
    if (z_change < 0.00008 && z_change > -0.00008)
    {
        z_change = 0;
    }

    M[0] = x_change;
    M[1] = y_change;
    M[2] = z_change;
}

// Assigns to force the value that comes form the RC control
void OpenLoopForce(Eigen::Vector3d &F)
{

    double x_force = (roll_input - 1499) * 0.0004225;
    double y_force = (pitch_input - 1499) * 0.0004225;
    double z_force = (yaw_input - 1499) * 0.0001225;

    F[0] = x_force;
    F[1] = y_force;
    F[2] = z_force;
}

void quats2rpy(Eigen::Quaterniond temp_orientation_desired, vector<double> &temp_desired_orientation_rpy)
{
    /// Converting the orientations back to quaternions
    tf2::Quaternion q_temp_des, q_temp_cur;

    q_temp_des.setX(temp_orientation_desired.x());
    q_temp_des.setY(temp_orientation_desired.y());
    q_temp_des.setZ(temp_orientation_desired.z());
    q_temp_des.setW(temp_orientation_desired.w());

    tf2::Matrix3x3 mat2(q_temp_des);
    tf2Scalar roll_des = 0, pitch_des = 0, yaw_des = 0;
    mat2.getEulerYPR(yaw_des, pitch_des, roll_des);

    /// The yaw, pitch, roll we get from the above function is in radians, so we need to convert it to degrees before putting it in the ros message
    temp_desired_orientation_rpy[0] = roll_des * (180 / pi);
    temp_desired_orientation_rpy[1] = pitch_des * (180 / pi);
    temp_desired_orientation_rpy[2] = yaw_des * (180 / pi);
}

void getCurrentDefaultMoment(Eigen::Vector3d &moment)
{
    moment = target_moment;
}

void getCurrentForce(Eigen::Vector3d &force)
{
    force = target_force;
}

void setCurrentDefaultMoment(Eigen::Vector3d moment)
{
    target_moment = moment;
}

void setCurrentForce(Eigen::Vector3d &force)
{
    target_force = force;
}

void resetBools()
{

    isaid_board = false;

    isaid_manual_orientation_world_incremental = false;
    isaid_manual_orientation_world_absolute = false;

    isaid_manual_orientation_body_incremental = false;
    isaid_manual_orientation_body_absolute = false;

    isaid_manual_position_incremental = false;
    isaid_manual_position_absolute = false;

    isaid_openloop_torque_world = false;
    isaid_openloop_torque_body = false;

    isaid_openloop_force = false;
}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("space_cobot_interface");

    // Publishing and Subscribing

    auto state_sub = node->create_subscription<mavros_msgs::msg::State>("mavros/state", 10, state_cb);

    auto local_pos_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);

    auto actuator_controls_pub = node->create_publisher<mavros_msgs::msg::ActuatorControl>("/mavros/actuator_control", 1000);

    auto arming_client = node->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

    auto set_mode_client = node->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    auto fmode = node->create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in", 1000, rc_controlCallback);

    auto pwm_values = node->create_subscription<space_cobot_interface::msg::PwmValues>("important_values", 1000, pwmValuesCallback);

    auto imu_values = node->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data", 1000, imuCallback);

    auto pub_force = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("/open_loop/force", 1000);

    auto pub_torque = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("/open_loop/torque", 1000);

    rclcpp::Rate rate(20);

    Eigen::Quaterniond current_orientation_manual_absolute;
    Eigen::Vector3d current_position_manual_absolute;

    /// wait for FCU connection
    while (rclcpp::ok() && !current_state.connected)
    {
        rclcpp::spin_some(node);
        cout << "Connecting to FCU....." << endl;
        rate.sleep();
    }
    if (current_state.connected)
        cout << "Connection to FCU established!" << endl;
    else
    {
        cout << "Connection to FCU FAILED!" << endl;
        return 0;
    }
    /// send a few setpoints before starting in order to arm it
    //%%%%%%%%%%%%%%%%%%%%%%% --------NEED TO CHECK THIS to make sure if this doesnt cause any sudden motion or tells the cobot to go to the desired setpoint... %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // UPDATE on the above comment: NO such thing was observed but still keeping the comment if any future issues arise

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    for (int i = 100; rclcpp::ok() && i > 0; --i)
    {
        local_pos_pub->publish(pose);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    auto offb_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();

    offb_set_mode->custom_mode = "OFFBOARD";

    auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_cmd->value = true;

    rclcpp::Time last_request = node->get_clock()->now();
    vector<double> force_ol(3), torque_ol(3);

    /// Initializing the rosparams
    auto orientation_desired_rpy = node->declare_parameter<vector<double>>("desired_orientation_rpy");
    auto angular_velocity_desired = node->declare_parameter<vector<double>>("desired_angular_velocity");
    auto position_desired = node->declare_parameter<vector<double>>("desired_position", {0.0, 0.0, 0.0});
    auto velocity_desired = node->declare_parameter<vector<double>>("desired_velocity", {0.0, 0.0, 0.0});
    auto incremental = node->declare_parameter<double>("incremental_mode");
    auto force_ol1 = node->declare_parameter<vector<double>>("force_ol");
    auto torque_ol2 = node->declare_parameter<vector<double>>("moment_ol");

    while (rclcpp::ok())
    {

        node->get_parameter("desired_position", position_desired);
        node->get_parameter("desired_velocity", velocity_desired);
        node->get_parameter("desired_orientation_rpy", orientation_desired_rpy);
        node->get_parameter("desired_angular_velocity", angular_velocity_desired);
        node->get_parameter("incremental_mode", incremental);
        node->get_parameter("force_ol", force_ol);
        node->get_parameter("moment_ol", torque_ol);

        /// Making sure the mode set is always OFFBOARD and its always armed
        if (current_state.mode != "OFFBOARD" &&
            (node->get_clock()->now() - last_request > rclcpp::Duration::from_seconds(5.0)))
        {
            auto result = set_mode_client->async_send_request(offb_set_mode);

            if (rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(node->get_logger(), "Offboard enabled");
            }
            last_request = node->get_clock()->now();
        }
        else
        {
            if (!current_state.armed &&
                (node->get_clock()->now() - last_request > rclcpp::Duration::from_seconds(5.0)))
            {
                auto result = arming_client->async_send_request(arm_cmd);
                if (rclcpp::spin_until_future_complete(node, result) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_INFO(node->get_logger(), "Vehicle armed");
                }
                last_request = node->get_clock()->now();
            }
        }

        mavros_msgs::msg::ActuatorControl actuator_control_msg;

        /**********************************SETTING FLIGHT MODES**************************************************
        *********************************************************************************************************/
        FlightModes flight_mode;
        /// SETTING THE FLIGHT MODE
        setFlightMode(flight_mode);

        if (flight_mode == FlightModes::BOARD)
        {
            // get motor_actuation_values from the the pwm_values rosmessage of the space_cobot_controller rosnode
            if (!isaid_board)
            {
                // Prints once
                cout << "Board Mode" << endl;
                resetBools();
                isaid_board = true;
            }
        }

        ///////////////////////////////////////      MANUAL MODES /////////////////////////////////////////////////////////
        //////////////////////////////////////                 ////////////////////////////////////////////////////////
        else if (flight_mode == FlightModes::MANUAL_ORIENTATION_WORLD_INCREMENTAL)
        {
            // set the desired rpy values from the transmitter and then get the motor actuation values from the rosmessage of the space_cobot_controller rosnode
            manualControl_OrientationWorldIncremental();
            if (!isaid_manual_orientation_world_incremental)
            {
                cout << "Manual Orientation World Frame Incremental Mode" << endl;
                resetBools();
                isaid_manual_orientation_world_incremental = true;
            }

            // Setting the desired  orientation rosparam
            quats2rpy(orientation_desired, orientation_desired_rpy);
            node->set_parameter(rclcpp::Parameter("desired_orientation_rpy", orientation_desired_rpy));
        }

        else if (flight_mode == FlightModes::MANUAL_ORIENTATION_WORLD_ABSOLUTE)
        {

            if (!isaid_manual_orientation_world_absolute)
            {
                cout << "Manual Orientation World Frame Absolute Mode: " << endl;
                resetBools();
                isaid_manual_orientation_world_absolute = true;

                /// Explanation: In absolute case we want to stay fixed around a particular orientation which is the orientation at which the switch was flicked
                current_orientation_manual_absolute = orientation_current;
            }

            manualControl_OrientationWorldAbsolute(current_orientation_manual_absolute);

            // Setting the desired  orientation rosparam
            quats2rpy(orientation_desired, orientation_desired_rpy);
            node->set_parameter(rclcpp::Parameter("desired_orientation_rpy", orientation_desired_rpy));
        }

        else if (flight_mode == FlightModes::MANUAL_ORIENTATION_BODY_INCREMENTAL)
        {

            manualControl_OrientationBodyIncremental();

            if (!isaid_manual_orientation_body_incremental)
            {
                cout << "Manual Orientation Body Frame Incremental Mode: " << endl;
                resetBools();
                isaid_manual_orientation_body_incremental = true;
            }

            // Setting the desired  orientation rosparam
            quats2rpy(orientation_desired, orientation_desired_rpy);
            node->set_parameter(rclcpp::Parameter("desired_orientation_rpy", orientation_desired_rpy));
        }

        else if (flight_mode == FlightModes::MANUAL_ORIENTATION_BODY_ABSOLUTE)
        {

            if (!isaid_manual_orientation_body_absolute)
            {
                cout << "Manual Orientation Body Frame Absolute Mode: " << endl;
                resetBools();
                isaid_manual_orientation_body_absolute = true;
                current_orientation_manual_absolute = orientation_current;
            }

            manualControl_OrientationBodyAbsolute(current_orientation_manual_absolute);

            // Setting the desired  orientation rosparam
            node->set_parameter(rclcpp::Parameter("desired_orientation_rpy", orientation_desired_rpy));
        }

        else if (flight_mode == FlightModes::MANUAL_POSITION_INCREMENTAL)
        {

            manualControl_PositionIncremental();

            if (!isaid_manual_position_incremental)
            {
                cout << "Manual Position World Frame Incremental Mode: " << endl;
                resetBools();
                isaid_manual_position_incremental = true;
            }

            // Setting the desired position rosparam
            node->set_parameter(rclcpp::Parameter("desired_position", position_desired));
        }

        else if (flight_mode == FlightModes::MANUAL_POSITION_ABSOLUTE)
        {

            if (!isaid_manual_position_absolute)
            {
                cout << "Manual Position Absolute Mode: " << endl;
                resetBools();
                isaid_manual_position_absolute = true;
                current_position_manual_absolute[0] = position_current[0];
                current_position_manual_absolute[1] = position_current[1];
                current_position_manual_absolute[2] = position_current[2];
            }

            manualControl_PositionAbsolute(current_position_manual_absolute);
            node->set_parameter(rclcpp::Parameter("desired_position", position_desired));
        }

        ///////////////////////////////////////      OPEN LOOP /////////////////////////////////////////////////////////
        //////////////////////////////////////                 ////////////////////////////////////////////////////////
        else if (flight_mode == FlightModes::OPEN_LOOP_TORQUE_WORLD ||
                 flight_mode == FlightModes::OPEN_LOOP_TORQUE_BODY ||
                 flight_mode == FlightModes::OPEN_LOOP_FORCE)
        {

            // set the torque values and get the actuation vector from the in-program Actuation.cpp and set the actutation vector from it

            /// OPEN LOOP PROBLEM: the torque became zero because the difference in transmitter sticks is zero. so what you need to do is keep the moment same as before and then add on it.
            // getting a default value from the current moment being applied.
            if (flight_mode == FlightModes::OPEN_LOOP_TORQUE_WORLD)
            {

                if (!isaid_openloop_torque_world)
                {
                    cout << "Open Loop Torque World Frame" << endl;
                    resetBools();
                    isaid_openloop_torque_world = true;
                    getCurrentDefaultMoment(moment); // The moment needs to be taken only once when switched to open loop.
                }

                /// setting value of torque from transmitter depending on the difference i        mavros_msgs
                if (!isaid_openloop_torque_body)
                {
                    cout << "Open Loop: Torque Body" << endl;
                    resetBools();
                    isaid_openloop_torque_body = true;
                    getCurrentDefaultMoment(moment);
                }

                OpenLoopTorqueBody(moment);
            }

            if (flight_mode == FlightModes::OPEN_LOOP_FORCE)
            {

                if (!isaid_openloop_force)
                {
                    cout << "Open Loop: Force" << endl;
                    resetBools();
                    isaid_openloop_force = true;
                    getCurrentForce(force);
                }

                OpenLoopForce(force);
            }
            // Shady ifs that do not make sense
            //   if(FORCE_MODE){
            //       force[0] = force_ol[0];
            //       force[1] = force_ol[1];
            //       force[2] = force_ol[2];
            //
            //      moment[0] = 0;
            //      moment[1] = 0;
            //      moment[2] = 0;
            // }
            //
            // if(MOMENT_MODE){
            //     /*force[0] = 0;
            //     force[1] = 0;
            //     force[2] = 0;*/
            //
            //     force[0] = force_ol[0];
            //     force[1] = force_ol[1];
            //     force[2] = force_ol[2];
            //
            //
            //     moment[0] = torque_ol[0];
            //     moment[1] = torque_ol[1];
            //     moment[2] = torque_ol[2];
            // }

            Eigen::VectorXd u = getActuationVector(force, moment);
            double RPM[NUM_MOTORS];

            /// Limiting PWM values to the range 1350-1650
            for (int j = 0; j < NUM_MOTORS; j++)
            {
                double rpm = thrustToRPM(u(j));
                double pwm = convertToPWM(rpm);

                /// safe range of PWM-- limiting the values of PWM
                if (pwm < 1350)
                    pwm = 1350;
                else if (pwm > 1615)
                    pwm = 1615;

                /// Saving these values to the array
                RPM[j] = rpm;
                actuator_pwm_values[j] = pwm;
                // actuator_pwm_values[j]= 1520; //MOTOR TESTS /Should be commented
            }
        }

        // cout<<"PWM: "<<actuator_pwm_values[0]<<" "<<actuator_pwm_values[1]<<" "<<actuator_pwm_values[2]<<" "<<actuator_pwm_values[3]<<" "<<actuator_pwm_values[4]<<" "<<actuator_pwm_values[5]<<"\n ";

        // Force and torque to be published to open loop topic
        geometry_msgs::msg::Vector3Stamped opl_force, opl_torque;
        // opl_force.header.stamp = rclcpp::Time::now();
        opl_force.vector.x = force[0];
        opl_force.vector.y = force[1];
        opl_force.vector.z = force[2];

        // opl_torque.header.stamp = rclcpp::Time::now();
        opl_torque.vector.x = moment[0];
        opl_torque.vector.y = moment[1];
        opl_torque.vector.z = moment[2];

        pub_force->publish(opl_force);
        pub_torque->publish(opl_torque);

        // Send to the controller
        actuator_control_msg.header.stamp = node->get_clock()->now();
        actuator_control_msg.group_mix = 0;

        /// running a loop to send the PWM values to motor
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            /// Converting it to the system pixhawk understands---as 0 in input gives 1500 in output.
            // This values are between -1 and 1
            actuator_control_msg.controls[i] = (actuator_pwm_values[i] - 1500) / 500;
        }

        /// publishing the actuator pwms (assigned above) to the topic "mavros_msgs::ActuatorControl" controls the motors
        actuator_controls_pub->publish(actuator_control_msg);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    return 0;
}
