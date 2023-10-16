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

using namespace std;

// desired pose callback function
void desiredPoseCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg)
{
}

// desired twist callback function
void desiredTwistCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg)
{
}

/// Current Orientation, Current Acceleration, Current Angular velocity
/// The Eigen::Quaternion is (w, x, y, z)
Eigen::Quaterniond orientation_current(1, 0, 0, 0);
vector<double> acceleration_current(3), angular_velocity_current(3), position_current(3), velocity_current(3);

/// Desired Orientation, Desired position, Desired velocity
Eigen::Quaterniond orientation_desired(1, 0, 0, 0);
vector<double> orientation_desired_rpy(3), acceleration_desired(3), angular_velocity_desired(3), position_desired(3), velocity_desired(3);
// Eigen:.Vector3d acceleration_desired(0,0,0), angular_velocity_desired(0,0,0), position_desired(0,0,0), velocity_desired(0,0,0);

// Attitude gains
vector<double> att_p_gain{0.25, 0.25, 0.25}, att_i_gain{0.1, 0.1, 0.1}, att_d_gain{0.1, 0.1, 0.1};

// Position gains
vector<double> pos_p_gain{2.02, 2.02, 2.02}, pos_i_gain{0.1, 0.1, 0.1}, pos_d_gain{10.32, 10.32, 10.32};

// Flight modes for the rosparam server
int FLIGHT_MODE = 0;

// time step
double time_step = 0.05;

// integration error
Eigen::Vector3d err_int(0.0, 0.0, 0.0);

// integration windup limits
float windup_limit_up = 1, windup_limit_down = -1;

// force mode
bool force_mode = false;

int kill_switch = 1099;

void IMUCallback(const sensor_msgs::msg::Imu::ConstPtr &msg)
{

    orientation_current.x() = msg->orientation.x;
    orientation_current.y() = msg->orientation.y;
    orientation_current.z() = msg->orientation.z;
    orientation_current.w() = msg->orientation.w;

    angular_velocity_current[0] = msg->angular_velocity.x;
    angular_velocity_current[1] = msg->angular_velocity.y;
    angular_velocity_current[2] = msg->angular_velocity.z;
}

void mocapCallback(const mocap_interface::msg::MocapMsg &msg)
{

    /*orientation_current.x() = msg.pose.pose.orientation.x;
    orientation_current.y() = msg.pose.pose.orientation.y;
    orientation_current.z() = msg.pose.pose.orientation.z;
    orientation_current.w() = msg.pose.pose.orientation.w;*/

    if (POSITION)
    {
        position_current[0] = msg.pose.pose.position.x;
        position_current[1] = msg.pose.pose.position.y;
        position_current[2] = msg.pose.pose.position.z;

        velocity_current[0] = msg.velocity.twist.linear.x;
        velocity_current[1] = msg.velocity.twist.linear.y;
        velocity_current[2] = msg.velocity.twist.linear.z;

        acceleration_current[0] = msg.acceleration.twist.linear.x;
        acceleration_current[1] = msg.acceleration.twist.linear.y;
        acceleration_current[2] = msg.acceleration.twist.linear.z;
    }

    else
    {

        position_current[0] = 0;
        position_current[1] = 0;
        position_current[2] = 0;

        velocity_current[0] = 0;
        velocity_current[1] = 0;
        velocity_current[2] = 0;

        acceleration_current[0] = 0;
        acceleration_current[1] = 0;
        acceleration_current[2] = 0;
    }
}

void vrpnCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg)
{
    orientation_current.x() = msg->pose.orientation.x;
    orientation_current.y() = msg->pose.orientation.y;
    orientation_current.z() = msg->pose.orientation.z;
    orientation_current.w() = msg->pose.orientation.w;
}

void rc_controlCallback(const mavros_msgs::msg::RCIn::ConstPtr &msg)
{

    kill_switch = msg->channels[4];
}

void checkKill(Eigen::Vector3d &err_int, vector<double> &des_orientation_rpy)
{
    if (kill_switch == 1099)
        return;
    else if (kill_switch == 1901)
    {
        err_int = {0, 0, 0};
        des_orientation_rpy = {0, 0, 0};
    }
}

void getGainsAng(Eigen::Matrix3d &K_P, Eigen::Matrix3d &K_I, Eigen::Matrix3d &K_D, vector<double> &att_p_gain, vector<double> &att_i_gain, vector<double> &att_d_gain)
{

    K_P = Eigen::DiagonalMatrix<double, 3>(att_p_gain[0], att_p_gain[1], att_p_gain[2]);
    K_I = Eigen::DiagonalMatrix<double, 3>(att_i_gain[0], att_i_gain[1], att_i_gain[2]);
    K_D = Eigen::DiagonalMatrix<double, 3>(att_d_gain[0], att_d_gain[1], att_d_gain[2]);
}

void getGainsPos(Eigen::Matrix3d &K_P, Eigen::Matrix3d &K_I, Eigen::Matrix3d &K_D, vector<double> &pos_p_gain, vector<double> &pos_i_gain, vector<double> &pos_d_gain)
{

    K_P = Eigen::DiagonalMatrix<double, 3>(pos_p_gain[0], pos_p_gain[1], pos_p_gain[2]);
    K_I = Eigen::DiagonalMatrix<double, 3>(pos_i_gain[0], pos_i_gain[1], pos_i_gain[2]);
    K_D = Eigen::DiagonalMatrix<double, 3>(pos_d_gain[0], pos_d_gain[1], pos_d_gain[2]);
}

double wrap(double x)
{
    return x - 2 * pi * floor(x / (2 * pi) + 0.5);
}

void computeQuaterror(Eigen::Vector3d &err_Quat)
{
    /// In this function, the error between orientation_curr and orientation_des will be calculated and saved in an array for publishing
    Eigen::Quaterniond error_Quaternion;
    error_Quaternion = orientation_desired * (orientation_current.inverse());

    err_Quat[0] = wrap(2 * (acos(error_Quaternion.w())));
    err_Quat[1] = 2 * 0 * (asin(error_Quaternion.y()));
    err_Quat[2] = 2 * 0 * (asin(error_Quaternion.z()));
}

void quats2rpy(space_cobot_interface::msg::PwmValues &Data)
{
    /// Converting the orientations back to quaternions
    tf2::Quaternion q_temp_des, q_temp_cur;

    q_temp_des.setX(orientation_desired.x());
    q_temp_des.setY(orientation_desired.y());
    q_temp_des.setZ(orientation_desired.z());
    q_temp_des.setW(orientation_desired.w());

    q_temp_cur.setX(orientation_current.x());
    q_temp_cur.setY(orientation_current.y());
    q_temp_cur.setZ(orientation_current.z());
    q_temp_cur.setW(orientation_current.w());

    tf2::Matrix3x3 mat1(q_temp_cur);
    tf2Scalar roll_cur = 0, pitch_cur = 0, yaw_cur = 0;
    mat1.getEulerYPR(yaw_cur, pitch_cur, roll_cur);

    tf2::Matrix3x3 mat2(q_temp_des);
    tf2Scalar roll_des = 0, pitch_des = 0, yaw_des = 0;
    mat2.getEulerYPR(yaw_des, pitch_des, roll_des);

    /// The yaw, pitch, roll we get from the above function is in radians, so we need to convert it to degrees before putting it in the ros message
    Data.current_rpy[0] = roll_cur * (180 / pi);
    Data.current_rpy[1] = pitch_cur * (180 / pi);
    Data.current_rpy[2] = yaw_cur * (180 / pi);

    Data.desired_rpy[0] = roll_des * (180 / pi);
    Data.desired_rpy[1] = pitch_des * (180 / pi);
    Data.desired_rpy[2] = yaw_des * (180 / pi);

    Data.error_rpy[0] = Data.desired_rpy[0] - Data.current_rpy[0];
    Data.error_rpy[1] = Data.desired_rpy[1] - Data.current_rpy[1];
    Data.error_rpy[2] = Data.desired_rpy[2] - Data.current_rpy[2];
}

void assign_data(space_cobot_interface::msg::PwmValues &Data, Eigen::Vector3d force, Eigen::Vector3d M, vector<double> att_p_gain, vector<double> att_i_gain, vector<double> att_d_gain, vector<double> des_position, vector<double> des_velocity, vector<double> des_ang_vel, Eigen::VectorXd u, double PWM[6], double RPM[6], double time_step, Eigen::Vector3d err_int, Eigen::Vector3d err_Quat)
{

    quats2rpy(Data);

    for (int i = 0; i < 3; i++)
    {
        Data.force[i] = force[i];
        Data.moment[i] = M[i];
        Data.att_p_gains[i] = att_p_gain[i];
        Data.att_i_gains[i] = att_i_gain[i];
        Data.att_d_gains[i] = att_d_gain[i];

        Data.pos_p_gains[i] = pos_p_gain[i];
        Data.pos_d_gains[i] = pos_d_gain[i];

        Data.desired_position[i] = des_position[i];
        Data.desired_velocity[i] = des_velocity[i];
        Data.desired_omega[i] = des_ang_vel[i];
        Data.err_int[i] = err_int[i];
        Data.quat_error[i] = err_Quat[i];
    }

    for (int i = 0; i < 6; i++)
    {
        Data.actuation[i] = u(i);
        Data.pwm[i] = PWM[i];
        Data.rpm[i] = RPM[i];
    }

    Data.time_step = time_step;
}

void getTargetOrientation(vector<double> des_orientation_rpy)
{

    /// converting degrees to radians for use in equations
    for (int i = 0; i < 3; i++)
    {
        des_orientation_rpy[i] = des_orientation_rpy[i] * (pi / 180);
    }

    /// converting the target orientation in rpy to quaternion and assigning it
    tf2::Quaternion q_temp;
    q_temp.setRPY(des_orientation_rpy[0], des_orientation_rpy[1], des_orientation_rpy[2]);
    // q_temp.normalize();

    orientation_desired.x() = q_temp.x();
    orientation_desired.y() = q_temp.y();
    orientation_desired.z() = q_temp.z();
    orientation_desired.w() = q_temp.w();
}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("space_cobot_controller");

    // Publishing and Subscribing

    // Subscribers to get target pose and twist
    auto sub_des_pose = node->create_subscription<geometry_msgs::msg::PoseStamped>("/space_cobot_interface/desired_pose", 10, desiredPoseCallback);
    auto sub_des_twist = node->create_subscription<geometry_msgs::msg::PoseStamped>("/space_cobot_interface/desired_twist", 10, desiredTwistCallback);
    // Subscribe the RC controller inputs
    auto fmode = node->create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in", 1000, rc_controlCallback);

    // Subscribers to get current pose
    // TODO(ALG): Shift this to values from the Mocap or the localization node.
    // TODO(ALG): Get the position and orientation values from the rostopic data itself.
    auto sub_current_pose = node->create_subscription<sensor_msgs::msg::Imu>("mavros/imu/data", 1000, IMUCallback);
    // ros::Subscriber sub_vrpn = nh.subscribe("/vrpn_client_node/space_cobot_gt/pose", vrpnCallback);

    auto sub_current_position = node->create_subscription<mocap_interface::msg::MocapMsg>("/space_cobot/mocap_interface/data", 1000, mocapCallback);

    auto imp_values = node->create_publisher<space_cobot_interface::msg::PwmValues>("important_values", 1000);

    auto current_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 1000);

    rclcpp::Rate rate(20);

    // Force and Moment variables
    Eigen::Vector3d force(0, 0, 0);
    Eigen::Vector3d moment(0, 0, 0);

    // Controller Gains
    // position
    Eigen::Matrix3d KXp, KXd;
    // Orientation -- these values will be changed by the rosparam dynamically.
    Eigen::Matrix3d att_K_P = Eigen::DiagonalMatrix<double, 3>(0.1, 0.1, 0.1);
    Eigen::Matrix3d att_K_I = Eigen::DiagonalMatrix<double, 3>(0.0, 0.0, 0.0);
    Eigen::Matrix3d att_K_D = Eigen::DiagonalMatrix<double, 3>(0.0, 0.0, 0.0);

    // Position
    // Orientation
    Eigen::Matrix3d pos_K_P = Eigen::DiagonalMatrix<double, 3>(0.15, 0.15, 0.15);
    Eigen::Matrix3d pos_K_I = Eigen::DiagonalMatrix<double, 3>(0.1, 0.1, 0.1);
    Eigen::Matrix3d pos_K_D = Eigen::DiagonalMatrix<double, 3>(0.1, 0.1, 0.1);

    /// Initialising the rosparams
    /// commenting position and velocity rosparam for now for safety
    /// this is incorporated in the getGains() where these are made into a diagonal matrix for using in the Moment function
    auto orientation_desired_rpy = node->declare_parameter<vector<double>>("desired_orientation_rpy", orientation_desired_rpy);
    auto angular_velocity_desired = node->declare_parameter<vector<double>>("desired_angular_velocity", angular_velocity_desired);
    auto position_desired = node->declare_parameter<vector<double>>("desired_position", {0.0, 0.0, 0.0});
    auto velocity_desired = node->declare_parameter<vector<double>>("desired_velocity", {0.0, 0.0, 0.0});
    auto att_p_gain = node->declare_parameter<vector<double>>("AttPGain", att_p_gain);
    auto att_i_gain = node->declare_parameter<vector<double>>("AttIGain", att_i_gain);
    auto att_d_gain = node->declare_parameter<vector<double>>("AttDGain", att_d_gain);
    auto time_step = node->declare_parameter<float>("TimeStep", time_step);
    auto force_mode = node->declare_parameter<bool>("ForceMode", force_mode);
    auto pos_p_gain = node->declare_parameter<vector<double>>("PosPGain", pos_p_gain);
    auto pos_i_gain = node->declare_parameter<vector<double>>("PosIGain", pos_i_gain);
    auto pos_d_gain = node->declare_parameter<vector<double>>("PosDGain", pos_d_gain);

    while (rclcpp::ok())
    {
        /// getting all the rosparams
        node->get_parameter("desired_position", position_desired);
        node->get_parameter("desired_velocity", velocity_desired);
        node->get_parameter("desired_orientation_rpy", orientation_desired_rpy);
        node->get_parameter("desired_angular_velocity", angular_velocity_desired);
        node->get_parameter("AttPGain", att_p_gain);
        node->get_parameter("AttIGain", att_i_gain);
        node->get_parameter("AttDGain", att_d_gain);
        node->get_parameter("TimeStep", time_step);
        node->get_parameter("ForceMode", force_mode);

        node->get_parameter("PosPGain", pos_p_gain);
        node->get_parameter("PosIGain", pos_i_gain);
        node->get_parameter("PosDGain", pos_d_gain);

        // Getting target orientation from the rosparam to global variable quaternion
        getTargetOrientation(orientation_desired_rpy);

        /********************POSITION CONTROL****************************************
        ****************************************************************************/

        // Position desried is directly set from the rosparam
        // Get target gains position
        getGainsPos(pos_K_P, pos_K_I, pos_K_D, pos_p_gain, pos_i_gain, pos_d_gain);

        /// converting Eigen::quaternion to RowVector4d to rotation matrix
        Eigen::RowVector4d q_current = Eigen::RowVector4d(orientation_current.w(), orientation_current.x(), orientation_current.y(), orientation_current.z());
        Eigen::Matrix3d rotation_matrix = getRotationMatrix(q_current);

        position_desired[1] = position_current[1];

        // force = positionController(position_current, position_desired, velocity_current, velocity_desired, rotation_matrix, KXp,KXd);
        force = positionController(position_current, position_desired, velocity_current, velocity_desired, rotation_matrix, pos_K_P, pos_K_D);

        /**********************ATTITUDE CONTROL****************************************
        ******************************************************************************/

        // converting Eigen::quaternion to RowVector4d
        Eigen::RowVector4d q_desired = Eigen::RowVector4d(orientation_desired.w(), orientation_desired.x(), orientation_desired.y(), orientation_desired.z());
        Eigen::Vector3d omega_curr = Eigen::RowVector3d(angular_velocity_current[0], angular_velocity_current[1], angular_velocity_current[2]);
        Eigen::Vector3d omega_des = Eigen::RowVector3d(angular_velocity_desired[0], angular_velocity_desired[1], angular_velocity_desired[2]);
        Eigen::Vector3d omegaD_des = Eigen::Vector3d(0, 0, 0);

        getGainsAng(att_K_P, att_K_I, att_K_D, att_p_gain, att_i_gain, att_d_gain);

        // In order to solve the problem of the cobot bouncing back with high throttle when swithcing back on from kill switch (due to integration of error during the inactivity period when kill switch is deployed which BTW also happens when switching on for the first time), its better if there is function that checks if the kill switch is deployed and puts err_int=0, and also desired orienation at (0, 0, 0), so you know where it goes when coming back from kill. This will improve predictability of the cobot when recovering from kill
        checkKill(err_int, orientation_desired_rpy);
        /// Calculating moment
        moment = attitudeController(q_current, q_desired, omega_curr, omega_des, omegaD_des, att_K_P, att_K_D, err_int, att_K_I, time_step, windup_limit_up, windup_limit_down);

        /************************ACTUATION VALUES ************************************************
         **************************************************************************************/
        Eigen::VectorXd u = getActuationVector(force, moment);
        int NUM_MOTORS = 6;
        double RPM[6], PWM[6];

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
            PWM[j] = pwm;
        }

        space_cobot_interface::msg::PwmValues Data;
        Data.header.stamp = node->get_clock()->now();

        /// function assigns the message "Data" all the important values for reference
        Eigen::Vector3d err_Quat;

        computeQuaterror(err_Quat);
        // cout<<"Error Quaternion is "<<err_Quat<<endl;
        assign_data(Data, force, moment, att_p_gain, att_i_gain, att_d_gain, position_desired, velocity_desired, angular_velocity_desired, u, PWM, RPM, time_step, err_int, err_Quat);

        imp_values->publish(Data);

        geometry_msgs::msg::PoseStamped current_pose_msg;
        current_pose_msg.header.stamp = node->get_clock()->now();
        current_pose_msg.pose.position.x = position_current[0];
        current_pose_msg.pose.position.y = position_current[1];
        current_pose_msg.pose.position.z = position_current[2];

        current_pose_msg.pose.orientation.x = orientation_current.x();
        current_pose_msg.pose.orientation.y = orientation_current.y();
        current_pose_msg.pose.orientation.z = orientation_current.z();
        current_pose_msg.pose.orientation.w = orientation_current.w();

        current_pose_pub->publish(current_pose_msg);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    return 0;
}
