#include "attitude_controller/attitude_controller_handle.hpp"

AttitudeControl::AttitudeControl(std::string node_name) : Node(node_name)
{
    declare_publisher();
    declare_subscribers();
    declare_timer();

    desired_attitude = std::vector<double>(3);

    // Check in remote after
    this->euler_angle_rc = new RCControl(0, 1, 3);
};

AttitudeControl::~AttitudeControl()
{
}

void AttitudeControl::declare_publisher()
{
    std::string topic_name = "desired_attuation";
    desired_attitude_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, 10);
}

void AttitudeControl::declare_subscribers()
{
    std::string topic_name = "/mavros/rc_in";
    sub_remote = this->create_subscription<mavros_msgs::msg ::RCIn>(topic_name, 10, std::bind(&AttitudeControl::rc_callback, this, _1));

    topic_name = "/mavros/imu/data";

    sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(topic_name, 10, std::bind(&AttitudeControl::imu_callback, this, _1));
}

void AttitudeControl::declare_timer()
{
    this->attitude_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&AttitudeControl::calculate_desired_attitude, this));
}

void AttitudeControl::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_data = msg;
    imu_quaternion = msg->orientation;
}

void AttitudeControl::rc_callback(const mavros_msgs::msg::RCIn::SharedPtr msg)
{

    flight_mode_switch = msg->channels[flight_mode_channel];

    flight_mode_switch = msg->channels[joystick_mode_channel];

    euler_angle_rc->update(msg);
}

void AttitudeControl::select_flight_mode()
{
    if (joystick_mode_switch == 1099)
    {
        flight_mode = BOARD;
    }
    else if (joystick_mode_switch == 1500)
    {
        if (toggle_position_orientation < 1500)
        {
            if (toggle_world_body_frames < 1500)
            {
                flight_mode = (incremental == 1) ? MANUAL_ORIENTATION_WORLD_INCREMENTAL : MANUAL_ORIENTATION_WORLD_ABSOLUTE;
            }
            else
            {
                flight_mode = (incremental == 1) ? MANUAL_ORIENTATION_BODY_INCREMENTAL : MANUAL_ORIENTATION_BODY_ABSOLUTE;
            }
        }
    }
    else if (joystick_mode_switch == 1901)
    {
        if (toggle_position_orientation < 1500)
        {
            flight_mode = (toggle_world_body_frames < 1500) ? OPEN_LOOP_TORQUE_WORLD : OPEN_LOOP_TORQUE_BODY;
        }
    }
}

void AttitudeControl::calculate_desired_attitude()
{
    switch (this->flight_mode)
    {
    case MANUAL_ORIENTATION_WORLD_INCREMENTAL:
        this->manual_orientation_world_incremental();
        break;
    case MANUAL_ORIENTATION_WORLD_ABSOLUTE:
        this->manual_orientation_world_absolute();
        break;
    case MANUAL_ORIENTATION_BODY_INCREMENTAL:
        this->manual_orientation_body_incremental();
        break;
    case MANUAL_ORIENTATION_BODY_ABSOLUTE:
        this->manual_orientation_body_absolute();
        break;
    case OPEN_LOOP_TORQUE_WORLD:
        this->open_loop_torque_world();
        break;
    case OPEN_LOOP_TORQUE_BODY:
        this->open_loop_torque_body();
        break;
    default:
        break;
    }
}

void AttitudeControl::publish_desired_attitude()
{
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = this->desired_attitude;
    this->desired_attitude_pub->publish(msg);
}

void AttitudeControl::manual_orientation_world_incremental()
{
    double rc_x_change = ((this->euler_angle_rc->get_roll() - 1499) * 0.00225 * (PI / 180)); // converts change in rc PWM values into change in angles
    double rc_y_change = ((this->euler_angle_rc->get_pitch() - 1499) * 0.00225) * (PI / 180);
    double rc_z_change = ((this->euler_angle_rc->get_yaw() - 1499) * 0.00225) * (PI / 180);

    Eigen::Matrix3d Ri;
    /// cv version of Ri and angle change for rodriguez formula
    cv::Mat cvRI(3, 3, CV_32FC1);
    cv::Mat cvChange(1, 3, CV_32FC1);

    /// Assigning angle change values to cvChange for rodriguez formula
    cvChange.ptr<float>(0)[0] = rc_x_change;
    cvChange.ptr<float>(0)[1] = rc_y_change;
    cvChange.ptr<float>(0)[2] = rc_z_change;

    cv::Rodrigues(cvChange, cvRI);

    /// Need to convert cvRI to Ri i.e the eigen matrix for easier multiplication
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Ri(i, j) = cvRI.ptr<float>(i)[j];
        }
    }

    Eigen::Quaterniond orientation_current = this->gm_quaternion_to_eigen(imu_quaternion);
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
    rc_Rd = Ri * rc_Rd;

    Eigen::Quaterniond orientation_desired_;
    Eigen::Quaterniond quatdes(rc_Rd);
    quatdes.normalize();
    orientation_desired_.x() = quatdes.x();
    orientation_desired_.y() = quatdes.y();
    orientation_desired_.z() = quatdes.z();
    orientation_desired_.w() = quatdes.w();

    this->desired_attitude = this->quaternion_to_vector(orientation_desired_);
}

void AttitudeControl::manual_orientation_body_incremental()
{
    double rc_x_change = ((this->euler_angle_rc->get_roll() - 1497) * 0.00225 * (PI / 180)); // converts change in rc PWM values into change in angles in radians
    double rc_y_change = ((this->euler_angle_rc->get_pitch() - 1498) * 0.00225) * (PI / 180);
    double rc_z_change = ((this->euler_angle_rc->get_yaw() - 1499) * 0.00225) * (PI / 180);

    Eigen::Matrix3d Ri;

    cv::Mat cvRI(3, 3, CV_32FC1);
    cv::Mat cvChange(1, 3, CV_32FC1);

    /// Assigning angle change values to cvChange for rodriguez formula
    cvChange.ptr<float>(0)[0] = rc_x_change;
    cvChange.ptr<float>(0)[1] = rc_y_change;
    cvChange.ptr<float>(0)[2] = rc_z_change;

    /// The main Rodriguez Formulaf
    cv::Rodrigues(cvChange, cvRI);

    /// Need to convert cvRI to Ri i.e the eigen matrix for easier multiplication
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Ri(i, j) = cvRI.ptr<float>(i)[j];
        }
    }

    Eigen::Quaterniond orientation_current = this->gm_quaternion_to_eigen(imu_quaternion);
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

    Eigen::Quaterniond orientation_desired_;
    orientation_desired_.x() = quatdes.x();
    orientation_desired_.y() = quatdes.y();
    orientation_desired_.z() = quatdes.z();
    orientation_desired_.w() = quatdes.w();

    this->desired_attitude = this->quaternion_to_vector(orientation_desired_);
}

void AttitudeControl::manual_orientation_body_absolute()
{                                                                                   /// We get change in desired angles (rpy) in euler angles (degrees)
    double roll_angle_change = ((this->euler_angle_rc->get_roll() - 1499) * 0.225); /// it is multiplied by a constant (90/400) because I want a full stick up(or down) to mean 90 degrees in angle change. Also, the range of stick movement in one side is 400 (1100-1500). Hence each unit of stick movement change should give stick_input*(90/400) in angle change. Hence Result!!!
    double pitch_angle_change = ((this->euler_angle_rc->get_pitch() - 1499) * 0.225);
    double yaw_angle_change = ((this->euler_angle_rc->get_yaw() - 1499) * 0.225);

    Eigen::Quaternion current_orientation_manual_absolute = this->gm_quaternion_to_eigen(imu_quaternion);

    tf2::Quaternion q_temp_cur;

    q_temp_cur.setX(current_orientation_manual_absolute.x());
    q_temp_cur.setY(current_orientation_manual_absolute.y());
    q_temp_cur.setZ(current_orientation_manual_absolute.z());
    q_temp_cur.setW(current_orientation_manual_absolute.w());

    tf2::Matrix3x3 mat1(q_temp_cur);
    tf2Scalar roll_cur = 0, pitch_cur = 0, yaw_cur = 0;
    mat1.getEulerYPR(yaw_cur, pitch_cur, roll_cur);

    desired_attitude[0] = roll_cur * (180 / PI) + roll_angle_change; // roll_curr is changed to degrees before adding to change (also in degrees) and making orientation_desired (degrees)
    desired_attitude[1] = pitch_cur * (180 / PI) + pitch_angle_change;
    desired_attitude[2] = yaw_cur * (180 / PI) + yaw_angle_change;
}

void AttitudeControl::manual_orientation_world_absolute()
{
    double rc_x_change = ((this->euler_angle_rc->get_roll() - 1497) * 0.225 * (PI / 180)); // converts change in rc PWM values into change in angles in radians
    double rc_y_change = ((this->euler_angle_rc->get_pitch() - 1498) * 0.225) * (PI / 180);
    double rc_z_change = ((this->euler_angle_rc->get_yaw() - 1499) * 0.225) * (PI / 180);

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
    const geometry_msgs::msg::Quaternion &imu_quaternion_ = imu_quaternion;
    Eigen::Quaternion current_orientation_manual_absolute = this->gm_quaternion_to_eigen(imu_quaternion_);

    if (rc_flag == 1)
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

    Eigen::Quaterniond orientation_desired;
    orientation_desired.x() = quatdes.x();
    orientation_desired.y() = quatdes.y();
    orientation_desired.z() = quatdes.z();
    orientation_desired.w() = quatdes.w();

    this->desired_attitude = this->quaternion_to_vector(orientation_desired);
}

void AttitudeControl::open_loop_torque_world()
{
    auto normalize = [](double input)
    { return (input - 1499) * 0.000125; };

    std::vector<double> M = {
        normalize(this->euler_angle_rc->get_roll()),
        normalize(this->euler_angle_rc->get_pitch()),
        normalize(this->euler_angle_rc->get_yaw())};

    std::vector<double> current_orientation_ = this->quaternion_to_vector(imu_quaternion);

    for (int i = 0; i < 3; i++)
    {
        this->desired_attitude[i] = M[i] + current_orientation_[i];
    }
}

void AttitudeControl::open_loop_torque_body()
{
    auto normalize = [](double input)
    { return (input < 0.00008 && input > -0.00008) ? 0 : input; };

    double x_change = normalize((this->euler_angle_rc->get_roll() - 1499) * 0.000125);
    double y_change = normalize((this->euler_angle_rc->get_pitch() - 1499) * 0.000125);
    double z_change = normalize((this->euler_angle_rc->get_yaw() - 1499) * 0.000125);

    std::vector<double> desired_orientation_ = {x_change, y_change, z_change};
    std::vector<double> current_orientation_ = this->quaternion_to_vector(imu_quaternion);

    for (int i = 0; i < 3; i++)
    {
        this->desired_attitude[i] = desired_orientation_[i] + current_orientation_[i];
    }

    // Use M array as needed...
}

Eigen::Quaterniond AttitudeControl::gm_quaternion_to_eigen(const geometry_msgs::msg::Quaternion &imu_quat)
{
    return Eigen::Quaterniond(imu_quat.w, imu_quat.x, imu_quat.y, imu_quat.z);
}

std::vector<double> AttitudeControl::quaternion_to_vector(tf2::Quaternion &tf_quaternion)
{
    Eigen::Vector3d rpy = Eigen::Quaterniond(tf_quaternion.w(), tf_quaternion.x(), tf_quaternion.y(), tf_quaternion.z())
                              .toRotationMatrix()
                              .eulerAngles(0, 1, 2);

    return {rpy[0], rpy[1], rpy[2]};
}

std::vector<double> AttitudeControl::quaternion_to_vector(Eigen::Quaterniond q)
{
    tf2::Quaternion q_temp_des, q_temp_cur;
    std::vector<double> desired_attitude_(3);

    q_temp_des.setX(q.x());
    q_temp_des.setY(q.y());
    q_temp_des.setZ(q.z());
    q_temp_des.setW(q.w());

    tf2::Matrix3x3 mat2(q_temp_des);
    tf2Scalar roll_des = 0, pitch_des = 0, yaw_des = 0;
    mat2.getEulerYPR(yaw_des, pitch_des, roll_des);

    desired_attitude_[0] = roll_des * (180 / PI);
    desired_attitude_[1] = pitch_des * (180 / PI);
    desired_attitude_[2] = yaw_des * (180 / PI);

    return desired_attitude_;
}

std::vector<double> AttitudeControl::quaternion_to_vector(geometry_msgs::msg::Quaternion gm_quaternion)
{
    Eigen::Vector3d rpy = Eigen::Quaterniond(gm_quaternion.w, gm_quaternion.x, gm_quaternion.y, gm_quaternion.z)
                              .toRotationMatrix()
                              .eulerAngles(0, 1, 2);

    return {rpy[0], rpy[1], rpy[2]};
}
