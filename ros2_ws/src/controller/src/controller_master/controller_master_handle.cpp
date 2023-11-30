#include "controller_master/controller_master_handle.hpp"

ControllerMasterHandle::ControllerMasterHandle(std::string node_name) : Node(node_name)
{
    declare_publisher();
    declare_subscribers();
    declare_timer();

    desired_attitude = std::vector<double>(3);
    desired_translation = std::vector<double>(3);
}

ControllerMasterHandle::~ControllerMasterHandle()
{
}

void ControllerMasterHandle::declare_publisher()
{
    std::string topic_name = "desired_attuation";
    desired_attuation_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, 10);
}

void ControllerMasterHandle::declare_subscribers()
{
    std::string topic_name = "desired_attitude";
    desired_attitude_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(topic_name, 10, std::bind(&ControllerMasterHandle::desired_attitude_callback, this, _1));

    topic_name = "desired_translation";
    desired_translation_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(topic_name, 10, std::bind(&ControllerMasterHandle::desired_translation_callback, this, _1));
}

void ControllerMasterHandle::declare_timer()
{
    desired_atuation_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ControllerMasterHandle::publish_desired_attuation, this));
}

void ControllerMasterHandle::publish_desired_attuation()
{
    // Join both vector convert to pwm and publish
    Eigen::Vector3d desired_translation_(0, 0, 0);

    desired_translation_ << this->desired_translation[0], this->desired_translation[1], this->desired_translation[2];

    Eigen::Vector3d desired_attitude_(0, 0, 0);
    desired_attitude_ << this->desired_attitude[0], this->desired_attitude[1], this->desired_attitude[2];

    Eigen::VectorXd u = getActuationVector(desired_translation_, desired_attitude_);
    double RPM[NUM_MOTORS];
    double actuator_pwm_values[NUM_MOTORS];

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
    }
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.resize(6);

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        actuator_pwm_values[i] = (actuator_pwm_values[i] - 1500) / 500;
        actuator_pwm_values[i] = 100;
        msg.data[i] = actuator_pwm_values[i];
    }

    this->desired_attuation_pub->publish(msg);
}

void ControllerMasterHandle::desired_attitude_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    desired_attitude = std::move(msg->data);
}

void ControllerMasterHandle::desired_translation_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    desired_translation = msg->data;
}