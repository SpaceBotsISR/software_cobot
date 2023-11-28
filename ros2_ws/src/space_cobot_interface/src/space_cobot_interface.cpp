#include "space_cobot_interface/space_cobot_handle.hpp"

/// Inputs from the transmitter-- default neutral value being 1499.. These are used by manual_control, torque_control, force_control
int roll_input = 1499, pitch_input = 1499, yaw_input = 1499;

Space_Cobot_Interface::Space_Cobot_Interface(std::string node_name, bool intra_process_coms)
    : rclcpp_lifecycle::LifecycleNode(node_name,
                                      rclcpp::NodeOptions().use_intra_process_comms(intra_process_coms))
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Space_Cobot_Interface::on_configure(const rclcpp_lifecycle::State &state)
{
    LifecycleNode::on_configure(state);
    // Node Startup
    declare_publishers();
    declare_subscribers();
    declare_clients();

    rclcpp::Time start_wait = this->get_clock()->now();

    auto msg = std::make_shared<mavros_msgs::msg::State>(); //  :.auto msg = std::make_shared<mavros::msg::State>();
    auto node = std::make_shared<rclcpp::Node>("Space_cobot_interface_aux_node");
    do
    {
        rclcpp::wait_for_message(msg, node, "/mavros/state", std::chrono::milliseconds(500));

    } while (!msg->connected && this->get_clock()->now() - start_wait < rclcpp::Duration::from_seconds(3));

    if (!msg->connected)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to PX4");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    this->current_state = *msg;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Space_Cobot_Interface::on_activate(const rclcpp_lifecycle::State &state)
{
    // Make the publisher be active one again
    LifecycleNode::on_activate(state);

    // Some set points i need to send before arming px4
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //  Just gonna send one time and see if it works
    // for (int i = 100; i > 0; --i)
    //{
    this->local_pos_pub->publish(pose);
    //}

    // Make sure i keep setting px4 mode to offboard
    this->px4_arming_timer = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&Space_Cobot_Interface::set_mode_px4, this));

    if (!rclcpp::ok())
    {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for state subscriber to appear.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Space_Cobot_Interface::on_deactivate(const rclcpp_lifecycle::State &state)
{
    // Send zero force and torque one last time

    LifecycleNode::on_deactivate(state);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Space_Cobot_Interface::on_cleanup(const rclcpp_lifecycle::State &state)
{

    LifecycleNode::on_cleanup(state);
    reset_publishers();
    reset_subscribers();
    reset_clients();

    this->px4_arming_timer.reset();

    // Make sure PX4 is disarmed and in manual mode
    change_px4_arm_state(false);
    change_px4_custom_mode("MANUAL");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Space_Cobot_Interface::on_shutdown(const rclcpp_lifecycle::State &state)
{
    LifecycleNode::on_shutdown(state);
    this->px4_arming_timer.reset();

    reset_publishers();
    reset_subscribers();
    reset_clients();

    // Make sure PX4 is disarmed and in manual mode
    change_px4_arm_state(false);
    change_px4_custom_mode("MANUAL");

    // Send zero torque and force one last time
    // ! Still to do

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void Space_Cobot_Interface::declare_publishers()
{
    this->pub_force = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/open_loop/force", 1000);
    this->pub_torque = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/open_loop/torque", 1000);
    this->local_pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
    this->actuator_controls_pub = this->create_publisher<mavros_msgs::msg::ActuatorControl>("/mavros/actuator_control", 1000);
}

void Space_Cobot_Interface::declare_subscribers()
{
    this->state_sub = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&Space_Cobot_Interface::state_cb, this, _1));
    this->pwm_values = this->create_subscription<space_cobot_interface::msg::PwmValues>("/important_values", 1000, std::bind(&Space_Cobot_Interface::pwmValuesCallback, this, _1));
}

void Space_Cobot_Interface::declare_clients()
{
    this->arming_client = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    this->set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

void Space_Cobot_Interface::reset_publishers()
{
    this->pub_force.reset();
    this->pub_torque.reset();
    this->local_pos_pub.reset();
    this->actuator_controls_pub.reset();
}

void Space_Cobot_Interface::reset_subscribers()
{
    this->state_sub.reset();
    this->pwm_values.reset();
}

void Space_Cobot_Interface::reset_clients()
{
    this->arming_client.reset();
    this->set_mode_client.reset();
}

void Space_Cobot_Interface::pwmValuesCallback(const space_cobot_interface::msg::PwmValues::SharedPtr msg)
{
    mavros_msgs::msg::ActuatorControl actuator_control_msg;

    actuator_control_msg.header.stamp = this->get_clock()->now();
    actuator_control_msg.group_mix = 0;

    for (int i = 0; i < 6; i++)
    {
        // Convert to pixhawk system of -1 to 1, with 1500 == 0
        actuator_control_msg.controls[i] = (msg->pwm[i] - 1500) / 500;
    }

    this->actuator_controls_pub->publish(actuator_control_msg);
}

void Space_Cobot_Interface::state_cb(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state = *msg;
}

void Space_Cobot_Interface::set_mode_px4()
{
    if (current_state.mode == "OFFBOARD" && current_state.armed &&
        this->get_clock()->now() - last_arming_request < rclcpp::Duration::from_seconds(2.5))
    {
        return;
    }

    last_arming_request = this->get_clock()->now();

    if (current_state.mode != "OFFBOARD" && current_state.armed)
    {
        change_px4_custom_mode("OFFBOARD");
    }
    else if (!current_state.armed)
    {
        change_px4_arm_state(true);
    }
}

void Space_Cobot_Interface::change_px4_custom_mode(std::string new_mode)
{
    if (current_state.mode == new_mode)
    {
        return;
    }

    // Set the flight mode to offboard
    auto set_mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_req->custom_mode = new_mode;

    auto result = this->set_mode_client->async_send_request(std::move(set_mode_req));
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, std::chrono::milliseconds(50)) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Set mode sent %s", new_mode.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set mode");
    }
}

void Space_Cobot_Interface::change_px4_arm_state(bool arm)
{
    if (current_state.armed == arm)
    {
        return;
    }

    // Arm the drone
    auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_req->value = arm;

    auto arm_result = this->arming_client->async_send_request(arm_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), arm_result, std::chrono::milliseconds(50)) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Arming sent %s", arm_result.get()->success ? "OK" : "FAILED");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to arm");
    }
}
