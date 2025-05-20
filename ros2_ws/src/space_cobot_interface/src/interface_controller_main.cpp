#include "space_cobot_interface/interface_controller.hpp"
#include<unistd.h>

InterfaceController::InterfaceController() : Node("interface_controller")
{
    RCLCPP_INFO(this->get_logger(), "Interface Controller Node Started");
    kill_switch_sub = this->create_subscription<std_msgs::msg::Bool>("/key_pressed_topic", 10, std::bind(&InterfaceController::kill_switch_callback, this, _1));

    // Convert to ros2 param and get from launch file instead
    std::string get_state_srv_name = "/interface/interface_slave/get_state";
    std::string change_state_srv_name = "/interface/interface_slave/change_state";

    this->client_get_state = this->create_client<lifecycle_msgs::srv::GetState>(get_state_srv_name);
    this->client_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_srv_name);

   // this->get_state(std::chrono::seconds(3));
    this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, std::chrono::seconds(3));
    this->interface_already_running = false;

    this->kill_switch_timer = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&InterfaceController::kill_switch_timer_callback, this)
    );
    this->last_kill_switch_time = this->now();
    
    RCLCPP_INFO(get_logger(), "contructor_finished");
}

InterfaceController::~InterfaceController()
{
    RCLCPP_INFO(this->get_logger(), "Interface Controller Node Destroyed");
}


void InterfaceController::kill_switch_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->last_kill_switch_time = this->now();
    return;
}

template <typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
    FutureT &future,
    WaitTimeT time_to_wait)
{
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;
    do
    {
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0))
        {
            break;
        }
        status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
    } while (rclcpp::ok() && status != std::future_status::ready);
    return status;
}

void InterfaceController::change_state_callback(const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
    RCLCPP_INFO(get_logger(), "Inside change state callback");
}

void InterfaceController::kill_switch_timer_callback() {
    if (this->now() - this->last_kill_switch_time > rclcpp::Duration(std::chrono::milliseconds(100))){
        // Kill switch is no longer being pressed ->  Kill the motors and disarm
        if (interface_already_running) {
            RCLCPP_WARN(this->get_logger(), "Kill Switch Released, stopping robot and motors");
            change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
            node_restart = false;
            interface_already_running = false;
        }
    } else {
        if (interface_already_running) {
            return;
        } 
        
        if (first_start) {
            first_start = false;
            interface_already_running = true;
        }
        change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        interface_already_running = true;
        RCLCPP_WARN(this->get_logger(), "Kill Switch Pressed, starting robot and motors");
    }
    return;
}

bool InterfaceController::change_state(std::uint8_t transition, std::chrono::milliseconds timeout)
{
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state->wait_for_service(timeout))
    {
        RCLCPP_ERROR(
            get_logger(),
            "Service %s is not available.",
            client_change_state->get_service_name());
        return false;
    }

    using std::placeholders::_1;
    // We send the request with the transition we want to invoke.
    auto future_result = client_change_state->async_send_request(
        request, std::bind(&InterfaceController::change_state_callback, this, _1));
    
    //if ((this->get_node_base_interface(), future_result) == rclcpp::FutureReturnCode::SUCCESS) {
    //    RCLCPP_INFO(
    //        get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
    //    return true;
    //}
    //return false;

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    //auto future_status = wait_for_result(future_result, timeout);

    //if (future_status != std::future_status::ready)
    //{
    //    RCLCPP_ERROR(
    //        get_logger(), "Server time out while getting current state for node");
    //    return false;
    //}

    //// We have an answer, let's print our success.
    //if (future_result.get()->success)
    //{
    //    RCLCPP_INFO(
    //        get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
    //    return true;
    //}
    //else
    //{
    //    RCLCPP_WARN(
    //        get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
    //    return false;
    //}
}

unsigned int InterfaceController::get_state(std::chrono::milliseconds timeout)
{
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();



    if (!client_get_state->wait_for_service(timeout))
    {
        RCLCPP_ERROR(get_logger(), "SERVICE NOT AVAILABE %s", client_get_state->get_service_name());
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto future_result = this->client_get_state->async_send_request(std::move(request));
    auto future_status = wait_for_result(future_result, timeout);

    if (future_status != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "SERVICE TIMEOUT");
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if (future_result.get())
    {
        RCLCPP_INFO(get_logger(), "Space cobot interface current state is %s", future_result.get()->current_state.label.c_str());
        return future_result.get()->current_state.id;
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to get space cobot interface state");
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
}

int main(int argc, char **argv)
{
    // sleep for 3 seconds to let callbacks in slave start :)
    sleep(3);

    rclcpp::init(argc, argv);
    auto interface_controller = std::make_shared<InterfaceController>();
    rclcpp::spin (interface_controller);
    return 0;
}