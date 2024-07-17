#include "space_cobot_interface/interface_controller.hpp"
#include<unistd.h>

InterfaceController::InterfaceController() : Node("interface_controller")
{
    RCLCPP_INFO(this->get_logger(), "Interface Controller Node Started");
    rc_subscriber = this->create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in", 10, std::bind(&InterfaceController::rc_callback, this, _1));

    // Convert to ros2 param and get from launch file instead
    std::string get_state_srv_name = "/interface/interface_slave/get_state";
    std::string change_state_srv_name = "/interface/interface_slave/change_state";

    this->client_get_state = this->create_client<lifecycle_msgs::srv::GetState>(get_state_srv_name);
    this->client_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_srv_name);

    // Configure and activate the interface for sending commands
    this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, std::chrono::seconds(3));
    sleep (3);
    this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, std::chrono::seconds(3));
    
    RCLCPP_INFO(get_logger(), "contructor_finished");
}

InterfaceController::~InterfaceController()
{
    RCLCPP_INFO(this->get_logger(), "Interface Controller Node Destroyed");
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

void InterfaceController::rc_callback(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
    // No shutdown dont in software, all is done in hardware
    return;
}

void InterfaceController::change_state_callback(const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
    RCLCPP_INFO(get_logger(), "Inside change state callback");
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
    auto future_result = client_change_state->async_send_request(
        request, std::bind(&InterfaceController::change_state_callback, this, _1));
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