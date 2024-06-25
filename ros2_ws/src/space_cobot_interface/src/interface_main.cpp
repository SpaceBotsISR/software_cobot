#include "space_cobot_interface/space_cobot_handle.hpp"

/**
 * @brief Main function for the Space Cobot Interface node
 * @param argc Number of arguments
 * @param argv Arguments
 * @return Exit status
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor   ; 
    auto node = std::make_shared<Space_Cobot_Interface>("space_cobot_interface", false);
    executor.add_node(node->get_node_base_interface());
    executor.spin();



    return 0;
}