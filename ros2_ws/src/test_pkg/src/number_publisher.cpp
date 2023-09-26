#include "rclcpp/rclcpp.hpp"

class NumberPublisherNode : public rclcpp::Node {
   public:
    NumberPublisherNode() : Node("number_publisher"), number(0) {
        number_timer = this->create_wall_timer(std::chrono::seconds(1),
                                               std::bind(&NumberPublisherNode::print_number, this));

        RCLCPP_INFO(this->get_logger(), "Node Started");
    }

   private:
    int number;
    rclcpp::TimerBase::SharedPtr number_timer;

    void print_number() {
        RCLCPP_INFO(this->get_logger(), "Number: %d", number);
        number++;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
