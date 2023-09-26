#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node {
   public:
    NumberPublisherNode() : Node("number_publisher"), number(0) {
        number_timer = this->create_wall_timer(std::chrono::seconds(1),
                                               std::bind(&NumberPublisherNode::publish_number, this));

        number_publisher = this->create_publisher<std_msgs::msg::Int64>("number_p", 10);

        RCLCPP_INFO(this->get_logger(), "Node Started");
    }

   private:
    int number;
    rclcpp::TimerBase::SharedPtr number_timer;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr number_publisher;

    void publish_number() {
        auto msg = std_msgs::msg::Int64();
        msg.data = number;
        number_publisher->publish(msg);
        number++;
        std::cout << "Number: " << number << std::endl;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
