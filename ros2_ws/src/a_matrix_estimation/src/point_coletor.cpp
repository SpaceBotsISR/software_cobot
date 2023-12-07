#include "a_matrix_estimation/point_coletor.hpp"

PointColetor::PointColetor(std::string node_name, std::string file_name, bool use_intra_coms) : Node(node_name,
                                                                                                     rclcpp::NodeOptions().use_intra_process_comms(use_intra_coms)),
                                                                                                file_name(file_name)
{
    file = std::ofstream(file_name);

    //if (!file.is_open())
    //{
     //   RCLCPP_ERROR(this->get_logger(), "Could not open file %s", file_name.c_str());
     //   exit(EXIT_FAILURE);
    //}

    random_actuation_timer = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&PointColetor::random_actuation_generator, this));

    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    actuate_timer = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&PointColetor::actuate, this));

    actuation_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/desired_attuation", 10);

    using std::placeholders::_1;
   // imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    //    "/* topic_name */", 10, std::bind(&PointColetor ::imu_callback, this, _1));
}

PointColetor::~PointColetor()
{
    random_actuation_timer->cancel();
    file.close();
}

void PointColetor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
}

void PointColetor::random_actuation_generator()
{
    RCLCPP_INFO(get_logger(), "random actuation");
    // clear previus actuation
    random_actuation.clear();

    // generate new actuation  between -1 and 1
    for (int i = 0; i < 6; i++)
    {
        random_actuation.push_back( static_cast<double>(std::rand() * 2.0 ) / (RAND_MAX) - 1.0 );
    }
}

void PointColetor::actuate()
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data = random_actuation;
    actuation_pub->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointColetor>("point_coletor", "data.txt", true);
    rclcpp::spin(node);
    return 0;
}
