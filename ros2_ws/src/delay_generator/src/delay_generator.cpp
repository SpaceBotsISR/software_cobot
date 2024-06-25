#include "delay_generator/delay_generator.hpp"
#include <chrono>
#include <cstdio>
#include <iterator>
#include <string>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

DelayGenerator::DelayGenerator(std::string node_name,
							   std::string json_file_path)
	: Node(node_name), json_file_path_(json_file_path)
{
	//  Create the necessary callback groups that will be used here
	timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	subscriber_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	// Load Json
	std::ifstream f(this->json_file_path_);

	// Try to parse the json file
	try
	{
		this->j_ = json::parse(f);
	}
	catch (...)
	{
		// Convert std::cout to RCLCPP_ERROR
		std::cout << "Error in parsing file - ending program " << std::endl;
		rclcpp::shutdown();
		exit(1);
	}

	//  compute desired delay time
	int time_ms = static_cast<int>(j_["delay_information"]["delay_time"]["seconds"]) * 1000 +
				  static_cast<int>(j_["delay_information"]["delay_time"]["milliseconds"]);
	this->delay_time_ = std::chrono::milliseconds(time_ms);

	// Decode the json file to discover all the topics that should be delayed
	for (auto const &t : j_["delay_information"]["delay_topics"])
	{
		std::string original_topic = static_cast<std::string>(t["original_topic"]);
		std::string delayed_topic = static_cast<std::string>(t["delayed_topic"]);
		this->topics[original_topic] = delayed_topic;
	}

	create_all_subscriptions();
}

/*
BagRecorder::BagRecorder(rclcpp::Node::SharedPtr parent_node) : parent_node(parent_node)
{
	for (auto const & topic : this->parent_node->get_topic_names_and_types()) {
		this->sub.push_back(this->parent_node->create_generic_subscription(topic.first, topic.second[0], 10,
			[this, name = topic.first, type = topic.second[0]](std::shared_ptr<rclcpp::SerializedMessage> msg){ // Subscription callback
				if (this->state == STANDBY) {
					return;
				}

				this->writer->write(msg, name, type, this->parent_node->now());
			}));
	}
}
GenericPublisher::SharedPtr pub = rclcpp_generic::GenericPublisher::create(
  get_node_topics_interface(),
  "/my_node/velocity",
  "geometry_msgs/Twist",
  rclcpp::QoS{1});

*/
bool DelayGenerator::create_all_subscriptions()
{
	size_t number_of_delayed_topics = 0;

	auto callback = [this](std::string topic_name, std::string topic_type) {

	};

	for (auto const &t : this->get_topic_names_and_types())
	{
		if (this->topics.count(t.first))
		{
			std::cout << t.first << std::endl;
			std::cout << t.second[0] << std::endl;
			std::cout << this->topics[t.first] << std::endl;

			auto imu_QoS = rclcpp::QoS(10);
			imu_QoS.best_effort();
			imu_QoS.durability_volatile();

			// topic name    topic type   qos
			auto pub = this->create_generic_publisher(this->topics[t.first], t.second[0], imu_QoS);
			this->Publishers[t.first] = pub;

			auto sub = this->create_generic_subscription(t.first, t.second[0], imu_QoS,
														 [this, pub](std::shared_ptr<rclcpp::SerializedMessage> msg)
														 {
															 std::cout << "inside first lambda function" << std::endl;
															 auto delayed_msg = std::make_shared<DelayedMessage>(msg, pub);
															 auto timer = this->create_wall_timer(this->delay_time_, [this, delayed_msg]()
																								  {
																									std::cout << "timer" << std::endl;
																delayed_msg->publish_message();
																this->delayed_messages_map_[delayed_msg].reset();
																auto it = this->delayed_messages_map_.find(delayed_msg);
																if (it != this->delayed_messages_map_.end())
																	this->delayed_messages_map_.erase(it); });
															 this->delayed_messages_map_[delayed_msg] = timer;
														 });

			this->Subscribers[t.first] = sub;
			number_of_delayed_topics++;
		}
	}

	RCLCPP_WARN(this->get_logger(), "The number of topics that will be delayed is %ld", number_of_delayed_topics);

	return static_cast<bool>(number_of_delayed_topics == this->topics.size());
}

int main(int argc, char *argv[])
{
	std::string node_name = "delay_generator";
	std::string json_file_path = "/cobot/src/delay_generator/configuration/topics.json";

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DelayGenerator>(node_name, json_file_path));
}
