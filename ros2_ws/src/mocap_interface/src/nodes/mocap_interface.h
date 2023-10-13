#ifndef rotors_mcs_MSC_NODE_H
#define rotors_mcs_MSC_NODE_H

#include <boost/bind.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <math.h>
#include <vector>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <time.h>

#include <rclcpp/rclcpp.hpp>

#include <common.h>
#include <eigen_mav_msgs.h>

#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "mocap_interface/msg/mocap_msg.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mocap_interface_ns
{

	static const float kDefaultDt = 0.02;
	static const float kDefaultNSamples = 30;
	static const float kDefaultRateOptitrack = 200;

	static const std::string kDefaultTopicListen = "/vrpn_client_node/space_cobot/pose";
	// Since a message file with all the necessary values (pose, velocity and acceleration are used hence only one topic is sufficient)
	static const std::string kDefaultTopicPublish = "/space_cobot/mocap_interface/data";

	class MocapInterface : public rclcpp::Node
	{
	public:
		MocapInterface();
		~MocapInterface();

	private:
		float dt_;
		int n_samples_;
		std::string topic_listen;
		std::string topic_publish;

		int MotionRegression1D(std::vector<std::vector<double>> x, std::vector<rclcpp::Time> t,
							   std::vector<double> &v, std::vector<double> &a);
		int MotionRegression6D(std::vector<std::vector<double>> p, std::vector<std::vector<double>> q,
							   std::vector<rclcpp::Time> t, std::vector<double> &v, std::vector<double> &w,
							   std::vector<double> &a, std::vector<double> &alpha);

		void PoseTimeSequenceRegression();
		void PoseCallback(const geometry_msgs::msg::PoseStamped &msg);

		rclcpp::Publisher<mocap_interface::msg::MocapMsg>::SharedPtr pub_values_;

		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr timer_mcs_;
		rclcpp::TimerBase::SharedPtr timer_alg_;

		// Declare variables
		std::vector<std::vector<double>> p_;
		std::vector<std::vector<double>> q_;
		std::vector<rclcpp::Time> t_;

		std::vector<std::vector<double>> v_;
		std::vector<std::vector<double>> w_;

		std::vector<std::vector<double>> a_;
		std::vector<std::vector<double>> alpha_;
	};
}

#endif // rotors_mcs_MSC_NODE_H
