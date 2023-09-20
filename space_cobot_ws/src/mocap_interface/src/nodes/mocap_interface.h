#ifndef rotors_mcs_MSC_NODE_H
#define rotors_mcs_MSC_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <stdio.h>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <mav_msgs/common.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include "mocap_interface/mocap_msg.h"


#include <tf/transform_datatypes.h>

namespace mocap_interface_ns {

static const float kDefaultDt				= 0.02;
static const float kDefaultNSamples			= 30;
static const float kDefaultRateOptitrack 	= 200;

static const std::string 	kDefaultTopicListen   = "/vrpn_client_node/space_cobot/pose";
//Since a message file with all the necessary values (pose, velocity and acceleration are used hence only one topic is sufficient)
static const std::string 	kDefaultTopicPublish  = "/space_cobot/mocap_interface/data";



class MocapInterface
{
	public:
		MocapInterface();
		~MocapInterface();

	private:
	float dt_;
	int n_samples_;
	
	int MotionRegression1D(std::vector<std::vector<double>> x, std::vector<ros::Time> t, 
								std::vector<double> &v, std::vector<double> &a);
	int MotionRegression6D(std::vector<std::vector<double>> p, std::vector<std::vector<double>> q,
							std::vector<ros::Time> t, std::vector<double> &v, std::vector<double> &w,
							std::vector<double> &a,  std::vector<double> &alpha);
	void PoseTimeSequenceRegression(const ros::TimerEvent& event);
	void PoseCallback(const geometry_msgs::PoseStamped& msg);


	ros::Publisher  pub_values_;

	ros::Subscriber timer_mcs_;
	ros::Timer timer_alg_;


	// Declare variables
	std::vector<std::vector<double>> p_;
	std::vector<std::vector<double>> q_;
	std::vector<ros::Time> t_;

	std::vector<std::vector<double>> v_;
	std::vector<std::vector<double>> w_;

	std::vector<std::vector<double>> a_;
	std::vector<std::vector<double>> alpha_;

};
}
#endif // rotors_mcs_MSC_NODE_H
