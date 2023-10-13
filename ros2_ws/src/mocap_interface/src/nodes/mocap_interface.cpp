//------------------------------------------------------
// MCS Node Main .cpp
// Author: Marina Moreira
//------------------------------------------------------

#include "mocap_interface.h"

namespace mocap_interface_ns
{
	using std::placeholders::_1;

	// constructor
	MocapInterface::MocapInterface()
		: Node("mocap_interface_node")
	{
		// Declare paramters
		this->declare_parameter("dt", kDefaultDt);
		this->declare_parameter("n_samples", kDefaultNSamples);
		this->declare_parameter("topic_listen", kDefaultTopicListen);
		this->declare_parameter("topic_publish", kDefaultTopicPublish);

		// Checks if in the laucnh file the parameters value are change
		// No need to care about to use a try and catch for rclcpp::exceptions::ParameterNotDeclaredException since parameters are declare right above
		dt_ = this->get_parameter("dt").as_double();
		n_samples_ = this->get_parameter("n_samples").as_int();
		topic_listen = this->get_parameter("topic_listen").as_string();
		topic_publish = this->get_parameter("topic_publish").as_string();

		std::cout << "INIT TOTAL\n"
				  << std::endl
				  << std::flush;

		pub_values_ = create_publisher<mocap_interface::msg::MocapMsg>(topic_publish.c_str(), 1);
		timer_mcs_ = create_subscription<geometry_msgs::msg::PoseStamped>(topic_listen.c_str(), 10, std::bind(&MocapInterface::PoseCallback, this, _1));

		timer_alg_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<unsigned int>(dt_ * 1000)),
											 std::bind(&MocapInterface::PoseTimeSequenceRegression, this));

		std::cout << "END INIT TOTAL\n"
				  << std::endl
				  << std::flush;
	}

	// destructor
	MocapInterface::~MocapInterface() {}

	// Algorithm 1
	// Input: A sequence of values
	// Output: The first and second derivative
	int MocapInterface::MotionRegression1D(std::vector<std::vector<double>> x, std::vector<rclcpp::Time> t,
										   std::vector<double> &v, std::vector<double> &a)
	{
		int n = t.size();
		int x_dim = x[0].size();
		double s_x, s_tx, s_t2x, s_t, s_t2, s_t3, s_t4;
		double t_i;
		double A;

		for (int j = 0; j < x_dim; j++)
		{

			s_x = 0;
			s_tx = 0;
			s_t2x = 0;
			s_t = 0;
			s_t2 = 0;
			s_t3 = 0;
			s_t4 = 0;
			t_i = 0;

			for (int i = 0; i < n; i++)
			{
				t_i = t[i].nanoseconds() - t.end()[-1].nanoseconds();
				t_i = t_i / 1000000000.0; //  Convert back to seconds
				// std::cout << "t_i " << t_i << std::endl;

				s_x = s_x + x[i][j];
				s_tx = s_tx + x[i][j] * t_i;
				s_t2x = s_t2x + x[i][j] * t_i * t_i;

				s_t = s_t + t_i;
				s_t2 = s_t2 + t_i * t_i;
				s_t3 = s_t3 + t_i * t_i * t_i;
				s_t4 = s_t4 + t_i * t_i * t_i * t_i;
			}

			A = n * (s_t3 * s_t3 - s_t2 * s_t4) + s_t * (s_t * s_t4 - s_t2 * s_t3) + s_t2 * (s_t2 * s_t2 - s_t * s_t3);

			v.push_back((1 / A) * (s_x * (s_t * s_t4 - s_t2 * s_t3) + s_tx * (s_t2 * s_t2 - n * s_t4) + s_t2x * (n * s_t3 - s_t * s_t2)));

			a.push_back(2 * (1 / A) * (s_x * (s_t2 * s_t2 - s_t * s_t3) + s_tx * (n * s_t3 - s_t * s_t2) + s_t2x * (s_t * s_t - n * s_t2)));
		}
	}

	// Algorithm 2
	// Input: A sequence of positions and orientations
	// Output: The translational and rotational velocities and accelerations
	int MocapInterface::MotionRegression6D(std::vector<std::vector<double>> p, std::vector<std::vector<double>> q,
										   std::vector<rclcpp::Time> t, std::vector<double> &v, std::vector<double> &w,
										   std::vector<double> &a, std::vector<double> &alpha)
	{ // std::cout << "MotionRegression6D1" << std::endl;

		MotionRegression1D(p, t, v, a);
		// std::cout << "MotionRegression6D2" << std::endl;
		std::vector<double> q_dot, q_dot_dot;
		MotionRegression1D(q, t, q_dot, q_dot_dot);
		// std::cout << "MotionRegression6D3" << std::endl;

		/////Make the tf::Quaternions for orientation, angular velocity and angular acceleration
		tf2::Quaternion quat = tf2::Quaternion(q.back()[0], q.back()[1], q.back()[2], q.back()[3]);
		tf2::Quaternion quat_dot = tf2::Quaternion(q_dot[0], q_dot[1], q_dot[2], q_dot[3]);
		tf2::Quaternion quat_dot_dot = tf2::Quaternion(q_dot_dot[0], q_dot_dot[1], q_dot_dot[2], q_dot_dot[3]);
		// std::cout << "MotionRegression6D4" << std::endl;

		/*//////////////////////////////////////////////////////////////////////////////////
		std::cout<<"Angular velocity Quaternion: world? "<<quat_dot.x()<<" "<<quat_dot.y()<<" "<<quat_dot.z()<<" "<<quat_dot.w()<<"\n";
		tf::Quaternion temp_q_w = quat*quat_dot;
		std::cout<<"Angular velocity in radians: "<<2*temp_q_w.getAxis()[0]<<" "<<2*temp_q_w.getAxis()[1]<<" "<<2*temp_q_w.getAxis()[2]<<"\n";*/

		////////////////////////////////////////////////////////////////////////////////////////////////

		////Transforming to body fixed frame?-- There was a transformation to the body frame using adjoint of the quaternion but we need in the inertial frame.
		// ALG Edit: changing quat to adj_quat according to the formula and THEN CHANGED BACK TO NORMAL
		tf2::Quaternion adj_quat = tf2::Quaternion(-1 * quat.x(), -1 * quat.y(), -1 * quat.z(), quat.w());
		tf2::Quaternion q_w = adj_quat * quat_dot;
		tf2::Quaternion q_alpha = quat * quat_dot_dot;

		std::cout << "Angular Velocity: Body?: " << q_w.x() << " " << q_w.y() << " " << q_w.z() << " " << q_w.w() << "\n";

		// TODO(ALG): Didn't account for the ficititious forces

		// std::cout << "MotionRegression6D5" << std::endl;
		w.push_back(2 * q_w.getAxis()[0]);
		w.push_back(2 * q_w.getAxis()[1]);
		w.push_back(2 * q_w.getAxis()[2]);
		alpha.push_back(2 * q_alpha.getAxis()[0]);
		alpha.push_back(2 * q_alpha.getAxis()[1]);
		alpha.push_back(2 * q_alpha.getAxis()[2]);
	}

	// Algorithm 3
	// Input: The pose time sequence
	// Output: The sequence of translational and rotational velocities and accelerations
	void MocapInterface::PoseTimeSequenceRegression()
	{
		std::vector<double> v;
		std::vector<double> w;
		std::vector<double> a;
		std::vector<double> alpha;
		// std::cout << "PoseTimeSequenceRegression1" << std::endl;

		if (p_.size() < n_samples_)
			return;
		// Delete old values
		for (int i = n_samples_; i < p_.size(); i++)
		{
			p_.erase(p_.begin());
			q_.erase(q_.begin());
			t_.erase(t_.begin());
		}
		// std::cout << "PoseTimeSequenceRegression2" << std::endl;

		MotionRegression6D(p_, q_, t_, v, w, a, alpha);
		// std::cout << "PoseTimeSequenceRegression3" << std::endl;

		mocap_interface::msg::MocapMsg msg;
		// std::cout << "PoseTimeSequenceRegression4" << std::endl;

		// Position
		msg.pose.pose.position.x = p_.back()[0];
		msg.pose.pose.position.y = p_.back()[1];
		msg.pose.pose.position.z = p_.back()[2];

		// Linear Velocity
		if (!std::isnan(v[0]))
			msg.velocity.twist.linear.x = v[0];
		else
			msg.velocity.twist.linear.x = 0;
		if (!std::isnan(v[1]))
			msg.velocity.twist.linear.y = v[1];
		else
			msg.velocity.twist.linear.y = 0;
		if (!std::isnan(v[2]))
			msg.velocity.twist.linear.z = v[2];
		else
			msg.velocity.twist.linear.z = 0;

		// Linear Acceleration
		if (!std::isnan(a[0]))
			msg.acceleration.twist.linear.x = a[0];
		else
			msg.acceleration.twist.linear.x = 0;
		if (!std::isnan(a[1]))
			msg.acceleration.twist.linear.y = a[1];
		else
			msg.acceleration.twist.linear.y = 0;
		if (!std::isnan(a[2]))
			msg.acceleration.twist.linear.z = a[2];
		else
			msg.acceleration.twist.linear.z = 0;

		// Quaternions
		msg.pose.pose.orientation.x = q_.back()[0];
		msg.pose.pose.orientation.y = q_.back()[1];
		msg.pose.pose.orientation.z = q_.back()[2];
		msg.pose.pose.orientation.w = q_.back()[3];

		// Angular Velocity
		if (!std::isnan(w[0]))
			msg.velocity.twist.angular.x = w[0];
		else
			msg.velocity.twist.angular.x = 0;
		if (!std::isnan(w[1]))
			msg.velocity.twist.angular.y = w[1];
		else
			msg.velocity.twist.angular.y = 0;
		if (!std::isnan(w[2]))
			msg.velocity.twist.angular.z = w[2];
		else
			msg.velocity.twist.angular.z = 0;

		// Angular Acceleration
		if (!std::isnan(alpha[0]))
			msg.acceleration.twist.angular.x = alpha[0];
		else
			msg.acceleration.twist.angular.x = 0;
		if (!std::isnan(alpha[1]))
			msg.acceleration.twist.angular.y = alpha[1];
		else
			msg.acceleration.twist.angular.x = 0;
		if (!std::isnan(alpha[2]))
			msg.acceleration.twist.angular.z = alpha[2];
		else
			msg.acceleration.twist.angular.z = 0;

		msg.header.stamp = get_clock()->now();

		pub_values_->publish(msg);
	}

	void MocapInterface::PoseCallback(const geometry_msgs::msg::PoseStamped &msg)
	{

		// Store the data
		p_.push_back({msg.pose.position.x, msg.pose.position.y, msg.pose.position.z});

		q_.push_back({msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w});

		t_.push_back(msg.header.stamp);
	}

}

int main(int argc, char **argv)
{

	rclcpp::init(argc, argv);
	std::cout << "\nNode Started" << std::endl;

	rclcpp::spin(std::make_shared<mocap_interface_ns::MocapInterface>());

	return 0;
}
