//
// Created by filiperosa on 2/25/18.
//
#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include "SCobotPipeCommunication.h"

int velPipe_fd;
const char *velPipe_path="/home/erle/vel_fifo";

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& ros_msg)
{
    VelocityMsg pipe_msg;
    pipe_msg.id = ros_msg->header.seq;


    pipe_msg.v.x = ros_msg->twist.linear.x;
    pipe_msg.v.y = ros_msg->twist.linear.y;
    pipe_msg.v.z = ros_msg->twist.linear.z;

    pipe_msg.w.x = ros_msg->twist.angular.x;
    pipe_msg.w.y = ros_msg->twist.angular.y;
    pipe_msg.w.z = ros_msg->twist.angular.z;

    if( write( velPipe_fd, &pipe_msg , sizeof(PoseMsg) ) < 0 ) {
        ROS_INFO("FIFO ERROR sending target velocities");
        return;
    }
    ROS_INFO("v_ref=[%f,%f,%f] w_ref=[%f,%f,%f]",
             ros_msg->twist.linear.x,
             ros_msg->twist.linear.y,
             ros_msg->twist.linear.z,
             ros_msg->twist.angular.x,
             ros_msg->twist.angular.y,
             ros_msg->twist.angular.z
    );
}


int main(int argc, char **argv)
{
    velPipe_fd = 0;
    setupPipeProducer(velPipe_path,&velPipe_fd,false);

    ros::init(argc, argv, "velocity_subscriber");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/scobot/target_vel", 0, velocityCallback);

    ros::spin();

    return 0;
}
