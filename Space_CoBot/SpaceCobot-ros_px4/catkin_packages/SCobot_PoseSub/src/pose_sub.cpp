//
// Created by filiperosa on 2/25/18.
//
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "SCobotPipeCommunication.h"

int posePipe_fd;
const char *posePipe_path="/home/erle/pose_fifo"; //carries current position and velocity from simulation

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& ros_msg)
{
    PoseMsg pipe_msg;
    pipe_msg.id = ros_msg->header.seq;

    pipe_msg.position.x = ros_msg->pose.position.x;
    pipe_msg.position.y = ros_msg->pose.position.y;
    pipe_msg.position.z = ros_msg->pose.position.z;

    pipe_msg.att_quaternion.w = ros_msg->pose.orientation.w ;
    pipe_msg.att_quaternion.x = ros_msg->pose.orientation.x ;
    pipe_msg.att_quaternion.y = ros_msg->pose.orientation.y ;
    pipe_msg.att_quaternion.z = ros_msg->pose.orientation.z ;

    if( write( posePipe_fd, &pipe_msg , sizeof(PoseMsg) ) < 0 ) {
        ROS_INFO("FIFO ERROR sending target pose");
        return;
    }
    ROS_INFO("p_ref=[%f,%f,%f] q_ref=[%f,%f,%f,%f]",
             ros_msg->pose.position.x,ros_msg->pose.position.y,ros_msg->pose.position.z,
             ros_msg->pose.orientation.w,ros_msg->pose.orientation.x,ros_msg->pose.orientation.y,ros_msg->pose.orientation.z );

}


int main(int argc, char **argv)
{
    posePipe_fd = 0;
    setupPipeProducer(posePipe_path,&posePipe_fd,false);

    ros::init(argc, argv, "pose_subscriber");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/scobot/target_pose", 0, poseCallback);

    ros::spin();

    return 0;
}
