//
// Created by filiperosa on 2/25/18.
//
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <scobot_msgs/scobot_motion.h>
#include "SCobotPipeCommunication.h"

int motionPipe_fd;
const char *motionPipe_path="/home/erle/motion_fifo"; //carries current position and velocity from simulation

void motionCallback(const scobot_msgs::scobot_motion::ConstPtr& ros_msg)
{
    MotionMsg pipe_msg;
    pipe_msg.id = ros_msg->header.seq;

    pipe_msg.pos.x = ros_msg->position.x;
    pipe_msg.pos.y = ros_msg->position.y;
    pipe_msg.pos.z = ros_msg->position.z;

    pipe_msg.vel.x = ros_msg->velocity.x;
    pipe_msg.vel.y = ros_msg->velocity.y;
    pipe_msg.vel.z = ros_msg->velocity.z;

    if( write( motionPipe_fd, &pipe_msg , sizeof(MotionMsg) ) < 0 ) {
        ROS_INFO("FIFO ERROR sending position");
        return;
    }
    ROS_INFO("p=[%f,%f,%f] v=[%f,%f,%f]",
             ros_msg->position.x,ros_msg->position.y,ros_msg->position.z,
             ros_msg->velocity.x,ros_msg->velocity.y,ros_msg->velocity.z );
}


int main(int argc, char **argv)
{
    motionPipe_fd = 0;
    setupPipeProducer(motionPipe_path,&motionPipe_fd,false);

    ros::init(argc, argv, "motion_subscriber");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/scobot/motion", 0, motionCallback);

    ros::spin();

    return 0;
}
