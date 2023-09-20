//
// Created by filiperosa on 2/7/18.
//
//ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <scobot_msgs/scobot_position.h>

///* The publisher to this topic is the V-REP Simulator
/// The aim of this node is to forward the position
/// to the vehicle through a linux FIFO*/

//Other includes
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>

struct PositionMsg{
    int id;
    int x;
    int y;
    int z;
};

int position_pipe;
const char *pos_fifo="/home/erle/pos_fifo";

void positionCallback(const scobot_msgs::scobot_position& ros_msg)
{
    PositionMsg pipe_msg;
    pipe_msg.id = ros_msg.header.seq;
    pipe_msg.x = ros_msg.position.x;
    pipe_msg.y = ros_msg.position.y;
    pipe_msg.z = ros_msg.position.z;

    if( write( position_pipe, &pipe_msg , sizeof(PositionMsg) ) < 0 ) {
        ROS_INFO("FIFO ERROR sending position");
    }
    else ROS_INFO("x=%f y=%f z=%f",ros_msg.position.x, ros_msg.position.y, ros_msg.position.z);
}

int main(int argc, char **argv)
{
    //Setup FIFO communication
    remove(pos_fifo); //remove previous instance to avoid mkfifo error

    if( mkfifo(pos_fifo, 0666 ) < 0 ) {
        perror("Error: mkfifo( ): ");
        return 0;
    }
    printf("Trying to connect to autopilot.\n");
    if( (position_pipe = open(pos_fifo, O_WRONLY)) < 1 ) {
        perror("Error: open( ): ");
        return 0;
    }

    ros::init(argc, argv, "pos_listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/scobot/position", 0, positionCallback);

    ros::spin();

    return 0;
}