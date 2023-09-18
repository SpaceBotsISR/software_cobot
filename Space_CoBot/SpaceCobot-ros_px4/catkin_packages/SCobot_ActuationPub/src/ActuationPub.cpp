//
// Created by filiperosa on 2/7/18.
//
//ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <scobot_msgs/scobot_actuation.h>
//#include <ActuationVectorMsg.h>

//Other includes
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>

#include <sstream>

#define COPTER_LOOP_RATE 100

struct ActuationMsg{
    int id;
    float u1;
    float u2;
    float u3;
    float u4;
    float u5;
    float u6;
};

void consumeActuationMessage(ActuationMsg*);


int lastMsgId;
ros::Publisher actuation_pub;


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    //Pipe initialization
    lastMsgId = -1;
    int pipe;
    const char *myfifo="/home/erle/act_fifo";

    printf("Trying to connect to server.\n");
    if( (pipe = open(myfifo, O_RDONLY, 0 )) == -1 ) {
        perror("Error: open( ): ");
        return 1;
    }
    printf("Connection succeeded.\n");


    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "actuation_publisher");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    actuation_pub = n.advertise<scobot_msgs::scobot_actuation>("/scobot/motor_actuation",0);


    ros::Rate loop_rate(COPTER_LOOP_RATE*4);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok())
    {
        ActuationMsg msg;
        if( (read(pipe, &msg, sizeof(ActuationMsg))) < 0){
            ROS_INFO("Erro: read(actuation_pipe): ");
            //return 1;
        }
        else consumeActuationMessage(&msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}

void consumeActuationMessage(ActuationMsg* msg){
    if (msg->id != lastMsgId) {
        lastMsgId = msg->id;

        scobot_msgs::scobot_actuation ros_msg;
        ros_msg.header.frame_id = "fcu";
        ros_msg.header.seq = msg->id;

        ros_msg.u1 = msg->u1;
        ros_msg.u2 = msg->u2;
        ros_msg.u3 = msg->u3;
        ros_msg.u4 = msg->u4;
        ros_msg.u5 = msg->u5;
        ros_msg.u6 = msg->u6;

        actuation_pub.publish(ros_msg);
    }
}




