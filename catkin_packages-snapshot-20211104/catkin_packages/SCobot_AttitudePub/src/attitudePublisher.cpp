//ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

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

struct AttitudeMsg{
    int id;
    struct Quaternion{
        float w;
        float x;
        float y;
        float z;
    }quaternion;
};

void consumeAttMessage(AttitudeMsg*);

int lastMsgId;
ros::Publisher attitude_pub;


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    //Pipe initialization
    lastMsgId = -1;
    int pipe;
    const char *myfifo="/home/erle/imu_fifo";

    printf("Trying to connect to server.\n");
    if( (pipe = open(myfifo, O_RDONLY, 0 )) == -1 ) {
        perror("Error: open( ): ");
        return 1;
    }
    printf("Connection succeeded.\n");

    ros::init(argc, argv, "attitude_publisher");

    ros::NodeHandle n;

    attitude_pub = n.advertise<sensor_msgs::Imu>("/scobot/imu",0);

    ros::Rate loop_rate(COPTER_LOOP_RATE*4);


    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok())
    {
        AttitudeMsg msg;
        if( (read(pipe, &msg, sizeof(AttitudeMsg))) < 0){
            ROS_INFO("Erro: read(attitude_pipe): ");
            //return 1;
        }
        else consumeAttMessage(&msg);


        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}

void consumeAttMessage(AttitudeMsg* msg){
    if (msg->id != lastMsgId) {
        ROS_INFO("Q(%d) = [ %f, %f, %f, %f]\n",msg->id,msg->quaternion.w,msg->quaternion.x,msg->quaternion.y,msg->quaternion.z);
        lastMsgId = msg->id;


        sensor_msgs::Imu att;
        att.header.frame_id = "fcu";
        att.header.seq = msg->id;

        //Attitude quaternion
        att.orientation.w = msg->quaternion.w;
        att.orientation.x = msg->quaternion.x;
        att.orientation.y = msg->quaternion.y;
        att.orientation.z = msg->quaternion.z;

        attitude_pub.publish(att);
    }
}

