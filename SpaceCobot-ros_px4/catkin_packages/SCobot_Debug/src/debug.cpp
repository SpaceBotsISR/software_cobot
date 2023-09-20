//
// Created by filiperosa on 2/20/18.
//

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

//Other includes
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>

#define MAX_PUBLISHERS 10
#define COPTER_LOOP_RATE 100

struct StringMsg{
    int id;
    char topic[40];
    char message[512];
};

void consumeMessage(StringMsg*,ros::NodeHandle*);

int lastMsgId, pubCounter;
ros::Publisher pub[MAX_PUBLISHERS];

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

    const char *myfifo="/home/erle/debug_fifo";
    int readfd;
    struct stat status;

    printf("Trying to connect to server.\n");

    readfd = open(myfifo, O_RDONLY | O_NONBLOCK);
    if(readfd==-1)
    {
        perror("readfd: open()");
        exit(EXIT_FAILURE);
    }
    printf("OK 1\n");

    if(fstat(readfd, &status)==-1)
    {
        perror("fstat");
        close(readfd);
        exit(EXIT_FAILURE);
    }
    printf("OK 2\n");

    if(!S_ISFIFO(status.st_mode))
    {
        printf("%s in not a fifo!\n", myfifo);
        close(readfd);
        exit(EXIT_FAILURE);
    }
    printf("OK 3\n");
    printf("Connection succeeded.\n");

    pubCounter = 0;

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
    ros::init(argc, argv, "scobot_debugger");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle node;

    ros::Rate loop_rate(COPTER_LOOP_RATE*10);

    int count = 0;
    while (ros::ok())
    {
        StringMsg msg;
        if( (read(readfd, &msg, sizeof(StringMsg))) < 0){
            //perror("Erro: read( ): ");
            //return 1;
        }
        else consumeMessage(&msg,&node);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}

void consumeMessage(StringMsg* msg, ros::NodeHandle* node){
    if (msg->id != lastMsgId) {
        lastMsgId = msg->id;

        //Return if message does not specify rostopic
        if(msg->topic=="") return;

        //Declare ros message and fill it
        std_msgs::String ros_msg;
        ros_msg.data = msg->message;

        //Search for topic in the array of publishers
        bool found = false;
        for (int i = 0; i < pubCounter; i++) {
           if(pub[i].getTopic().compare(msg->topic)==0){
               pub[i].publish(ros_msg);
               ROS_INFO("Published to '%s' topic",msg->topic);
               found = true;
               break;
           }
        }
        //If not found create a new publisher for the new topic
        if(!found && pubCounter<MAX_PUBLISHERS) {
            pub[pubCounter] = node->advertise<std_msgs::String>(msg->topic, 0);
            pub[pubCounter].publish(ros_msg);
            ROS_INFO("Registered new topic: %s",msg->topic);
            pubCounter++;
        }
        else if(!found) ROS_INFO("MAX_PUBLISHERS reached");
    }
}
