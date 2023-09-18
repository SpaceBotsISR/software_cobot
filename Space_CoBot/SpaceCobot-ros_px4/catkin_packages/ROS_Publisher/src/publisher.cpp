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

struct AttitudeMsg{
    int id;
    struct Quaternion{
        float x;
        float y;
        float z;
        float w;
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
    ros::init(argc, argv, "talker");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    attitude_pub = n.advertise<sensor_msgs::Imu>("/scobot/imu",50);

    ros::Rate loop_rate(50);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        /*
        std_msgs::String ros_msg;

        std::stringstream ss;
        ss << "hello world " << count;
        ros_msg.data = ss.str();

        ROS_INFO("%s", ros_msg.data.c_str());
        */
        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        //chatter_pub.publish(ros_msg);


        AttitudeMsg msg;
        if( (read(pipe, &msg, sizeof(AttitudeMsg))) < 0){
            perror("Erro: read( ): ");
            return 1;
        }
        consumeAttMessage(&msg);


        ros::spinOnce();

        //loop_rate.sleep();
        ++count;
    }


    return 0;
}

void consumeAttMessage(AttitudeMsg* msg){
    if (msg->id > lastMsgId) {
        ROS_INFO("Q(%d) = [ %f, %f, %f, %f]\n",msg->id,msg->quaternion.x,msg->quaternion.y,msg->quaternion.z,msg->quaternion.w);
        lastMsgId = msg->id;


        sensor_msgs::Imu att;
        att.header.frame_id = "fcu";

        //Attitude quaternion
        att.orientation.x = msg->quaternion.x;
        att.orientation.y = msg->quaternion.y;
        att.orientation.z = msg->quaternion.z;
        att.orientation.w = msg->quaternion.w;

        attitude_pub.publish(att);
    }
}

