//
// Created by filiperosa on 2/25/18.
//

#ifndef SCOBOT_MOTIONSUB_SCOBOTPIPECOMMUNICATION_H
#define SCOBOT_MOTIONSUB_SCOBOTPIPECOMMUNICATION_H


#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>

/**
 * Structures below define the objects to be sent through the pipes
 */

struct AttitudeMsg{
    int id;
    //int time;
    struct Quaternion{
        float w;
        float x;
        float y;
        float z;
    }quaternion;
};

struct ActuationMsg{
    int id;
    struct ActuationVector{
        float u1;
        float u2;
        float u3;
        float u4;
        float u5;
        float u6;
    }U;
};

struct StringMsg{
    int id;
    char topic[40];
    char message[512];
};

struct MotionMsg{
    int id;
    struct Position{
        float x;
        float y;
        float z;
    }pos;
    struct Velocity{
        float x;
        float y;
        float z;
    }vel;
};

struct PoseMsg{
    int id;
    struct Point{
        float x;
        float y;
        float z;
    }position;
    struct Quaternion{
        float w;
        float x;
        float y;
        float z;
    }att_quaternion;
};

struct VelocityMsg{
    int id;
    struct linear{
        float x;
        float y;
        float z;
    }v;
    struct angular{
        float x;
        float y;
        float z;
    }w;
};

/**
 * Setup a linux fifo producer
 * This ROS version assumes the fifo is already created.
 *
 * @param pipe_path - path to the fifo
 * @param pipe_fd - pointer to variable where the fifo file descriptor will be stored
 * @param waitForListener - true if open is a a blocking operation (waits for other process open())
 *                        - false if non-blocking -> we'll write to the pipe regardless of anyone being reading
 * @return
 */
int setupPipeProducer(const char *pipe_path, int * pipe_fd, bool waitForListener);

/**
 * Setup a linux fifo listener
 * This ROS version assumes the fifo is already created.
 *
 * @param pipe_path - path to the fifo
 * @param pipe_fd - pointer to variable where the fifo file descriptor will be stored
 * @param waitForProducer - true if open is a a blocking operation (waits for other process open())
 *                        - false if non-blocking -> reads will fail until someone writes to the pipe
 * @return
 */
int setupPipeConsumer(const char* pipe_path, int *pipe_fd, bool waitForProducer);


#endif //SCOBOT_MOTIONSUB_SCOBOTPIPECOMMUNICATION_H
