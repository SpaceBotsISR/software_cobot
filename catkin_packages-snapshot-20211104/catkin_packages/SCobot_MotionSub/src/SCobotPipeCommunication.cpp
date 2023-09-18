//
// Created by filiperosa on 2/25/18.
//

#include "SCobotPipeCommunication.h"

int setupPipeProducer(const char *pipe_path, int * pipe_fd, bool waitForListener){
    struct stat status;

    int readfd = open(pipe_path, O_RDONLY | O_NONBLOCK);
    if(readfd==-1)
    {
        printf("readfd: open(%s)",pipe_path);
        exit(EXIT_FAILURE);
    }

    if(fstat(readfd, &status)==-1)
    {
        printf("fstat");
        close(readfd);
        exit(EXIT_FAILURE);
    }

    if(waitForListener){ //Wait for ROS node to open pipe. Mind that if it closes we terminate execution.
        close(readfd);
        printf("Trying to connect to autopilot\n");
        *pipe_fd = open(pipe_path, O_WRONLY);
    }
    else *pipe_fd = open(pipe_path, O_WRONLY | O_NONBLOCK); //Here we open without blocking


    if(*pipe_fd==-1)
    {
        printf("writefd: open(%s)",pipe_path);
        close(readfd);
        exit(EXIT_FAILURE);
    }
    printf("Setup %s pipe.\n",pipe_path);
    return 1;
}

int setupPipeConsumer(const char* pipe_path, int *pipe_fd, bool waitForProducer){
    struct stat status;

    if(waitForProducer){
        printf("Trying to connect to autopilot.\n");
        *pipe_fd = open(pipe_path, O_RDONLY);
    }
    else *pipe_fd = open(pipe_path, O_RDONLY | O_NONBLOCK);

    if(*pipe_fd==-1)
    {
        perror("readfd: open()");
        exit(EXIT_FAILURE);
    }

    if(fstat(*pipe_fd, &status)==-1)
    {
        perror("fstat");
        close(*pipe_fd);
        exit(EXIT_FAILURE);
    }

    printf("Setup %s pipe.\n",pipe_path);
    return 1;
}
