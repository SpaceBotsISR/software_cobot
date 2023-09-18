//
//  main.cpp
//  FIFO_NONBLOCK_Producer
//
//  Created by Filipe Rosa on 14/02/2018.
//  Copyright © 2018 FRosa. All rights reserved.
//

//Instructions:
//
//This is a fifo writer to test if the scobot_debug node is correctly publishing to ROS
//Compile with: g++ -std=c++11 debug_fifo_writer.cpp -o debug_fifo_writer
//
//
//Based on: https://stackoverflow.com/questions/7360473/linux-non-blocking-fifo-on-demand-logging
//This code assumes a FIFO was created before running "mkfifo /tmp/arqfifo" on a terminal

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string>
#include <cstring>
#include <algorithm>

#define MAX_SIZE 32

using namespace std;

struct StringMsg{
    int id;
    char topic[20];
    char message[512];
};

struct stat status;
int readfd, writefd, counter;


string random_string( size_t );

int main( void ) {
    
    counter=0;
    const char *myfifo="/home/erle/debug_fifo";
    
     //Doing mkfifo in terminal before running
     remove(myfifo);
     
     if( mkfifo(myfifo, 0666 ) < 0 ) {
     perror("Error: mkfifo( ): ");
     return 1;
     }
    
    
    //signal(SIGPIPE, SIG_IGN);
    
    readfd = open(myfifo, O_RDONLY | O_NONBLOCK);
    if(readfd==-1)
    {
        perror("readfd: open()");
        exit(EXIT_FAILURE);
    }
    
    if(fstat(readfd, &status)==-1)
    {
        perror("fstat");
        close(readfd);
        exit(EXIT_FAILURE);
    }
    
    if(!S_ISFIFO(status.st_mode))
    {
        printf("%s in not a fifo!\n", myfifo);
        close(readfd);
        exit(EXIT_FAILURE);
    }
    
    writefd = open(myfifo, O_WRONLY | O_NONBLOCK);
    if(-1==writefd)
    {
        perror("writefd: open()");
        close(readfd);
        exit(EXIT_FAILURE);
    }
    printf("Connection succeeded.\n");
    //close(readfd);
    
    /*
     printf("Trying to connect to client.\n");
     if( (pipe = open(myfifo, O_WRONLY)) < 1 ) {
     perror("Error: open( ): ");
     return 1;
     }
     printf("Connection succeeded.\n");
     */
    
    
    
    while (true){

        StringMsg msg;
        msg.id = counter++;
        strcpy(msg.topic,"/scobot/debug");
        strcpy(msg.message, random_string(rand()%10).c_str());
        printf("%s\n",msg.message );   

        if(write( writefd, &msg , sizeof(StringMsg)) < 0 ){
            //write failed
        }

        usleep(100000);
    }
    
    
    close( writefd );
    return 0;
}

string random_string( size_t length )
{
    auto randchar = []() -> char
    {
        const char charset[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[ rand() % max_index ];
    };
    string str(length,0);
    generate_n( str.begin(), length, randchar );
    return str;
}


