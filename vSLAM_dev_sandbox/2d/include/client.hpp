#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

class Client {
   public:
    Client(const char* serverIp, int serverPort);
    std::vector<float> receive_msg();
    ~Client();

   private:
    int sockfd_;
    const char* serverIp_;
    int serverPort_;
    struct sockaddr_in serverAddr_;
};

#endif