#include "client.hpp"

Client::Client(const char *serverIp, int serverPort)
    : serverIp_(serverIp), serverPort_(serverPort) {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        throw std::runtime_error("Error creating socket");
    }

    memset(&serverAddr_, 0, sizeof(serverAddr_));
    serverAddr_.sin_family = AF_INET;
    serverAddr_.sin_port = htons(serverPort_);
    serverAddr_.sin_addr.s_addr = inet_addr(serverIp_);

    if (bind(sockfd_, (struct sockaddr *)&serverAddr_, sizeof(serverAddr_)) < 0) {
        perror("Error binding socket");
        return;
    }
}

std::vector<float> Client::receive_msg() {
    std::vector<float> msg;
    char buffer[1024];
    memset(buffer, 0, sizeof(buffer));
    socklen_t len = sizeof(serverAddr_);

    int n = recvfrom(sockfd_, buffer, sizeof(buffer), 0, (struct sockaddr *)&serverAddr_, &len);
    if (n < 0) {
        perror("Error receiving message");
        return msg;
    }

    std::string str(buffer);
    std::cout << "Received message: " << str << std::endl;
    std::string delimiter = " ";
    size_t pos = 0;
    std::string token;

    while ((pos = str.find(delimiter)) != std::string::npos) {
        token = str.substr(0, pos);
        msg.push_back(std::stof(token));
        str.erase(0, pos + delimiter.length());
    }
    msg.push_back(std::stof(str));

    return msg;
}

Client::~Client() {
    if (sockfd_ >= 0) {
        close(sockfd_);
    }
}