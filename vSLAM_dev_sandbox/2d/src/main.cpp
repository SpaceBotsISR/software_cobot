#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "client.hpp"

void test_client() {
    Client client("127.0.0.1", 8009);
    std::vector<double> pose = {0.0, 0.0, 0.0};

    while (true) {
        std::vector<double> msg = client.receive_msg();
        double vx = msg[0];
        double vy = msg[1];
        double v_theta = msg[2];

        pose[2] += v_theta;
        pose[0] += vx * cos(pose[2]) + vy * sin(pose[2]);
        pose[1] += vx * sin(pose[2]) + vy * cos(pose[2]);

        client.send_msg(pose);
    }
}

int main() {
    test_client();
    return 0;
}
