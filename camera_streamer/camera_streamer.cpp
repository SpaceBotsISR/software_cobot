#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#define CAPTURE_WIDTH 1920
#define CAPTURE_HEIGHT 1080
#define DISPLAY_WIDTH 960
#define DISPLAY_HEIGHT 540
#define FRAMERATE 30
#define FLIP_METHOD 2

std::string gstreamer_pipeline(int sensor_id) {
    return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) +
           " ! "
           "video/x-raw(memory:NVMM), width=(int)" +
           std::to_string(CAPTURE_WIDTH) + ", height=(int)" + std::to_string(CAPTURE_HEIGHT) +
           ", FRAMERATE=(fraction)" + std::to_string(FRAMERATE) +
           "/1 ! "
           "nvvidconv flip-method=" +
           std::to_string(FLIP_METHOD) +
           " ! "
           "video/x-raw, width=(int)" +
           std::to_string(DISPLAY_WIDTH) + ", height=(int)" + std::to_string(DISPLAY_HEIGHT) +
           ", format=(string)BGRx ! "
           "videoconvert ! "
           "video/x-raw, format=(string)BGR ! appsink";
}

// ... [Your VideoServer class remains unchanged]

int main(int argc, char* argv[]) {
    signal(SIGPIPE, SIG_IGN);  // Ignore SIGPIPE signals

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " [-single | -all | -list <sensor_id_1> <sensor_id_2> ...]" << std::endl;
        return 1;
    }

    std::vector<int> sensor_ids;
    std::string mode = argv[1];

    if (mode == "-single") {
        sensor_ids.push_back(0);
    } else if (mode == "-all") {
        for (int i = 0; i < 4; ++i) {
            sensor_ids.push_back(i);
        }
    } else if (mode == "-list") {
        for (int i = 2; i < argc; ++i) {
            sensor_ids.push_back(std::stoi(argv[i]));
        }
    } else {
        std::cerr << "Invalid option: " << mode << std::endl;
        return 1;
    }

    int base_port = 8080;

    // Fork a child process for each camera sensor
    for (int sensor_id : sensor_ids) {
        pid_t pid = fork();

        if (pid == 0) {  // Child process
            std::cout << gstreamer_pipeline(sensor_id) << std::endl;
            VideoServer video_server(base_port + sensor_id);
            video_server.stream();
            exit(0);  // End child process after streaming
        } else if (pid < 0) {
            std::cerr << "Failed to fork." << std::endl;
            exit(1);
        }
    }

    // Parent process continues to run, waiting for children to finish
    for (int i = 0; i < sensor_ids.size(); i++) {
        wait(NULL);
    }

    return 0;
}
