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

class VideoServer {
   public:
    VideoServer(int port) {
        init_socket(port);
        video_capture = cv::VideoCapture(gstreamer_pipeline(), cv::CAP_GSTREAMER);
        if (!video_capture.isOpened()) {
            std::cerr << "Error: Unable to open camera" << std::endl;
            exit(1);
        }
    }

    void init_socket(int port) {
        listening_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (listening_socket < 0) {
            std::cerr << "Error creating socket!" << std::endl;
            exit(1);
        }

        int opt = 1;
        if (setsockopt(listening_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            std::cerr << "Error setting SO_REUSEADDR socket option!" << std::endl;
            exit(1);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = INADDR_ANY;

        bind(listening_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
        listen(listening_socket, 1);
    }

    void wait_for_client() {
        std::cout << "Waiting for a connection..." << std::endl;
        client_socket = accept(listening_socket, NULL, NULL);
        if (client_socket < 0) {
            std::cerr << "Error accepting connection!" << std::endl;
        }
    }

    void stream() {
        cv::Mat frame;
        while (true) {
            wait_for_client();
            std::cout << "Client connected!" << std::endl;
            while (video_capture.read(frame)) {
                if (!send_frame(frame)) {
                    std::cerr << "Client disconnected. Waiting for a new client..." << std::endl;
                    close(client_socket);  // Close the client socket when they disconnect
                    break;
                }
            }
        }
        video_capture.release();
        close(listening_socket);  // Close the listening socket only when we're completely done
    }

   private:
    int listening_socket;
    int client_socket;
    struct sockaddr_in server_addr;
    cv::VideoCapture video_capture;

    bool send_frame(cv::Mat& frame) {
        ssize_t msg_size = frame.total() * frame.elemSize();
        unsigned char* frame_data_ptr = frame.data;

        while (msg_size > 0) {
            ssize_t bytes_sent = send(client_socket, frame_data_ptr, msg_size, 0);

            if (bytes_sent <= 0) {
                std::cerr << "Error sending data: " << strerror(errno) << std::endl;
                return false;
            }

            msg_size -= bytes_sent;
            frame_data_ptr += bytes_sent;
        }
        return true;
    }
};

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
