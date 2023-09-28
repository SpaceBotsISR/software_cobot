#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

const int sensor_id = 0;
const int capture_width = 1920;
const int capture_height = 1080;
const int display_width = 960;
const int display_height = 540;
const int framerate = 30;
const int flip_method = 0;

std::string gstreamer_pipeline() {
    return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) + " ! "
           "video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" + std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) + "/1 ! "
           "nvvidconv flip-method=" + std::to_string(flip_method) + " ! "
           "video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" + std::to_string(display_height) + ", format=(string)BGRx ! "
           "videoconvert ! "
           "video/x-raw, format=(string)BGR ! appsink";
}

int main() {
    int client_socket;
    struct sockaddr_in server_addr;

    client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket < 0) {
        std::cerr << "Error creating socket!" << std::endl;
        return -1;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8080);
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    // Connect to server
    if (connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Error connecting to server!" << std::endl;
        return -1;
    }

    cv::VideoCapture video_capture(gstreamer_pipeline(), cv::CAP_GSTREAMER);

    if (video_capture.isOpened()) {
        try {
            while (true) {
                cv::Mat frame;
                if (!video_capture.read(frame)) {
                    break;
                }

                // Send frame data to the server
                send(client_socket, frame.data, frame.total() * frame.elemSize(), 0);

                int keyCode = cv::waitKey(10) & 0xFF;
                if (keyCode == 27 || keyCode == 'q') {
                    break;
                }
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
        video_capture.release();
    } else {
        std::cerr << "Error: Unable to open camera" << std::endl;
    }

    close(client_socket);
    return 0;
}
