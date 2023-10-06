#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

const int sensor_id = 0;
const int capture_width = 1920;
const int capture_height = 1080;
const int display_width = 960;
const int display_height = 540;
const int framerate = 30;
const int flip_method = 0;

std::string gstreamer_pipeline() {
    return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) +
           " ! "
           "video/x-raw(memory:NVMM), width=(int)" +
           std::to_string(capture_width) + ", height=(int)" + std::to_string(capture_height) +
           ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! "
           "nvvidconv flip-method=" +
           std::to_string(flip_method) +
           " ! "
           "video/x-raw, width=(int)" +
           std::to_string(display_width) + ", height=(int)" + std::to_string(display_height) +
           ", format=(string)BGRx ! "
           "videoconvert ! "
           "video/x-raw, format=(string)BGR ! appsink";
}

class VideoServer {
   public:
    VideoServer(int port) {
        // Create socket
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket < 0) {
            std::cerr << "Error creating socket!" << std::endl;
            exit(1);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = INADDR_ANY;

        bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
        listen(server_socket, 1);
        std::cout << "Waiting for a connection..." << std::endl;

        server_socket = accept(server_socket, NULL, NULL);

        video_capture = cv::VideoCapture(gstreamer_pipeline(), cv::CAP_GSTREAMER);
        if (!video_capture.isOpened()) {
            std::cerr << "Error: Unable to open camera" << std::endl;
            exit(1);
        }
    }

    void stream() {
        cv::Mat frame;
        while (video_capture.read(frame)) {
            send_frame(frame);
        }
        video_capture.release();
        close(server_socket);
    }

   private:
    int server_socket;
    struct sockaddr_in server_addr;
    cv::VideoCapture video_capture;
    bool isConnected;

    void send_frame(cv::Mat& frame) {
        ssize_t msg_size = frame.total() * frame.elemSize();
        unsigned char* frame_data_ptr = frame.data;
        std::string buff_size_msg = std::to_string(msg_size);

        while (msg_size > 0) {
            ssize_t bytes_sent = send(server_socket, frame_data_ptr, msg_size, 0);

            if (bytes_sent <= 0) {
                std::cerr << "Error sending data: " << strerror(errno) << std::endl;
                break;
            }

            msg_size -= bytes_sent;
            frame_data_ptr += bytes_sent;
        }
    }
};

int main() {
    std::cout << gstreamer_pipeline() << std::endl;
    VideoServer video_server(8080);
    video_server.stream();

    return 0;
}