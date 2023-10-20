#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#define SENSOR_ID 0
#define CAPTURE_WIDTH 1920
#define CAPTURE_HEIGHT 1080
#define DISPLAY_WIDTH 960
#define DISPLAY_HEIGHT 540
#define FRAMERATE 30
#define FLIP_METHOD 2
#define FRAME_SIZE DISPLAY_WIDTH* DISPLAY_HEIGHT * 3

std::string gstreamer_pipeline() {
    return "nvarguscamerasrc sensor-id=" + std::to_string(SENSOR_ID) +
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

int main() {
    signal(SIGPIPE, SIG_IGN);  // Ignore SIGPIPE signals

    std::cout << gstreamer_pipeline() << std::endl;
    VideoServer video_server(8080);
    video_server.stream();

    return 0;
}
