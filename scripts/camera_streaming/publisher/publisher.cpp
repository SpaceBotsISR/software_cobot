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

class VideoStreamer {
   public:
    VideoStreamer(std::string ip_address, int port) : isConnected(false) {
        client_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (client_socket < 0) {
            std::cerr << "Error creating socket!" << std::endl;
            return;
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());

        // Connect to server
        if (connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Error connecting to server!" << std::endl;
            return;
        }

        isConnected = true;

        video_capture = cv::VideoCapture(gstreamer_pipeline(), cv::CAP_GSTREAMER);
    }

    void stream() {
        if (!isConnected) {
            std::cerr << "Not connected to server!" << std::endl;
            return;
        }

        if (!video_capture.isOpened()) {
            std::cerr << "Error: Unable to open camera" << std::endl;
            return;
        }

        try {
            while (true) {
                cv::Mat frame;
                if (!video_capture.read(frame)) {
                    break;
                }

                send_frame(frame);
            }

            video_capture.release();
        } catch (const std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        }

        close(client_socket);
    }

    ~VideoStreamer() {
        if (isConnected) {
            close(client_socket);
        }
    }

   private:
    int client_socket;
    struct sockaddr_in server_addr;
    cv::VideoCapture video_capture;
    bool isConnected;

    void send_frame(cv::Mat& frame) {
        ssize_t msg_size = frame.total() * frame.elemSize();
        unsigned char* frame_data_ptr = frame.data;
        std::string buff_size_msg = std::to_string(msg_size);

        send(client_socket, buff_size_msg.c_str(), buff_size_msg.length() + 1, 0);

        while (msg_size > 0) {
            ssize_t bytes_sent = send(client_socket, frame_data_ptr, msg_size, 0);

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
    VideoStreamer video_streamer("127.0.0.1", 8080);
    video_streamer.stream();

    return 0;
}