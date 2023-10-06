#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

const int frame_size = 1555200;
const int sensor_id = 0;
const int capture_width = 1920;
const int capture_height = 1080;
const int display_width = 960;
const int display_height = 540;
const int framerate = 30;
const int flip_method = 0;

class VideoReceiver {
   public:
    VideoReceiver(std::string ip_address, int port) {
        // Create socket
        client_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (client_socket < 0) {
            std::cerr << "Error creating socket!" << std::endl;
            exit(1);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());

        if (connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Error connecting to server!" << std::endl;
            exit(1);
        }

        cv::namedWindow("Received Frames", cv::WINDOW_AUTOSIZE);
    }

    void run() {
        cv::namedWindow("Received Frames", cv::WINDOW_AUTOSIZE);

        while (true) {
            ssize_t msg_size = frame_size;
            ssize_t total_bytes_received = 0;
            ssize_t offset = 0;

            read_and_display(msg_size);

            int keyCode = cv::waitKey(10) & 0xFF;
            if (keyCode == 27 || keyCode == 'q') {
                break;
            }
        }
    }

    ~VideoReceiver() {
        close(client_socket);
        close(server_socket);
    }

   private:
    int server_socket, client_socket;
    struct sockaddr_in server_addr;

    ssize_t get_msg_size() {
        char len_buffer[48] = {0};

        ssize_t bytes_received = recv(client_socket, len_buffer, sizeof(len_buffer), 0);
        if (bytes_received <= 0) {
            std::cerr << "Connection closed or error" << std::endl;
            exit(1);
        }

        return std::stol(len_buffer);
    }

    void read_and_display(ssize_t msg_size) {
        cv::Mat frame(display_height, display_width, CV_8UC3);
        char* buffer = new char[msg_size];

        ssize_t total_bytes_received = 0;
        ssize_t offset = 0;
        ssize_t bytes_received;

        while (total_bytes_received < msg_size) {
            bytes_received =
                recv(client_socket, buffer + offset, msg_size - total_bytes_received, 0);

            if (bytes_received <= 0) {
                std::cerr << "Connection closed or error" << std::endl;
                exit(1);
            }

            total_bytes_received += bytes_received;
            offset += bytes_received;  // Update the offset by the number of bytes received
        }

        std::memcpy(frame.data, buffer, total_bytes_received);
        cv::imshow("Received Frames", frame);
        delete[] buffer;
    }
};

int main() {
    VideoReceiver receiver("127.0.0.1", 8080);
    receiver.run();

    return 0;
}
