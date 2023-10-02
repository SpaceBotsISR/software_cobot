#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <opencv2/opencv.hpp>

const int sensor_id = 0;
const int capture_width = 1920;
const int capture_height = 1080;
const int display_width = 960;
const int display_height = 540;
const int framerate = 30;
const int flip_method = 0;

class VideoStreamClient {
   public:
    VideoStreamClient(std::string ip_address, int port) : isConnected(false) {
        client_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (client_socket < 0) {
            std::cerr << "Error creating socket!" << std::endl;
            return;
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());

        if (connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Error connecting to server!" << std::endl;
            return;
        }

        isConnected = true;
    }

    void receiveAndDisplay() {
        if (!isConnected) {
            std::cerr << "Not connected to server!" << std::endl;
            return;
        }

        cv::namedWindow("Video Stream", cv::WINDOW_AUTOSIZE);

        while (isConnected) {
            cv::Mat frame = receive_frame();
            if (frame.empty()) {
                std::cerr << "Received an empty frame or error occurred!" << std::endl;
                break;
            }

            cv::imshow("Video Stream", frame);
            if (cv::waitKey(1) >= 0) break;  // Exit on any key press
        }
    }

    ~VideoStreamClient() {
        if (isConnected) {
            close(client_socket);
        }
    }

   private:
    int client_socket;
    struct sockaddr_in server_addr;
    bool isConnected;

    cv::Mat receive_frame() {
        int imgSize = 1920 * 1080 * 3;  // Assuming 3 bytes per pixel (BGR format)
        std::vector<uchar> img_data(imgSize);

        ssize_t bytesReceived = 0;
        uchar* buffer_ptr = img_data.data();

        while (bytesReceived < imgSize) {
            ssize_t bytesRead = recv(client_socket, buffer_ptr, imgSize - bytesReceived, 0);
            if (bytesRead <= 0) {
                std::cerr << "Error receiving data or connection closed by server!" << std::endl;
                isConnected = false;
                return cv::Mat();
            }
            bytesReceived += bytesRead;
            buffer_ptr += bytesRead;
        }

        return cv::Mat(capture_height, capture_width, CV_8UC3, img_data.data());
    }
};

int main() {
    VideoStreamClient client("127.0.0.1", 8080);
    client.receiveAndDisplay();

    return 0;
}
