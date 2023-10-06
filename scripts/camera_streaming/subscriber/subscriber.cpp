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

class VideoClient {
   public:
    VideoClient(int port) {
        // Create socket
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket < 0) {
            std::cerr << "Error creating socket!" << std::endl;
            exit(1);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = INADDR_ANY;

        // Bind
        bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr));

        // Listen
        listen(server_socket, 1);
        std::cout << "Listening for incoming connections..." << std::endl;

        // Accept client connection
        client_socket = accept(server_socket, NULL, NULL);
        std::cout << "Connected!" << std::endl;
    }

    void run() {
        cv::namedWindow("Received Frames", cv::WINDOW_AUTOSIZE);

        while (true) {
            ssize_t msg_size = 1555200;
            ssize_t total_bytes_received = 0;
            ssize_t offset = 0;

            read_and_display(msg_size);

            int keyCode = cv::waitKey(10) & 0xFF;
            if (keyCode == 27 || keyCode == 'q') {
                break;
            }
        }
    }

    ~VideoClient() {
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

        std::cout << "\n\nlen_buffer: " << len_buffer << std::endl;
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
            std::cout << "Recived: " << bytes_received << " | Total: " << total_bytes_received
                      << std::endl;
        }

        std::memcpy(frame.data, buffer, total_bytes_received);
        cv::imshow("Received Frames", frame);
        delete[] buffer;
    }
};

int main() {
    VideoClient client(8080);
    client.run();

    return 0;
}
