#include <opencv2/opencv.hpp>

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
    std::string window_title = "CSI Camera";

    cv::VideoCapture video_capture(gstreamer_pipeline(), cv::CAP_GSTREAMER);

    if (video_capture.isOpened()) {
        try {
            cv::namedWindow(window_title, cv::WINDOW_AUTOSIZE);
            while (true) {
                cv::Mat frame;
                bool ret_val = video_capture.read(frame);
                if (!ret_val) {
                    break;
                }
                cv::imshow(window_title, frame);
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
        cv::destroyAllWindows();
    } else {
        std::cerr << "Error: Unable to open camera" << std::endl;
    }

    return 0;
}
