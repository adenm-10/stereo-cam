#ifndef CV_CAM_ARDUCAM_HPP
#define CV_CAM_ARDUCAM_HPP

#include <opencv2/opencv.hpp>
#include <atomic>
#include <string>
#include <chrono>
#include <stdexcept>

namespace cv_cam {

class Arducam {
public:
    explicit Arducam(const std::string& pipeline);
    ~Arducam();

    // Thread-safe frame getter with timeout
    bool getVideoFrame(cv::Mat& frame, unsigned int timeout_ms);

private:
    cv::VideoCapture cap;
    std::atomic<bool> running{true};  // Capture thread control
};

}  // namespace cv_cam

#endif  // CV_CAM_ARDUCAM_HPP
