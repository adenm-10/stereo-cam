#include "cv_cam/cv_cam.hpp"
#include <libcamera/libcamera/stream.h>
#include <time.h>

using namespace cv;
using namespace cv_cam;

Arducam::Arducam(const std::string& pipeline)
    : cap(pipeline, cv::CAP_GSTREAMER)
{
    if (!cap.isOpened()) {
        throw std::runtime_error("Failed to open pipeline: " + pipeline);
    }
}

Arducam::~Arducam() {
    cap.release();
}

bool Arducam::getVideoFrame(cv::Mat &frame, unsigned int timeout_ms) {
    return cap.read(frame);
}