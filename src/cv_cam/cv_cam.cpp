#include "cv_cam/cv_cam.hpp"
#include <libcamera/libcamera/stream.h>
#include <time.h>

using namespace cv;
using namespace cv_cam;

Arducam::Arducam(const std::string& device_file,
                 const std::string& payload_type,
                 const std::string& pixel_format,
                 int width,
                 int height,
                 int framerate)
{
    std::ostringstream pipeline;

    // Start with v4l2src
    // pipeline << "v4l2src device=" << device_file << " io-mode=2 ! ";
    pipeline << "v4l2src device=" << device_file << " io-mode=0";

    if (payload_type == "YUYV") {
        pipeline << " ! video/x-raw, format=YUY2, width=" << width << ", height=" << height
                << ", framerate=" << framerate << "/1 ! ";
    } else if (payload_type == "MJPG") {
        pipeline << " ! image/jpeg, width=" << width << ", height=" << height
                << ", framerate=" << framerate << "/1 ! jpegdec ! ";
    } else if (payload_type == "H264") {
        pipeline << " ! video/x-h264, width=" << width << ", height=" << height
                << ", framerate=" << framerate << "/1 ! h264parse ! avdec_h264 ! ";
    } else {
        throw std::runtime_error("Unsupported payload_type: " + payload_type + "\nPayload must be of type \"YUYV\", \"MJPG\", or \"H264\"");
    }

    // Final format and sink
    pipeline << "videoconvert ! video/x-raw, format=" << pixel_format << " ! appsink";
    std::string gst_str = pipeline.str();
    cap.open(gst_str, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        cap.release();
        throw std::runtime_error("Failed to open GStreamer pipeline: " + gst_str);
    }
}

Arducam::~Arducam() {
    cap.release();
}

bool Arducam::getVideoFrame(cv::Mat &frame, unsigned int timeout_ms) {
    return cap.read(frame);
}