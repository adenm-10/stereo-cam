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

    // Add configuration / pipeline construction in this class?

}

Arducam::~Arducam() {
    cap.release();
}

bool Arducam::getVideoFrame(cv::Mat &frame, unsigned int timeout_ms)
{
    if (!running.load(std::memory_order_acquire))
        return false;

    auto start_time = std::chrono::high_resolution_clock::now();
    bool timeout_reached = false;
    timespec req = {0, 1000000};  // Sleep 1ms

    while (!timeout_reached) {
        if (cap.read(frame)) {
            return true;
        }

        nanosleep(&req, nullptr);

        timeout_reached = (std::chrono::high_resolution_clock::now() - start_time >
                           std::chrono::milliseconds(timeout_ms));
    }

    return false; // Timeout occurred
}

// void Arducam::captureLoop(cv::VideoCapture& cap,
//     std::mutex& mtx,
//     cv::Mat& latestFrame,
//     std::atomic<bool>& frameready,
//     std::atomic<bool>& running)
// {
//     while (running.load(std::memory_order_acquire)) {
//         std::cout << ""
//         cv::Mat frame;
//         if (cap.read(frame)) {
//             std::lock_guard<std::mutex> lock(mtx);
//             latestFrame = frame.clone();
//             frameready.store(true, std::memory_order_release);
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(1));
//     }
// }

// bool Arducam::getSynchronizedFrames(cv::Mat& frame1, cv::Mat& frame2, unsigned int timeout_ms) {
//     auto start_time = std::chrono::high_resolution_clock::now();
//     timespec req = {0, 1000000};  // 1ms
//     bool timeout_reached = false;

//     while (!(frameready1.load(std::memory_order_acquire) &&
//              frameready2.load(std::memory_order_acquire)) &&
//            !timeout_reached) {
//         nanosleep(&req, nullptr);
//         timeout_reached = (
//             std::chrono::high_resolution_clock::now() - start_time >
//             std::chrono::milliseconds(timeout_ms));
//     }

//     if (frameready1.load(std::memory_order_acquire) &&
//         frameready2.load(std::memory_order_acquire)) {

//         std::lock_guard<std::mutex> lock1(mtx1);
//         std::lock_guard<std::mutex> lock2(mtx2);

//         frame1 = latestFrame1.clone();
//         frame2 = latestFrame2.clone();

//         frameready1.store(false, std::memory_order_release);
//         frameready2.store(false, std::memory_order_release);
//         return true;
//     }

//     return false;
// }