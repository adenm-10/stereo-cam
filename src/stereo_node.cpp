#include "stereo_cam/stereo_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <iostream>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

namespace stereo_cam {

const std::vector<ArducamCameraConfig> ARDUCAM_SUPPORTED_MODES = {
    // Format, Port, Device, width, height, fps
    
    // Laptop Port 4
    {"H264", "/dev/video4", "Laptop", 1920, 1080, 30},
    {"H264", "/dev/video4", "Laptop", 1280, 720,  30},
    {"H264", "/dev/video4", "Laptop", 800,  600,  30},
    {"H264", "/dev/video4", "Laptop", 640,  480,  30},
    {"H264", "/dev/video4", "Laptop", 640,  360,  30},
    {"H264", "/dev/video4", "Laptop", 352,  288,  30},
    {"H264", "/dev/video4", "Laptop", 320,  240,  30},

    // Laptop Port 6
    {"MJPG", "/dev/video6", "Laptop", 1920, 1080, 30},
    {"MJPG", "/dev/video6", "Laptop", 1280, 720,  30},
    {"MJPG", "/dev/video6", "Laptop", 800,  600,  30},
    {"MJPG", "/dev/video6", "Laptop", 640,  480,  30},
    {"MJPG", "/dev/video6", "Laptop", 640,  360,  30},
    {"MJPG", "/dev/video6", "Laptop", 640,  360,  20},
    {"MJPG", "/dev/video6", "Laptop", 640,  360,  15},
    {"MJPG", "/dev/video6", "Laptop", 640,  360,  10},
    {"MJPG", "/dev/video6", "Laptop", 640,  360,  5},
    {"MJPG", "/dev/video6", "Laptop", 352,  288,  30},
    {"MJPG", "/dev/video6", "Laptop", 320,  240,  30},

    // Raspi Port 0
    {"MJPG", "/dev/video0", "Raspi", 1920, 1080, 30},
    {"MJPG", "/dev/video0", "Raspi", 1280, 720,  30},
    {"MJPG", "/dev/video0", "Raspi", 800,  600,  30},
    {"MJPG", "/dev/video0", "Raspi", 640,  480,  30},
    {"MJPG", "/dev/video0", "Raspi", 640,  360,  30},
    {"MJPG", "/dev/video0", "Raspi", 640,  360,  20},
    {"MJPG", "/dev/video0", "Raspi", 640,  360,  15},
    {"MJPG", "/dev/video0", "Raspi", 640,  360,  10},
    {"MJPG", "/dev/video0", "Raspi", 640,  360,  5},
    {"MJPG", "/dev/video0", "Raspi", 352,  288,  30},
    {"MJPG", "/dev/video0", "Raspi", 320,  240,  30},

    // Raspi Port 4
    {"MJPG", "/dev/video4", "Raspi", 1920, 1080, 30},
    {"MJPG", "/dev/video4", "Raspi", 1280, 720,  30},
    {"MJPG", "/dev/video4", "Raspi", 800,  600,  30},
    {"MJPG", "/dev/video4", "Raspi", 640,  480,  30},
    {"MJPG", "/dev/video4", "Raspi", 640,  360,  30},
    {"MJPG", "/dev/video4", "Raspi", 640,  360,  20},
    {"MJPG", "/dev/video4", "Raspi", 640,  360,  15},
    {"MJPG", "/dev/video4", "Raspi", 640,  360,  10},
    {"MJPG", "/dev/video4", "Raspi", 640,  360,  5},
    {"MJPG", "/dev/video4", "Raspi", 352,  288,  30},
    {"MJPG", "/dev/video4", "Raspi", 320,  240,  30},
};

StereoNode::StereoNode(const rclcpp::NodeOptions& options, const std::string& name)
    : Node(name, options) {
    
    // Only declare and get parameters in constructor
    this->declare_parameter("left_format", "");
    this->declare_parameter("left_port", "");
    this->declare_parameter("left_device", "");

    this->declare_parameter("right_format", "");
    this->declare_parameter("right_port", "");
    this->declare_parameter("right_device", "");

    this->declare_parameter("image_width", rclcpp::ParameterValue(640));
    this->declare_parameter("image_height", rclcpp::ParameterValue(480));
    this->declare_parameter("frame_rate", rclcpp::ParameterValue(30));

    this->declare_parameter("frame_id", "camera_frame");
    this->declare_parameter("resolution_preset", "1080p");
    this->declare_parameter("left_camera_info_url", "");
    this->declare_parameter("right_camera_info_url", "");
    this->declare_parameter("stereo.baseline", 0.06);  // 60mm default
    this->declare_parameter("enable_depth", false);

    // Default to VGA if no match
    width_ = this->get_parameter("image_width").as_int();
    height_ = this->get_parameter("image_height").as_int();
    frame_rate_ = this->get_parameter("frame_rate").as_int();

    right_format_ = this->get_parameter("right_format").as_string();
    right_port_ = this->get_parameter("right_port").as_string();
    right_device_ = this->get_parameter("right_device").as_string();

    left_format_ = this->get_parameter("left_format").as_string();
    left_port_ = this->get_parameter("left_port").as_string();
    left_device_ = this->get_parameter("left_device").as_string();

    // Format, Port, Device, width, height, fps
    left_config_ = {left_format_, left_port_, left_device_, width_, height_, frame_rate_};       
    right_config_ = {right_format_, right_port_, right_device_, width_, height_, frame_rate_};

    left_config_.print();
    right_config_.print();

    bool found_preset = false;

    for (const ArducamCameraConfig& mode : ARDUCAM_SUPPORTED_MODES) {
        if (left_config_ == mode) {
            RCLCPP_INFO(get_logger(), "\nLeft Camera Config Confirmed as format: %s, port: %s, device: %s, width: %d, height: %d, frame rate: %d\n", 
                left_format_.c_str(), left_port_.c_str(), left_device_.c_str(), width_, height_, frame_rate_);
            found_preset = true;
            break;
        }
    }

    if (!found_preset) {
        RCLCPP_ERROR(this->get_logger(), "Left camera config not found, exiting");
        return;
    }
    found_preset = false;

    for (const ArducamCameraConfig& mode : ARDUCAM_SUPPORTED_MODES) {
        if (right_config_ == mode) {
            RCLCPP_INFO(get_logger(), "\nRight Camera Config Confirmed as format: %s, port: %s, device: %s, width: %d, height: %d, frame rate: %d\n", 
                right_format_.c_str(), right_port_.c_str(), right_device_.c_str(), width_, height_, frame_rate_);
            found_preset = true;
            break;
        }
    }

    if (!found_preset) {
        RCLCPP_ERROR(this->get_logger(), "Right camera config not found, exiting");
        return;
    }

    RCLCPP_INFO(get_logger(), "Final width: %d, height: %d, frame rate: %d", width_, height_, frame_rate_);

    // Get parameters
    frame_id_ = this->get_parameter("frame_id").as_string();
    
    // Get camera info URLs
    left_camera_info_url_ = this->get_parameter("left_camera_info_url").as_string();
    right_camera_info_url_ = this->get_parameter("right_camera_info_url").as_string();

    // Register callback for when node is fully up
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&StereoNode::on_configure, this));

}

void StereoNode::on_configure() {
    // Stop the timer
    timer_.reset();
    
    // Now safe to use shared_from_this()
    initialize();
}

void StereoNode::initialize() {
    // Create publishers
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    left_pub_ = it_->advertise("left/image_raw", 10);
    right_pub_ = it_->advertise("right/image_raw", 10);

    // Initialize camera info publishers
    left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);

    // Initialize cameras
    if (left_config_.device == "Laptop" && right_config_.device == "Laptop") {

        std::ostringstream pipeline;
        pipeline << "v4l2src device=" << left_port_ << " io-mode=2 ! "
                 << "video/x-h264,width=" << width_ << ",height=" << height_
                 << ",framerate=" << frame_rate_ << "/1 ! "
                 << "h264parse ! avdec_h264 ! videoconvert ! "
                 << "video/x-raw,format=BGR ! appsink";
        
        left_gst_str = pipeline.str();

        std::ostringstream right_pipeline;
        right_pipeline << "v4l2src device=" << right_port_ << " io-mode=2 ! "
                       << "image/jpeg,width=" << width_ << ",height=" << height_
                       << ",framerate=" << frame_rate_ << "/1 ! "
                       << "jpegdec ! videoconvert ! "
                       << "video/x-raw,format=BGR ! appsink";
        
        right_gst_str = right_pipeline.str();
        
    } else if (left_config_.device == "Raspi" && right_config_.device == "Raspi") {

        std::ostringstream left_pipeline;
        left_pipeline << "v4l2src device=" << left_port_ << " io-mode=2 ! "
                       << "image/jpeg,width=" << width_ << ",height=" << height_
                       << ",framerate=" << frame_rate_ << "/1 ! "
                       << "jpegdec ! videoconvert ! "
                       << "video/x-raw,format=BGR ! appsink";
        
        left_gst_str = left_pipeline.str();

        std::ostringstream right_pipeline;
        right_pipeline << "v4l2src device=" << right_port_ << " io-mode=2 ! "
                       << "image/jpeg,width=" << width_ << ",height=" << height_
                       << ",framerate=" << frame_rate_ << "/1 ! "
                       << "jpegdec ! videoconvert ! "
                       << "video/x-raw,format=BGR ! appsink";
        
        right_gst_str = right_pipeline.str();
        
    } else {
        RCLCPP_ERROR(this->get_logger(), "Mismatched configuration devices");
    }

    // cams(left_gst_str, right_gst_str);
    // right_cam_ = std::make_unique<lccv::PiCamera>(1);
    left_cam_ = std::make_unique<cv_cam::Arducam>(left_gst_str);
    right_cam_ = std::make_unique<cv_cam::Arducam>(right_gst_str);

    // Initialize camera info managers with their respective URLs
    left_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this, 
        "left_camera",
        left_camera_info_url_
    );

    right_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this,
        "right_camera", 
        right_camera_info_url_
    );

    if (!left_info_manager_->loadCameraInfo(left_camera_info_url_)) {
        RCLCPP_WARN(get_logger(), "Failed to load left camera calibration");
    }
    if (!right_info_manager_->loadCameraInfo(right_camera_info_url_)) {
        RCLCPP_WARN(get_logger(), "Failed to load right camera calibration");
    }

    // In initialize() function after creating camera info managers
    std::string calib_file = this->declare_parameter("calibration_file", "");
    calib_file = this->get_parameter("calibration_file").as_string();
    if (!calib_file.empty()) {
        sensor_msgs::msg::CameraInfo left_info, right_info;
        // check if the content of the file is empty
        std::ifstream file(calib_file);
        std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        left_info.width = width_;
        left_info.height = height_;
        right_info.width = width_;
        right_info.height = height_;
        if (CalibrationUtils::updateCameraInfo(calib_file, left_info, right_info)) {
            left_info_manager_->setCameraInfo(left_info);
            right_info_manager_->setCameraInfo(right_info);
            RCLCPP_INFO(get_logger(), "Updated camera info from calibration file");
        }
    }

    // Instead of creating a timer, we'll use a flag to control the loop
    running_ = true;

    RCLCPP_INFO(get_logger(), "Starting capture loop with period %d microseconds", 
        1000000 / frame_rate_);

    run();
}

void StereoNode::run() {
    auto period = std::chrono::microseconds(1000000 / frame_rate_);
    // auto next_capture = std::chrono::steady_clock::now();

    // Create a separate thread for camera capture
    std::thread capture_thread([this, period]() {
        while (running_ && rclcpp::ok()) {
            auto start = std::chrono::high_resolution_clock::now();
            cv::Mat left_frame, right_frame;
            
            // Capture images
            auto t0 = std::chrono::high_resolution_clock::now();
            bool left_ok = left_cam_->getVideoFrame(left_frame, 100);
            auto t1 = std::chrono::high_resolution_clock::now();
            bool right_ok = right_cam_->getVideoFrame(right_frame, 100);
            auto t2 = std::chrono::high_resolution_clock::now();

            auto dt_left = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
            auto dt_right = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            auto dt_total = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0).count();
            
            // if (dt_total > 5) {
            //     continue;
            // }
            
            // RCLCPP_INFO(this->get_logger(),
            //     "Left: %ld ms | Right: %ld ms | Total: %ld ms",
            //     dt_left, dt_right, dt_total);

            if (left_ok && right_ok) {
                // RCLCPP_INFO(this->get_logger(), "Publishing raw images - %ld", std::chrono::high_resolution_clock::now());
                publish_images(left_frame, right_frame);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to capture stereo images - Left: %d, Right: %d",
                    left_ok, right_ok);
            }

            // Sleep until next capture time
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

            // RCLCPP_INFO(get_logger(), "Frame Processed");

            if (duration < period) {
                std::this_thread::sleep_for(period - duration);
            }
        }
    });

    // Detach the thread so it runs independently
    capture_thread.detach();
}

StereoNode::~StereoNode() {
    running_ = false;
}

void StereoNode::publish_images(const cv::Mat& left_img, const cv::Mat& right_img) {
    auto stamp = this->now();

    // std::cout << "left_bgr: channels = " << left_img.channels()
    //       << ", step = " << left_img.step << std::endl;
    
    // Convert and publish left image
    sensor_msgs::msg::Image::SharedPtr left_msg = 
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_img).toImageMsg();
    left_msg->header.stamp = stamp;
    left_msg->header.frame_id = left_camera_frame_;
    left_pub_.publish(left_msg);

    // Get and publish left camera info
    sensor_msgs::msg::CameraInfo left_info = left_info_manager_->getCameraInfo();
    left_info.header.stamp = stamp;
    left_info.header.frame_id = left_camera_frame_;
    left_info_pub_->publish(left_info);

    // Convert and publish right image
    sensor_msgs::msg::Image::SharedPtr right_msg = 
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_img).toImageMsg();
    right_msg->header.stamp = stamp;
    right_msg->header.frame_id = right_camera_frame_;
    right_pub_.publish(right_msg);

    // Get and publish right camera info
    sensor_msgs::msg::CameraInfo right_info = right_info_manager_->getCameraInfo();
    right_info.header.stamp = stamp;
    right_info.header.frame_id = right_camera_frame_;
    right_info_pub_->publish(right_info);
}

} // namespace stereo_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_cam::StereoNode)  