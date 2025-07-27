#include "stereo_cam/stereo_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <iostream>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

namespace stereo_cam {

StereoNode::StereoNode(const rclcpp::NodeOptions& options)
    : Node("stereo_node", options) {
    
    // Parameter Declaration
    this->declare_parameter("left_device_file", "");
    this->declare_parameter("left_payload_type", "");
    
    this->declare_parameter("right_device_file", "");
    this->declare_parameter("right_payload_type", "");
    
    this->declare_parameter("gst_pixel_format", "BGR");
    this->declare_parameter("it_pixel_format", "bgr8");
    this->declare_parameter("image_width", rclcpp::ParameterValue(640));
    this->declare_parameter("image_height", rclcpp::ParameterValue(480));
    this->declare_parameter("frame_rate", rclcpp::ParameterValue(30));

    this->declare_parameter("frame_id", "camera_link");
    this->declare_parameter("left_camera_info_url", "");
    this->declare_parameter("right_camera_info_url", "");
    this->declare_parameter("enable_depth", false);

    // Parameter Initialization
    width_ = this->get_parameter("image_width").as_int();
    height_ = this->get_parameter("image_height").as_int();
    framerate_ = this->get_parameter("frame_rate").as_int();
    gst_pixel_format_ = this->get_parameter("gst_pixel_format").as_string();
    it_pixel_format_ = this->get_parameter("it_pixel_format").as_string();

    right_device_file_ = this->get_parameter("right_device_file").as_string();
    right_payload_type_ = this->get_parameter("right_payload_type").as_string();

    left_device_file_ = this->get_parameter("left_device_file").as_string();
    left_payload_type_ = this->get_parameter("left_payload_type").as_string();

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
    left_pub_ = it_->advertise("left/image_raw", 0);
    right_pub_ = it_->advertise("right/image_raw", 0);

    // Initialize camera info publishers
    left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 0);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 0);
    
    try {
        RCLCPP_INFO(get_logger(),
                    "Initializing left camera with device='%s', payload='%s', format='%s', resolution=%dx%d, framerate=%d fps",
                    left_device_file_.c_str(),
                    left_payload_type_.c_str(),
                    gst_pixel_format_.c_str(),
                    width_,
                    height_,
                    framerate_);

        left_cam_ = std::make_unique<cv_cam::Arducam>(left_device_file_,
                                                      left_payload_type_,
                                                      gst_pixel_format_,
                                                      width_,
                                                      height_,
                                                      framerate_);

        RCLCPP_INFO(get_logger(), 
                    "Initializing right camera with device='%s', payload='%s', format='%s', resolution=%dx%d, framerate=%d fps",
                    right_device_file_.c_str(),
                    right_payload_type_.c_str(),
                    gst_pixel_format_.c_str(),
                    width_,
                    height_,
                    framerate_);
        
        right_cam_ = std::make_unique<cv_cam::Arducam>(right_device_file_,
                                                       right_payload_type_,
                                                       gst_pixel_format_,
                                                       width_,
                                                       height_,
                                                       framerate_);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), 
            "%s\nEnsure device configuration (device file, payload type, pixel format, resolution, and framerate) "
            "is available by running: \"v4l2-ctl --device=/dev/videoX --list-formats-ext\"", 
            e.what());
         rclcpp::shutdown();
         return;
    }    

    RCLCPP_INFO(get_logger(), "Cameras successfully instantiated");

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

    // In initialize() function after creating camera info managers
    std::string calib_file = this->declare_parameter("calibration_file", "");
    calib_file = this->get_parameter("calibration_file").as_string();
    if (!calib_file.empty()) {
        sensor_msgs::msg::CameraInfo left_info, right_info;

        std::ifstream file(calib_file);
        std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        
        left_info.width = width_;
        left_info.height = height_;
        right_info.width = width_;
        right_info.height = height_;
        
        if (CalibrationUtils::updateCameraInfo(calib_file, left_info, right_info)) {
            left_info_manager_->setCameraInfo(left_info);
            right_info_manager_->setCameraInfo(right_info);
            RCLCPP_INFO(get_logger(), "Updated camera info from stereo calibration file");
        }
    }

    // Instead of creating a timer, we'll use a flag to control the loop
    running_ = true;

    RCLCPP_INFO(get_logger(), "Starting capture loop with period %d microseconds", 
        1000000 / framerate_);

    run();
}

void StereoNode::run() {
    auto period = std::chrono::microseconds(1000000 / framerate_);

    // Create a separate thread for camera capture
    std::thread capture_thread([this, period]() {
        while (running_ && rclcpp::ok()) {

            cv::Mat left_frame, right_frame;
            bool left_ok = false, right_ok = false;
            rclcpp::Time stamp_left, stamp_right;

            left_ok = left_cam_->getVideoFrame(left_frame, 100);
            stamp_left = this->now();

            right_ok = right_cam_->getVideoFrame(right_frame, 100);
            stamp_right = this->now();
            
            // Also log stamps (as floating point seconds for easy reading)
            RCLCPP_DEBUG(this->get_logger(),
                "Stamps - Left: %.6f, Right: %.6f, Diff: %.6f",
                stamp_left.seconds(),
                stamp_right.seconds(),
                (stamp_left - stamp_right).seconds());
            
            if (left_ok && right_ok) {
                publish_images(left_frame, right_frame, stamp_left, stamp_right);
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Failed to capture stereo images - Left: %d, Right: %d", left_ok, right_ok);
            }
        }
    });

    // Detach the thread so it runs independently
    capture_thread.detach();
}

StereoNode::~StereoNode() {
    running_ = false;
}

void StereoNode::publish_images(const cv::Mat& left_img, const cv::Mat& right_img, rclcpp::Time stamp_left, rclcpp::Time stamp_right) {
    
    // Create a single header to reuse
    std_msgs::msg::Header header_left;
    header_left.stamp = stamp_left;

    // Left Image
    header_left.frame_id = left_camera_optical_frame_;
    auto left_msg = cv_bridge::CvImage(header_left, it_pixel_format_, left_img).toImageMsg();
    left_pub_.publish(left_msg);

    // Left Camera Info
    sensor_msgs::msg::CameraInfo left_info = left_info_manager_->getCameraInfo();
    left_info.header = header_left;
    left_info_pub_->publish(left_info);

    // Right image
    std_msgs::msg::Header header_right;
    header_right.stamp = stamp_right;
    header_right.frame_id = right_camera_optical_frame_;
    auto right_msg = cv_bridge::CvImage(header_right, it_pixel_format_, right_img).toImageMsg();
    right_pub_.publish(right_msg);

    // Right Camera Info
    sensor_msgs::msg::CameraInfo right_info = right_info_manager_->getCameraInfo();
    right_info.header = header_right;
    right_info_pub_->publish(right_info);
}

} // namespace stereo_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_cam::StereoNode)  