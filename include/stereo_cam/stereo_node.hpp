#ifndef STEREO_CAM_STEREO_NODE_HPP
#define STEREO_CAM_STEREO_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <fstream>

#include "cv_cam/cv_cam.hpp"
#include "stereo_cam/calib_utils.hpp"

namespace stereo_cam {

class StereoNode : public rclcpp::Node {
public:
    explicit StereoNode(const rclcpp::NodeOptions& options);
    ~StereoNode();

    void initialize();
    void on_configure();
    void run();  // Move run() to public section

private:
    void timer_callback();
    void configure_cameras();
    void publish_images(const cv::Mat& left_img, const cv::Mat& right_img, rclcpp::Time stamp_left, rclcpp::Time stamp_right);

    std::unique_ptr<cv_cam::Arducam> left_cam_;
    std::unique_ptr<cv_cam::Arducam> right_cam_;

    // Publishers
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher left_pub_;
    image_transport::Publisher right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

    // Camera info messages
    sensor_msgs::msg::CameraInfo left_info_;
    sensor_msgs::msg::CameraInfo right_info_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    int width_;
    int height_;
    int framerate_;
    std::string frame_id_;
    std::string gst_pixel_format_;
    std::string it_pixel_format_;

    std::string right_device_file_;
    std::string right_payload_type_;

    std::string left_device_file_;
    std::string left_payload_type_;

    // Camera info managers
    std::unique_ptr<camera_info_manager::CameraInfoManager> left_info_manager_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> right_info_manager_;
    std::string left_camera_info_url_;
    std::string right_camera_info_url_;
    
    // Add frame IDs as member variables
    std::string left_camera_optical_frame_{"left_camera_optical_frame"};
    std::string right_camera_optical_frame_{"right_camera_optical_frame"};

    // Add to member variables
    bool enable_depth_;

    std::atomic<bool> running_{false};
};

} // namespace stereo_cam

#endif // STEREO_CAM_STEREO_NODE_HPP 