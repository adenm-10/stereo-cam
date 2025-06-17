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

#include "lccv/lccv.hpp"
#include "cv_cam/cv_cam.hpp"
#include "stereo_cam/icm20948.hpp"
#include "stereo_cam/calib_utils.hpp"

namespace stereo_cam {

struct CameraConfig {
    static constexpr int MAX_WIDTH = 3280;
    static constexpr int MAX_HEIGHT = 2464;
    static constexpr double FOCAL_LENGTH = 2.6; // mm
    static constexpr double BASELINE = 0.06; // meters (60mm)
    static constexpr double FOV_H = 73.0; // degrees
    static constexpr double FOV_V = 50.0; // degrees
    
    struct Resolution {
        int width;
        int height;
        double fps;
    };
    
    static const std::vector<Resolution> SUPPORTED_MODES;  // Declaration only
};

struct ArducamCameraConfig {
    std::string format;
    std::string port;
    std::string device;
    int width;
    int height;
    int fps;

    bool operator==(const ArducamCameraConfig& other) const {
        return format == other.format &&
               port == other.port &&
               device == other.device &&
               width == other.width &&
               height == other.height &&
               fps == other.fps;
    }

    void print() const {
        std::cout << "ArducamCameraConfig:" << std::endl;
        std::cout << "  format: " << format << std::endl;
        std::cout << "  port: " << port << std::endl;
        std::cout << "  device: " << device << std::endl;
        std::cout << "  width: " << width << std::endl;
        std::cout << "  height: " << height << std::endl;
        std::cout << "  fps: " << fps << std::endl;
    }
};

class StereoNode : public rclcpp::Node {
public:
    explicit StereoNode(const rclcpp::NodeOptions& options, const std::string& name = "stereo_node");
    ~StereoNode();

    void initialize();
    void on_configure();
    void run();  // Move run() to public section

private:
    void timer_callback();
    void configure_cameras();
    void publish_images(const cv::Mat& left_img, const cv::Mat& right_img);

    // Camera instances
    // std::unique_ptr<lccv::PiCamera> left_cam_;
    // std::unique_ptr<lccv::PiCamera> right_cam_;
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
    int frame_rate_;
    std::string frame_id_;

    std::string right_format_;
    std::string right_port_;
    std::string right_device_;

    std::string left_format_;
    std::string left_port_;
    std::string left_device_;

    ArducamCameraConfig left_config_;
    ArducamCameraConfig right_config_;

    std::string left_gst_str;
    std::string right_gst_str;

    // Camera info managers
    std::unique_ptr<camera_info_manager::CameraInfoManager> left_info_manager_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> right_info_manager_;
    std::string left_camera_info_url_;
    std::string right_camera_info_url_;

    // Add broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    
    // Add frame IDs as member variables
    std::string left_camera_frame_{"left_camera_frame"};
    std::string right_camera_frame_{"right_camera_frame"};
    double stereo_baseline_;

    // Add function declaration
    void setup_static_transforms();

    // Add to member variables
    bool enable_depth_;

    std::atomic<bool> running_{false};
};

} // namespace stereo_cam

#endif // STEREO_CAM_STEREO_NODE_HPP 