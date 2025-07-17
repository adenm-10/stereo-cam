#include <rclcpp/rclcpp.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <limits>
#include <vector>
#include <algorithm>

#include "navis_msgs/msg/control_out.hpp"

class ClosestObstacleDetector : public rclcpp::Node
{
public:
  ClosestObstacleDetector()
  : Node("closest_obstacle_detector")
  {
    this->declare_parameter("baseline", 0.152);
    this->declare_parameter("focal_length_px", 479.14);
    this->declare_parameter("roi_fraction", 0.3); // Center ROI size (50% of width & height)

    baseline_ = this->get_parameter("baseline").as_double();
    focal_length_px_ = this->get_parameter("focal_length_px").as_double();
    roi_fraction_ = this->get_parameter("roi_fraction").as_double();

    sub_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
      "/disparity", rclcpp::SensorDataQoS(),
      std::bind(&ClosestObstacleDetector::disparityCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<navis_msgs::msg::ControlOut>(
      "/control_output", 10);
      
    RCLCPP_INFO(this->get_logger(), "ClosestObstacleDetector node started.");
  }

private:
  void disparityCallback(const stereo_msgs::msg::DisparityImage::SharedPtr msg)
  {
    // Access msg->image, which is the actual disparity image
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg->image, msg->image.encoding);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat disparity = cv_ptr->image;
    RCLCPP_DEBUG(this->get_logger(), "Disparity image dimensions: %d x %d, type: %d", disparity.cols, disparity.rows, disparity.type());

    // Convert to float32 if needed
    if (disparity.type() != CV_32F)
    {
      disparity.convertTo(disparity, CV_32F);
      RCLCPP_DEBUG(this->get_logger(), "Converted disparity to CV_32F.");
    }

    // Scale disparity if needed
    double mean_disp = cv::mean(disparity)[0];
    RCLCPP_DEBUG(this->get_logger(), "Mean disparity before scaling: %.2f", mean_disp);
    if (mean_disp > 1000.0)
    {
      disparity = disparity / 16.0;
      RCLCPP_DEBUG(this->get_logger(), "Scaled disparity by 1/16.");
    }

    // Center ROI
    int w = disparity.cols;
    int h = disparity.rows;
    int roi_w = static_cast<int>(w * roi_fraction_);
    int roi_h = static_cast<int>(h * roi_fraction_);
    int x0 = (w - roi_w) / 2;
    int y0 = (h - roi_h) / 2;
    cv::Rect roi_rect(x0, y0, roi_w, roi_h);
    RCLCPP_DEBUG(this->get_logger(), "ROI rectangle: x=%d, y=%d, w=%d, h=%d", x0, y0, roi_w, roi_h);

    if (x0 < 0 || y0 < 0 || roi_w <= 0 || roi_h <= 0 || (x0 + roi_w) > w || (y0 + roi_h) > h)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid ROI parameters. Skipping this frame.");
      return;
    }

    cv::Mat roi = disparity(roi_rect);
    RCLCPP_DEBUG(this->get_logger(), "Extracted ROI.");

    // Valid disparity mask
    cv::Mat valid_mask = roi > 0.1;
    int num_valid_pixels = cv::countNonZero(valid_mask);
    RCLCPP_DEBUG(this->get_logger(), "Number of valid disparity pixels in ROI: %d", num_valid_pixels);

    std::vector<float> valid_depths;
    valid_depths.reserve(num_valid_pixels);

    std::vector<float> valid_x;
    valid_x.reserve(num_valid_pixels);

    for (int y = 0; y < roi.rows; ++y)
    {
      for (int x = 0; x < roi.cols; ++x)
      {
        float d = roi.at<float>(y, x);
        if (d > 0.1f)
        {
          float z = (focal_length_px_ * baseline_) / d;
          valid_depths.push_back(z);
          valid_x.push_back(x);
        }
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "Collected %zu valid depth values.", valid_depths.size());

    // Check if enough valid pixels
    size_t min_valid_pixels = (roi.rows * roi.cols) * 0.05; // require at least 5% valid
    if (valid_depths.size() < min_valid_pixels)
    {
      RCLCPP_WARN(this->get_logger(), "Not enough valid pixels in ROI: %zu (minimum required: %zu)", valid_depths.size(), min_valid_pixels);
      return;
    }

    // Use median for robustness
    // std::nthsmoothed__element(valid_depths.begin(), valid_depths.begin() + valid_depths.size() / 2, valid_depths.end());
    
    // Find minimum
    float min_depth = *std::min_element(valid_depths.begin(), valid_depths.end());

    // Find lower quartile (25th percentile)
    size_t q1_idx = valid_depths.size() / 4;
    std::nth_element(valid_depths.begin(), valid_depths.begin() + q1_idx, valid_depths.end());
    float lower_quartile_depth = valid_depths[q1_idx];

    std::vector<int> x_near_lq;
    for (size_t i = 0; i < valid_depths.size(); ++i)
    {
      if (std::abs(valid_depths[i] - lower_quartile_depth) < 0.1f)  // adjust tolerance
      {
        x_near_lq.push_back(valid_x[i]);
      }
    }

    float avg_x = 0.0f;

    if (!x_near_lq.empty())
    {
      for (int x_val : x_near_lq)
      {
        avg_x += x_val;
      }
      avg_x /= x_near_lq.size();
    }
    else
    {
      avg_x = -1.0f;
    }

    float avg_x_global = avg_x + x0;

    float image_center_x = w / 2.0f;
    float offset_from_center = avg_x_global - image_center_x;

    RCLCPP_INFO(this->get_logger(),
      "Closest mass at ~%.2f m. Avg x offset from center: %.1f px (points: %zu).",
      lower_quartile_depth,
      offset_from_center,
      x_near_lq.size());

    // Publisher
    if (obstacle_flag) {
      if (lower_quartile_depth > 2.0f) {
        RCLCPP_WARN(this->get_logger(), "Obstacle detection flag reset.");
        obstacle_flag = false;
      }
    } else {
      if (lower_quartile_depth < 2.0f) {

        navis_msgs::msg::ControlOut obstacle_is_msg;
        obstacle_is_msg.buzzer_strength = 0; // No haptic feedback
        obstacle_is_msg.speaker_wav_index = 21; // "Obstacle is"
      
        navis_msgs::msg::ControlOut two;
        two.buzzer_strength = 0; // No haptic feedback
        two.speaker_wav_index = 9; // "2" meters
      
        navis_msgs::msg::ControlOut meters;
        meters.buzzer_strength = 0; // No haptic feedback
        meters.speaker_wav_index = 7; // "Meters"

        RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f m, setting flag and publishing.", lower_quartile_depth);
        obstacle_flag = true;
        pub_->publish(obstacle_is_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(250));
        pub_->publish(two);
        rclcpp::sleep_for(std::chrono::milliseconds(250));
        pub_->publish(meters);
      }
    }
  }


  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr sub_;
  double baseline_;
  double focal_length_px_;
  double roi_fraction_;
  float smoothed_l_r_x_ = -1.0;  // Initialize at start

  rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr pub_;

  // Published messages - to be reused
  bool obstacle_flag = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClosestObstacleDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
