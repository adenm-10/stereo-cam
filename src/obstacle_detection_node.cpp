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
    this->declare_parameter("roi_width_fraction", 0.2);  // How much of the center width to use for ROI
    this->declare_parameter("roi_height_fraction", 0.3); // How much of the center height to use for ROI

    baseline_ = this->get_parameter("baseline").as_double();
    focal_length_px_ = this->get_parameter("focal_length_px").as_double();
    roi_width_fraction_ = this->get_parameter("roi_width_fraction").as_double();
    roi_height_fraction_ = this->get_parameter("roi_height_fraction").as_double();

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

    if (disparity.type() != CV_32F)
    {
      disparity.convertTo(disparity, CV_32F);
    }

    if (cv::mean(disparity)[0] > 1000.0)
    {
      disparity = disparity / 16.0;
    }

    // Narrower ROI to exclude aisle walls and focus on the center path
    int w = disparity.cols;
    int h = disparity.rows;
    int roi_w = static_cast<int>(w * roi_width_fraction_);
    int roi_h = static_cast<int>(h * roi_height_fraction_);
    int x0 = (w - roi_w) / 2;
    int y0 = 0; // Start ROI from the top of the image to avoid the ground plane
    cv::Rect roi_rect(x0, y0, roi_w, roi_h);

    if (x0 < 0 || y0 < 0 || roi_w <= 0 || roi_h <= 0 || (x0 + roi_w) > w || (y0 + roi_h) > h)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid ROI parameters. Skipping this frame.");
      return;
    }

    cv::Mat roi = disparity(roi_rect).clone(); // Use clone to work on a copy

    // Apply a median filter to remove salt-and-pepper noise
    cv::medianBlur(roi, roi, 5); // 5x5 kernel, must be an odd number

    // To find the most representative obstacle distance, we'll use a histogram to find
    // the mode (most common) disparity value. This is more robust than percentiles,
    // especially when the ground plane creates a gradient of disparity values.
    
    // Define histogram parameters. We are interested in disparities from 1 to 256.
    int hist_bins = 255; // 255 bins for values 1-256
    float hist_range_params[] = {1.0f, 256.0f};
    const float* hist_range = {hist_range_params};

    // Create a mask to exclude invalid disparities (<= 0.1)
    cv::Mat mask;
    cv::inRange(roi, cv::Scalar(0.1), cv::Scalar(256), mask);

    // Check if we have enough valid pixels to begin with
    size_t num_valid_pixels = cv::countNonZero(mask);
    size_t min_valid_pixels = (roi.rows * roi.cols) * 0.05; // require at least 5% valid
    if (num_valid_pixels < min_valid_pixels)
    {
      RCLCPP_WARN(this->get_logger(), "Not enough valid pixels in ROI: %zu (minimum required: %zu)", num_valid_pixels, min_valid_pixels);
      return;
    }

    // Calculate histogram of valid disparity values
    cv::Mat hist;
    cv::calcHist(&roi, 1, 0, mask, hist, 1, &hist_bins, &hist_range);

    // Find the bin with the highest value (the mode)
    double max_val = 0;
    cv::Point max_loc;
    cv::minMaxLoc(hist, 0, &max_val, 0, &max_loc);

    // Ensure the peak in the histogram is significant enough to be a real object
    if (max_val < num_valid_pixels * 0.1) { // Require the peak to contain at least 10% of the valid pixels
        RCLCPP_WARN(this->get_logger(), "No dominant obstacle found in ROI. Valid pixels: %zu, peak count: %.0f", num_valid_pixels, max_val);
        return;
    }

    // The `max_loc.y` gives the bin index. Convert this back to a disparity value.
    // The disparity value is the center of the bin.
    float bin_width = hist_range_params[1] / hist_bins;
    float mode_disparity = (max_loc.y * bin_width) + (bin_width / 2.0f);

    // Calculate depth from the mode disparity
    float depth_m = (focal_length_px_ * baseline_) / mode_disparity;

    RCLCPP_INFO(this->get_logger(), "Mode disparity: %.2f px, Estimated distance: %.2f m", mode_disparity, depth_m);

    // Publisher logic
    if (obstacle_flag) {
      // If an obstacle was previously detected, check if it's now further than 2m
      if (depth_m > 2.0f) {
        RCLCPP_INFO(this->get_logger(), "Obstacle flag reset.");
        obstacle_flag = false;
      }
    } else {
      // If no obstacle was detected, check if one has appeared within 2m
      if (depth_m < 2.0f) {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f m, setting flag and publishing.", depth_m);
        
        navis_msgs::msg::ControlOut obstacle_is_msg;
        obstacle_is_msg.buzzer_strength = 0;
        obstacle_is_msg.speaker_wav_index = 21; // "Obstacle is"
      
        navis_msgs::msg::ControlOut two;
        two.buzzer_strength = 0;
        two.speaker_wav_index = 9; // "2"
      
        navis_msgs::msg::ControlOut meters;
        meters.buzzer_strength = 0;
        meters.speaker_wav_index = 7; // "Meters"

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
  rclcpp::Publisher<navis_msgs::msg::ControlOut>::SharedPtr pub_;
  double baseline_;
  double focal_length_px_;
  double roi_width_fraction_;
  double roi_height_fraction_;
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
