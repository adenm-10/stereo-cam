#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <limits>
#include <cmath>

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
  : Node("obstacle_detector")
  {
    // Subscription to stereo point cloud
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points2", 10,
      std::bind(&ObstacleDetector::pointCloudCallback, this, std::placeholders::_1));

    // Publisher for closest obstacle distance
    closest_pub_ = this->create_publisher<std_msgs::msg::Int32>("/closest_obstacle_cm", 10);

    RCLCPP_INFO(this->get_logger(), "Obstacle detector node started.");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Detection Callback Called");
    
    // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // RCLCPP_INFO(this->get_logger(), "Input points: %zu", pcl_cloud->points.size());

    // float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
    // float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();
    // float min_z = std::numeric_limits<float>::max(), max_z = std::numeric_limits<float>::lowest();
    // for (const auto& p : pcl_cloud->points) {
    //     if (p.x < min_x) min_x = p.x;
    //     if (p.x > max_x) max_x = p.x;
    //     if (p.y < min_y) min_y = p.y;
    //     if (p.y > max_y) max_y = p.y;
    //     if (p.z < min_z) min_z = p.z;
    //     if (p.z > max_z) max_z = p.z;
    // }
    // RCLCPP_INFO(this->get_logger(), "Cloud X: min %.2f, max %.2f", min_x, max_x);
    // RCLCPP_INFO(this->get_logger(), "Cloud Y: min %.2f, max %.2f", min_y, max_y);
    // RCLCPP_INFO(this->get_logger(), "Cloud Z: min %.2f, max %.2f", min_z, max_z);

    // [obstacle_detector-4] [INFO] [1751224815.412252550] [obstacle_detector]: Cloud X: min -2835.06, max 17222.87
    // [obstacle_detector-4] [INFO] [1751224815.412264018] [obstacle_detector]: Cloud Y: min -29527.56, max 3677.37
    // [obstacle_detector-4] [INFO] [1751224815.412265362] [obstacle_detector]: Cloud Z: min 584.65, max 116287.16

    // Define ROI parameters
    const float x_min = 1000.0;
    const float x_max = 7500.0;

    const float y_min = -3000.0;
    const float y_max = 3000.0; 

    const float z_min = 1000.0;
    const float z_max = 20000.0;
    
    
    // Filter in y (side to side)
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*cloud_y);
    // RCLCPP_INFO(this->get_logger(), "After y filter: %zu", cloud_y->points.size());
    
    // Filter in z (height)
    pass.setInputCloud(cloud_y);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_yz(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*cloud_yz);
    // RCLCPP_INFO(this->get_logger(), "After yz filter: %zu", cloud_yz->points.size());
    
    // Filter in x (forward distance)
    pass.setInputCloud(cloud_yz);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min, x_max);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*cloud_filtered);
    // RCLCPP_INFO(this->get_logger(), "After xyz filter: %zu", cloud_filtered->points.size());

    // Find closest point
    float min_dist = std::numeric_limits<float>::max();
    for (const auto& point : cloud_filtered->points) {
      float dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if (dist < min_dist) {
        min_dist = dist;
      }
    }

    std_msgs::msg::Int32 msg_out;
    if (min_dist < std::numeric_limits<float>::max()) {
      // Convert to cm and round
      msg_out.data = static_cast<int>(std::round(min_dist * 100.0));
    } else {
      // No obstacle found, set to -1
      msg_out.data = -1;
    }

    closest_pub_->publish(msg_out);
    
    RCLCPP_INFO(this->get_logger(), "Detection Distance Calculated: %f cm", min_dist);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr closest_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
