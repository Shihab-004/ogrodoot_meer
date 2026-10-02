#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

/**
 * @file obstacle_detector.cpp
 * @brief Boilerplate obstacle detection node for the MEER rover.
 *
 * TODO:
 *   - Subscribe to LiDAR scan or PointCloud2 from depth camera.
 *   - Implement min-range detection zone (front arc ±30°).
 *   - Publish /obstacle_detected [std_msgs/Bool] at 10 Hz.
 *   - Publish /nearest_obstacle [geometry_msgs/PointStamped].
 */

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
  : Node("obstacle_detector")
  {
    // Parameters
    this->declare_parameter("min_obstacle_dist", 0.5);   // metres
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("frame_id", "base_link");

    min_dist_ = this->get_parameter("min_obstacle_dist").as_double();
    scan_topic_ = this->get_parameter("scan_topic").as_string();

    // Publishers
    pub_detected_ = this->create_publisher<std_msgs::msg::Bool>(
      "/obstacle_detected", rclcpp::QoS(10));

    pub_nearest_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/nearest_obstacle", rclcpp::QoS(10));

    // Subscriber — LiDAR scan
    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ObstacleDetector::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "ObstacleDetector started. Listening on '%s', min_dist=%.2f m",
      scan_topic_.c_str(), min_dist_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // TODO: Implement real obstacle logic here
    bool obstacle_found = false;
    float min_range = std::numeric_limits<float>::max();

    for (float r : msg->ranges) {
      if (std::isfinite(r) && r < min_dist_) {
        obstacle_found = true;
      }
      if (std::isfinite(r) && r < min_range) {
        min_range = r;
      }
    }

    auto detected_msg = std_msgs::msg::Bool();
    detected_msg.data = obstacle_found;
    pub_detected_->publish(detected_msg);

    if (min_range < std::numeric_limits<float>::max()) {
      auto pt_msg = geometry_msgs::msg::PointStamped();
      pt_msg.header = msg->header;
      pt_msg.point.x = static_cast<double>(min_range);
      pt_msg.point.y = 0.0;
      pt_msg.point.z = 0.0;
      pub_nearest_->publish(pt_msg);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_detected_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_nearest_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

  double min_dist_;
  std::string scan_topic_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
