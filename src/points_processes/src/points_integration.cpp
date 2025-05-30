#include "points_integration.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace points_processes {

PointIntegration::PointIntegration(const rclcpp::NodeOptions &options) : Node("points_integration", options) {
  this->declare_parameter<std::vector<std::string>>("scan_topic_names", std::vector<std::string>());
  this->declare_parameter<std::string>("merged_topic_name", "merged_scan");
  this->declare_parameter<std::string>("merged_frame_id", "robot_base");

  scan_topic_names = this->get_parameter("scan_topic_names").as_string_array();
  merged_topic_name = this->get_parameter("merged_topic_name").as_string();
  merged_frame_id = this->get_parameter("merged_frame_id").as_string();

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PointIntegration::send_merged_scan, this));
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  for (const auto &topic : scan_topic_names) {
    auto callback = [this, topic](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { scan_callback(topic, msg); };
    auto subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS{}, std::move(callback), sub_options);
    subscriptions_.push_back(subscription);
  }
}

void PointIntegration::scan_callback(const std::string &topic_name, sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  scans_[topic_name] = msg;
}

void PointIntegration::send_merged_scan() {
}

}  // namespace points_processes

RCLCPP_COMPONENTS_REGISTER_NODE(points_processes::PointIntegration)