#include "corners_find.hpp"

namespace corners_find {

CornersFinder::CornersFinder(const rclcpp::NodeOptions& options) : Node("corners_finder", options) {
  this->declare_parameter<std::string>("merged_topic_name", "merged_scan");
  std::string merged_topic_name = this->get_parameter("merged_topic_name").as_string();

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(merged_topic_name, rclcpp::SensorDataQoS{}, std::bind(&CornersFinder::topic_callback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<interface_pkg::msg::CornersPos>("corners_pos", 10);
}

void CornersFinder::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
}

}  // namespace corners_find

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(corners_find::CornersFinder)