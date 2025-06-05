#include "corners_find.hpp"

namespace corners_find {

Line Line::from_points(Eigen::Vector2d &a, Eigen::Vector2d &b) { return Line{a, (b - a).normalized()}; }

double Line::distance_to(Eigen::Vector2d &point) const {
  Eigen::Vector2d v = point - pos;
  double cross = dir.x() * v.y() - dir.y() * v.x();
  return std::abs(cross);
}

double Line::angle_to(const Line &other) const {
  return std::acos(dir.dot(other.dir));
}

std::optional<Eigen::Vector2d> Line::intersection(const Line &other) {
  double cross = dir.x() * other.dir.y() - dir.y() * other.dir.x();
  if (std::abs(cross) < 1e-3) {
    return std::nullopt;
  }
  Eigen::Vector2d diff = other.pos - pos;
  double t = (diff.x() * other.dir.y() - diff.y() * other.dir.x()) / cross;
  return pos + t * dir;
}

LineSegment::LineSegment(Line line) : line(line), low_par(1000.0), high_par(1000.0) {}

CornersFinder::CornersFinder(const rclcpp::NodeOptions &options) : Node("corners_finder", options) {
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