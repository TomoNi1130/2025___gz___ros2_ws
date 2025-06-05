#include <Eigen/Dense>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "interface_pkg/msg/corners_pos.hpp"

namespace corners_find {
struct SimplePoint {
  double x, y;
};

struct Line {
  static Line from_points(Eigen::Vector2d &a, Eigen::Vector2d &b);
  double distance_to(Eigen::Vector2d &point) const;
  double angle_to(const Line &other) const;
  std::optional<Eigen::Vector2d> intersection(const Line &other);
  Eigen::Vector2d dir, pos;
};

struct LineSegment {
  LineSegment(Line line);
  Line line;
  int low_par, high_par;  // 線分にするための最小、最大媒介変数
};

class CornersFinder : public rclcpp::Node {
 public:
  CornersFinder(const rclcpp::NodeOptions &options);

 private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<interface_pkg::msg::CornersPos>::SharedPtr publisher_;
};
}  // namespace corners_find