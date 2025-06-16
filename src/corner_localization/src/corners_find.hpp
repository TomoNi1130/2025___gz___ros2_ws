#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <mutex>
#include <optional>
#include <pcl/impl/point_types.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <thread>

#include "interface_pkg/msg/corners_pos.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace corners_find {
struct SimplePoint {
  double x, y;
};

struct Line {
  static Line from_points(Eigen::Vector2d &a, Eigen::Vector2d &b);
  double distance_to(const Eigen::Vector2d &point) const;
  double angle_to(const Line &other) const;
  std::optional<Eigen::Vector2d> intersection(const Line &other);
  double cat_point(Eigen::Vector2d &point_v);
  Eigen::Vector2d dir, pos;
};

struct LineSegment {
  LineSegment();
  LineSegment(Line &line);
  void set_start_point(Eigen::Vector2d &start_point);
  void set_end_point(Eigen::Vector2d &end_point);
  Line line;
  double end_par, start_par;  // 線分にするための最小、最大媒介変数
};

class RANSAC {
 public:
  RANSAC(const int iteration, const double threshold, const double gap_threshold, rclcpp::Logger logger);
  void do_ransac(pcl::PointCloud<pcl::PointXYZ> &target_cloud, std::vector<LineSegment> &return_line_segs, uint8_t max_line_num);

 private:
  const int max_iterations;
  const double distance_threshold = 0.1;
  const double gap_threshold = 1.0;

  std::vector<std::thread> threads;
  rclcpp::Logger logger;

  std::mutex mtx;
};

class CornersFinder : public rclcpp::Node {
 public:
  CornersFinder(const rclcpp::NodeOptions &options);

 private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);

  void visualize_line_segments(std::vector<LineSegment> &visualize_line_seg);

  std::string merged_topic_name;
  std::string merged_frame_id;

  RANSAC ransac{150, 0.1, 1.0, this->get_logger()};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<interface_pkg::msg::CornersPos>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_print;
};

std::optional<Eigen::Vector2d> Line::intersection(const Line &other) {
  double cross = dir.x() * other.dir.y() - dir.y() * other.dir.x();
  if (std::abs(cross) < 1e-3) {
    return std::nullopt;
  }
  Eigen::Vector2d diff = other.pos - pos;
  double t = (diff.x() * other.dir.y() - diff.y() * other.dir.x()) / cross;
  return pos + t * dir;
}

}  // namespace corners_find