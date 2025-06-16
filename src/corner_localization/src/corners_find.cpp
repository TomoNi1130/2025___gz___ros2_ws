#include "corners_find.hpp"

namespace corners_find {

Line Line::from_points(Eigen::Vector2d &a, Eigen::Vector2d &b) { return Line{a, (b - a).normalized()}; }

double Line::distance_to(const Eigen::Vector2d &point) const {
  Eigen::Vector2d v = point - pos;
  double cross = dir.x() * v.y() - dir.y() * v.x();
  return std::abs(cross);
}

double Line::angle_to(const Line &other) const {
  return std::acos(dir.dot(other.dir));
}

double Line::cat_point(Eigen::Vector2d &point_v) {  // この直線とある点の距離が最も近くなる媒介変数を求める
  Eigen::Vector2d v = point_v - pos;
  return v.dot(dir) / dir.squaredNorm();
}

LineSegment::LineSegment() : line(Line()), low_par(-10.0), high_par(10.0) {}

LineSegment::LineSegment(Line &line) : line(line), low_par(-10.0), high_par(10.0) {}
void LineSegment::set_start_point(Eigen::Vector2d &start_point) {
  Eigen::Vector2d v = start_point - line.pos;
  start_par = v.dot(line.dir) / line.dir.squaredNorm();
}
void LineSegment::set_end_point(Eigen::Vector2d &end_point) {
  Eigen::Vector2d v = end_point - line.pos;
  end_par = v.dot(line.dir) / line.dir.squaredNorm();
}

RANSAC::do_ransac(pcl::PointCloud<pcl::PointXYZ> &target_cloud, std::vector<LineSegment> &return_line_segs, uint8_t max_line_num) {
  std::vector<std::pair<Eigen::Vector2d, bool>> points_cloud_v;  // 位置ベクトルとinlierかどうか
  std::random_device rd;
  std::mt19937 gen(rd());
  if (target_cloud.points.size() == 0)
    return;
  for (const auto &pt : target_cloud.points)  // 点群をベクトルに変換
    points_cloud_v.emplace_back(Eigen::Vector2d(pt.x, pt.y), false);
  for (int i = 0; i < max_line_num; i++) {
    LineSegment best_line;  // 追加する線分
    int best_inlier_count = 0;
    for (int j = 0; j < max_iterations; j++) {  // inlier以外の点２つから直線を見つける
      std::uniform_int_distribution<int> dis(0, points_cloud.size() - 1);
      int guess_1 = dis(gen);
      int guess_2 = dis(gen);
      while (guess_1 == guess_2 || points_cloud_v[guess_1].second) {
        guess_1 = dis(gen);
      }
      std::vector<std::pair<int, double>> guess_line_inlier_IDs;  // 直線のinlier用
      Line guess_line = Line::from_points(points_cloud_v[guess_1].first, points_cloud_v[guess_2].first);
      for (size_t k = 0; k < points_cloud_v.size(); k++) {
        if (guess_line.distance_to(points_cloud_v[k].first) < distance_threshold) {
          guess_line_inlier_IDs.emplace_back(int(k), guess_line.cat_point(points_cloud_v[k].first));
        }
      }
      int guess_inlier_count = 0;  // 線分の近くの点の数
      if (guess_inlier_count > best_inlier_count) {
        best_line = guess_line;
        best_inlier_count = guess_inlier_count;
      }
    }
    LineSegment return_line_seg(best_line);
    return_line_segs.push_back(return_line_seg);
  }
}

CornersFinder::CornersFinder(const rclcpp::NodeOptions &options) : Node("corners_finder", options) {
  this->declare_parameter<std::string>("merged_topic_name", "merged_scan");
  this->declare_parameter<std::string>("merged_frame_id", "robot_base");
  merged_topic_name = this->get_parameter("merged_topic_name").as_string();
  merged_frame_id = this->get_parameter("merged_frame_id").as_string();

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(merged_topic_name, rclcpp::SensorDataQoS{}, std::bind(&CornersFinder::topic_callback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<interface_pkg::msg::CornersPos>("corners_pos", 10);
  marker_print = this->create_publisher<visualization_msgs::msg::Marker>("lines_position", 10);
}

void CornersFinder::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*msg_ptr, pcl_cloud);
  std::vector<LineSegment> line_segs;
  ransac.do_ransac(pcl_cloud, line_segs, 1);
  if (line_segs.size() != 0) {
    visualize_line_segments(line_segs);
  }
}

void visualize_line_segments(line_seg line_seg) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "test";
  marker.ns = "detected_lines";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;
  marker.color.g = 1.0;
  marker.color.a = 1.0;
  geometry_msgs::msg::Point p1, p2;
  p1.x = (line_seg.pos + (line_seg.dir * line_seg.low_per)).x();
  p1.y = (line_seg.pos + (line_seg.dir * line_seg.low_per)).y();
  p2.x = (line_seg.pos + (line_seg.dir * line_seg.high_per)).x();
  p2.y = (line_seg.pos + (line_seg.dir * line_seg.high_per)).y();
  RCLCPP_INFO(this->get_logger(), "%f %f", line_seg.low_per, line_seg.high_per);
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker_print->publish(marker);
}

}  // namespace corners_find

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(corners_find::CornersFinder)