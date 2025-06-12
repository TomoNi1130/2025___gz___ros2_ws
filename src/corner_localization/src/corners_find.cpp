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

LineSegment::LineSegment() : line(Line()), low_par(1000.0), high_par(1000.0) {}

LineSegment::LineSegment(Line line) : line(line), low_par(1000.0), high_par(1000.0) {}

RANSAC::RANSAC(const int iteration, const double threshold, const double gap_threshold, rclcpp::Logger logger) : max_iterations(iteration), distance_threshold(threshold), gap_threshold(gap_threshold), logger(logger) {}

void RANSAC::do_ransac(pcl::PointCloud<pcl::PointXYZ> &target_cloud, std::vector<LineSegment> &return_line_segs, uint8_t max_line_num) {
  std::random_device rd;
  std::mt19937 gen(rd());
  if (target_cloud.points.size() == 0)
    return;
  size_t cloud_size = target_cloud.points.size();
  std::uniform_int_distribution<int> dis(0, cloud_size - 1);
  std::vector<bool> inliers(cloud_size, false);
  for (int line_num = 0; line_num < max_line_num; line_num++) {
    LineSegment return_line_seg;
    int best_inlier_count = 0;
    std::vector<bool> guess_inliers = inliers;
    for (int j = 0; j < max_iterations; j++) {
      int guess_1 = dis(gen);
      int guess_2 = dis(gen);
      while (guess_1 == guess_2 || inliers[guess_1] || inliers[guess_2]) {
        guess_1 = dis(gen);
        guess_2 = dis(gen);
      }
      Eigen::Vector2d p1(target_cloud.points[guess_1].x, target_cloud.points[guess_1].y);
      Eigen::Vector2d p2(target_cloud.points[guess_2].x, target_cloud.points[guess_2].y);
      LineSegment guess_line_seg(Line::from_points(p1, p2));
      threads.push_back(std::thread(
          [this, &guess_line_seg, &best_inlier_count, &return_line_seg, &guess_inliers, &target_cloud, cloud_size]() {
            this->scrutinize_guess_line_seg(guess_line_seg, best_inlier_count, return_line_seg, guess_inliers, target_cloud, cloud_size);
          }));
    }
    for (std::thread &t : threads) {
      t.join();
    }
    threads.clear();
    if (best_inlier_count > cloud_size / 256) {
      for (int i = 0; i < cloud_size; i++) {
        if (guess_inliers[i])
          inliers[i] = true;
      }
      return_line_segs.push_back(return_line_seg);
    }
  }
}

void RANSAC::scrutinize_guess_line_seg(LineSegment &guess_line_seg, int &best_inlier_count, LineSegment &return_line_seg, std::vector<bool> &guess_inliers, const pcl::PointCloud<pcl::PointXYZ> &points_cloud, const size_t cloud_size) {
  std::vector<uint32_t> line_inlier_ids;
  std::vector<bool> line_seg_inliers = guess_inliers;
  uint32_t line_seg_inliers_count = 0;
  for (size_t i = 0; i < cloud_size; i++) {
  }
}

CornersFinder::CornersFinder(const rclcpp::NodeOptions &options) : Node("corners_finder", options) {
  this->declare_parameter<std::string>("merged_topic_name", "merged_scan");
  std::string merged_topic_name = this->get_parameter("merged_topic_name").as_string();

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(merged_topic_name, rclcpp::SensorDataQoS{}, std::bind(&CornersFinder::topic_callback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<interface_pkg::msg::CornersPos>("corners_pos", 10);
}

void CornersFinder::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*msg_ptr, pcl_cloud);
}

}  // namespace corners_find

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(corners_find::CornersFinder)