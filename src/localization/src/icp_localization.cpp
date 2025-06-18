#include "icp_localization.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace Localization {

Line::Line() : pos(Eigen::Vector2d(0, 0)), dir(Eigen::Vector2d(1, 0)) {}
Line::Line(const Eigen::Vector2d &position, const Eigen::Vector2d &direction) : pos(position), dir(direction) {}

Line Line::from_points(Eigen::Vector2d &a, Eigen::Vector2d &b) { return Line{a, (b - a).normalized()}; }
double Line::distance_to(Eigen::Vector2d &point) const {
  Eigen::Vector2d v = point - pos;
  double cross = dir.x() * v.y() - dir.y() * v.x();
  return std::abs(cross);
}

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions &options) : Node("points_integration", options) {
  this->declare_parameter<std::string>("merged_topic_name", "merged_scan");
  this->declare_parameter<std::string>("merged_frame_id", "robot_base");
  this->declare_parameter<std::string>("map_frame_id", "map");
  merged_topic_name = this->get_parameter("merged_topic_name").as_string();
  merged_frame_id = this->get_parameter("merged_frame_id").as_string();
  map_frame_id = this->get_parameter("map_frame_id").as_string();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(merged_topic_name, rclcpp::SensorDataQoS{}, std::bind(&LocalizationNode::topic_callback, this, std::placeholders::_1));
  marker_print = this->create_publisher<visualization_msgs::msg::Marker>("lines_position", 10);
  points_view = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_look_points", 10);
}

void LocalizationNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
  pcl::PointCloud<pcl::PointXYZ> robot_pcl_cloud, map_pcl_cloud;
  pcl::fromROSMsg(*msg_ptr, robot_pcl_cloud);
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(map_frame_id, merged_frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    return;
  }
  Eigen::Affine3d affine = tf2::transformToEigen(transform_stamped.transform);  // 点群の座標系を marged_frame_id から map_frame_id への変換
  Eigen::Matrix4f transform = affine.matrix().cast<float>();
  pcl::transformPointCloud(robot_pcl_cloud, map_pcl_cloud, transform, false);

  pcl::PointCloud<pcl::PointXYZ> clean_cloud;
  remove_outlier(map_pcl_cloud, clean_cloud, 0.01);  // 外れ値を除外

  // icp開始

  // map座標系で点群を表示
  auto ros2_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(clean_cloud, *ros2_points);
  ros2_points->header.frame_id = "map";
  points_view->publish(*ros2_points);
}

void LocalizationNode::remove_outlier(pcl::PointCloud<pcl::PointXYZ> &target_cloud, pcl::PointCloud<pcl::PointXYZ> &cloud_out, double threshold) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dis(0, target_cloud.size() - 1);
  std::vector<std::pair<Eigen::Vector2d, bool>> cloud_v;
  for (size_t i = 0; i < target_cloud.points.size(); i++) {
    cloud_v.emplace_back(Eigen::Vector2d(target_cloud.points[i].x, target_cloud.points[i].y), false);
  }
  bool be_line = true;
  int iterations_num = ITERATION_NUM;
  while (be_line) {
    int best_inlier_num = 0;
    std::vector<int> best_inlier_IDs;
    for (int i = 0; i < iterations_num; i++) {
      int guess_1 = dis(gen);
      int guess_2 = dis(gen);
      while (guess_1 == guess_2 || cloud_v[guess_1].second)
        guess_1 = dis(gen);
      Line guess_line = Line::from_points(cloud_v[guess_1].first, cloud_v[guess_2].first);
      int inlier_num = 0;
      std::vector<int> inlier_IDs;
      for (size_t j = 0; j < cloud_v.size(); j++) {
        if (!cloud_v[j].second && guess_line.distance_to(cloud_v[j].first) < threshold) {
          inlier_num++;
          inlier_IDs.push_back(j);
        }
      }
      if (best_inlier_num < inlier_num) {
        best_inlier_num = inlier_num;
        best_inlier_IDs = inlier_IDs;
      }
    }
    for (int id : best_inlier_IDs) {
      cloud_out.points.push_back(target_cloud.points[id]);
      cloud_v[id].second = true;
    }
    if (best_inlier_num < cloud_v.size() / 8)  // ここの数字は適当
      be_line = false;
  }
}

tf2::Transform do_icp(pcl::PointCloud<pcl::PointXYZ> &target_cloud, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &planes) {
}

}  // namespace Localization

RCLCPP_COMPONENTS_REGISTER_NODE(Localization::LocalizationNode)