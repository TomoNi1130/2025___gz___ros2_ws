#include "icp_localization.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace Localization {

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions& options) : Node("points_integration", options) {
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
  // 点群の座標系を marged_frame_id から map_frame_id への変換
  try {
    transform_stamped = tf_buffer_->lookupTransform(map_frame_id, merged_frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    return;
  }
  Eigen::Affine3d affine = tf2::transformToEigen(transform_stamped.transform);
  Eigen::Matrix4f transform = affine.matrix().cast<float>();
  pcl::transformPointCloud(robot_pcl_cloud, map_pcl_cloud, transform, false);

  pcl::PointCloud<pcl::PointXYZ> clean_cloud;  // 外れ値を除外した点群
  remove_outlier(map_pcl_cloud, clean_cloud, 0.01);

  // map座標系で点群を表示
  auto ros2_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(map_pcl_cloud, *ros2_points);
  ros2_points->header.frame_id = "map";
  points_view->publish(*ros2_points);
}

void LocalizationNode::remove_outlier(pcl::PointCloud<pcl::PointXYZ>& target_cloud, pcl::PointCloud<pcl::PointXYZ>& cloud_out, double threshold) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dis(0, target_cloud.size() - 1);
  std::vector<std::pair<Eigen::Vector2d, bool>> cloud_v;
  for (size_t i = 0; i < target_cloud.points.size(); i++) {
    cloud_v.emplace_back(Eigen::Vector2d(target_cloud.points[i].x, target_cloud.points[i].y), false);
  }
  bool be_line = true;
  int iterations_num = 100;
  while (be_line) {
    for (int i = 0; i < iterations_num; i++) {
      int guess_1 = dis(gen);
      int guess_2 = dis(gen);
      while (guess_1 == guess_2 || cloud_v[guess_1].second)
        guess_1 = dis(gen);
    }
  }
  RCLCPP_INFO(this->get_logger(), "points size:%zu", cloud_v.size());
}

}  // namespace Localization

RCLCPP_COMPONENTS_REGISTER_NODE(Localization::LocalizationNode)