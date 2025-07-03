#include "icp_localization.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace Localization {

Eigen::Matrix2d getR(double theta) {
  Eigen::Matrix2d R;
  R << cos(theta), -sin(theta),
      sin(theta), cos(theta);
  return R;
}

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
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  merged_topic_name = this->get_parameter("merged_topic_name").as_string();
  merged_frame_id = this->get_parameter("merged_frame_id").as_string();
  map_frame_id = this->get_parameter("map_frame_id").as_string();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(merged_topic_name, rclcpp::SensorDataQoS{}, std::bind(&LocalizationNode::topic_callback, this, std::placeholders::_1));
  marker_print = this->create_publisher<visualization_msgs::msg::Marker>("lines_position", 10);
  points_view = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_look_points", 10);
}

void LocalizationNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
  pcl::PointCloud<pcl::PointXYZ> map_pcl_cloud;
  sensor_msgs::msg::PointCloud2 cloud_msg = *msg_ptr;

  try {
    sensor_msgs::msg::PointCloud2 cloud_transformed;
    tf_buffer_->transform(cloud_msg, cloud_transformed, map_frame_id, tf2::durationFromSec(0.1));
    pcl::fromROSMsg(cloud_transformed, map_pcl_cloud);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(rclcpp::get_logger("Transform"), "Transform failed: %s", ex.what());
  }

  std::vector<Eigen::Vector2d> map_points;
  for (const auto &point : map_pcl_cloud.points) {
    map_points.emplace_back(point.x, point.y);
  }

  // pcl::PointCloud<pcl::PointXYZ> clean_cloud;
  // remove_outlier(robot_pcl_cloud, clean_cloud, 0.015);  // 外れ値を除外

  // icp開始
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> num_dis(-3.0, 3.0);

  // Eigen::Vector3d guess_par = {0, 0, 0};
  // double min_cost = std::numeric_limits<double>::max();
  // for (int inter = 0; inter < 500; inter++) {
  //   Eigen::Vector3d guess_guess_par(num_dis(gen), num_dis(gen), num_dis(gen) * M_PI);  // 初期値
  //   threads.push_back(std::thread([this, &guess_guess_par, &real_points, &guess_par, &min_cost]() { this->ICP(guess_guess_par, real_points, guess_par, min_cost); }));
  // }
  // for (std::thread &t : threads) {
  //   t.join();
  // }
  // threads.clear();

  // icp終了

  update_robot_pose(robot_pos, robot_yaw);  // ロボットの位置と姿勢を更新

  // // map座標系で点群を表示
  auto ros2_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(map_pcl_cloud, *ros2_points);
  ros2_points->header.frame_id = map_frame_id;
  points_view->publish(*ros2_points);
  visualize_lines(map_lines);
}

void LocalizationNode::transePoints(std::vector<Eigen::Vector2d> &points, Eigen::Vector3d par) {
  Eigen::Vector2d tra = {par(0), par(1)};
  Eigen::Matrix2d R = getR(par(2));
  for (size_t i = 0; i < points.size(); i++) {
    points[i] = R * (points[i] + tra);  // 移動してから回転
  }
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
        if (!cloud_v[j].second && abs(guess_line.distance_to(cloud_v[j].first)) < threshold) {
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
    if (best_inlier_num < (int)cloud_v.size() / 8)  // ここの数字は適当
      be_line = false;
  }
}

void LocalizationNode::update_robot_pose(Eigen::Vector2d &robot_pos, double &robot_yaw) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = map_frame_id;
  t.child_frame_id = "odom";
  t.transform.translation.x = robot_pos.x();
  t.transform.translation.y = robot_pos.y();
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, robot_yaw);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(t);
}

void LocalizationNode::visualize_lines(const std::vector<Line> &lines) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_id;
  marker.header.stamp = this->now();
  marker.ns = "lines";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;  // 線の太さ
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  for (const auto &line : lines) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = line.pos.x() - 100.0 * line.dir.x();
    p1.y = line.pos.y() - 100.0 * line.dir.y();
    p1.z = 0.0;
    p2.x = line.pos.x() + 100.0 * line.dir.x();
    p2.y = line.pos.y() + 100.0 * line.dir.y();
    p2.z = 0.0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  marker_print->publish(marker);
}

}  // namespace Localization

RCLCPP_COMPONENTS_REGISTER_NODE(Localization::LocalizationNode)