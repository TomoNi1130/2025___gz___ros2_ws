#include "icp_localization_ver2.hpp"

namespace Localization {

double LineSeg::distance_to(const Eigen::Vector2d &p) {
  Eigen::Vector2d v = p - pos;
  double t = v.dot(dir);
  if (t < 0)
    return (p - pos).norm();
  if (t > length)
    return (p - (pos + dir * length)).norm();
  Eigen::Vector2d normal(-dir.y(), dir.x());
  normal.normalize();
  return normal.dot(p - pos);
}

LineSeg LineSeg::transform(Eigen::Vector3d &par) {
  Eigen::Vector2d T = Eigen::Vector2d(-par.x(), -par.y());
  Eigen::Rotation2Dd R(par.z());
  Eigen::Vector2d new_pos = R * (pos + T);
  Eigen::Vector2d new_dir = R * dir;
  return LineSeg(new_pos, new_dir, length);
}

ICPNode::ICPNode(const rclcpp::NodeOptions &options) : Node("ICP_node", options) {
  this->declare_parameter<std::string>("merged_topic_name", "merged_scan");
  this->declare_parameter<std::string>("merged_frame_id", "robot_base");
  this->declare_parameter<std::string>("map_frame_id", "map");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  merged_topic_name = this->get_parameter("merged_topic_name").as_string();
  merged_frame_id = this->get_parameter("merged_frame_id").as_string();
  map_frame_id = this->get_parameter("map_frame_id").as_string();
  odom_frame_id = this->get_parameter("odom_frame_id").as_string();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(merged_topic_name, rclcpp::SensorDataQoS{}, std::bind(&ICPNode::topic_callback, this, std::placeholders::_1));
  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);
  marker_print = this->create_publisher<visualization_msgs::msg::Marker>("lines_position", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ICPNode::timer_callback, this));
}

void ICPNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
  cloud_header_ = msg_ptr->header;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;  // sensor_msgs::msg::PointCloud2 -> pcl::PointCloud<pcl::PointXYZ>
  pcl::fromROSMsg(*msg_ptr, pcl_cloud);
  robot_cloud.clear();
  robot_cloud.reserve(pcl_cloud.size());
  for (const auto &pt : pcl_cloud.points) {  // pcl::PointCloud<pcl::PointXYZ> -> std::vector<Eigen::Vector2d>
    robot_cloud.emplace_back(pt.x, pt.y);
  }

  robot_pos = Eigen::Vector3d(5.0, -3.0, M_PI / 2.0);
  std::vector<LineSeg> robot_line_segs;
  for (LineSeg &map_line : map_line_segs) {
    LineSeg transformed_line = map_line.transform(robot_pos);
    robot_line_segs.push_back(transformed_line);
  }

  // ICPの実行

  Eigen::Vector3d icp_result = do_icp(robot_cloud, robot_line_segs);

  visualize_line_segments(robot_line_segs);
}

Eigen::Vector3d ICPNode::do_icp(std::vector<Eigen::Vector2d> &point_cloud, std::vector<LineSeg> &line_segments) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> num_dis(-0.5, 0.5);
  std::uniform_real_distribution<double> angle_dis(-M_PI, M_PI);

  Eigen::Vector3d delta_par;
  Eigen::Vector3d guess_par;
  double min_cost = std::numeric_limits<double>::max();
  for (int inter = 0; inter < 80; inter++) {
    Eigen::Vector3d guess_guess_par(robot_pos.x(), robot_pos.y(), robot_pos.z());  // 初期値
    // threads.push_back(std::thread([this, &guess_guess_par, &point_cloud, &line_segments, &guess_par, &min_cost]() { this->ICP(guess_guess_par, point_cloud, line_segments, guess_par, min_cost); }));
  }
  for (std::thread &t : threads) {
    t.join();
  }
  threads.clear();
}

void ICPNode::timer_callback() {
  // 初期位置の設定
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = cloud_header_.stamp;
  t.header.frame_id = map_frame_id;
  t.child_frame_id = odom_frame_id;
  t.transform.translation.x = odom_pos.x();
  t.transform.translation.y = odom_pos.y();
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, odom_pos.z());
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(t);
}

void ICPNode::visualize_line_segments(std::vector<LineSeg> &line_segments) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = cloud_header_.stamp;
  marker.header.frame_id = odom_frame_id;
  marker.header.stamp = this->now();
  marker.ns = "line_segs";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;  // 線の太さ
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  for (const auto &line : line_segments) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = line.pos.x();
    p1.y = line.pos.y();
    p1.z = 0.0;
    p2.x = line.pos.x() + line.length * line.dir.x();
    p2.y = line.pos.y() + line.length * line.dir.y();
    p2.z = 0.0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }
  marker_print->publish(marker);
}

}  // namespace Localization

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(Localization::ICPNode)