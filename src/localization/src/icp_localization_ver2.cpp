#include "icp_localization_ver2.hpp"

namespace Localization {

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

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ICPNode::timer_callback, this));
}

void ICPNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
  timestamp_ = msg_ptr->header.stamp;
  // map座標変換
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;  // sensor_msgs::msg::PointCloud2 -> pcl::PointCloud<pcl::PointXYZ>
  pcl::fromROSMsg(*msg_ptr, pcl_cloud);
  robot_cloud.clear();
  robot_cloud.reserve(pcl_cloud.size());
  for (const auto &pt : pcl_cloud.points) {  // pcl::PointCloud<pcl::PointXYZ> -> std::vector<Eigen::Vector2d>
    robot_cloud.emplace_back(pt.x, pt.y);
  }
}

Eigen::Vector3d ICPNode::do_icp(pcl::PointCloud<pcl::PointXYZ> &point_cloud) {
}

void ICPNode::timer_callback() {
  robot_pos = {0.0, 4.75, 0.0};

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = timestamp_;
  t.header.frame_id = map_frame_id;
  t.child_frame_id = odom_frame_id;
  t.transform.translation.x = robot_pos.x();
  t.transform.translation.y = robot_pos.y();
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, robot_pos.z());
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(t);
}

}  // namespace Localization

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(Localization::ICPNode)