#include "static_tf.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace UC {

StaticTF::StaticTF(const rclcpp::NodeOptions &options) : rclcpp::Node("static_tf_node", options) {
  this->declare_parameter<std::string>("robot_frame_id", "robot_base");
  this->declare_parameter<std::string>("left_lidar_frame_id", "left_lidar");
  this->declare_parameter<std::string>("right_lidar_frame_id", "right_lidar");

  robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();
  left_lidar_frame_id_ = this->get_parameter("left_lidar_frame_id").as_string();
  right_lidar_frame_id_ = this->get_parameter("right_lidar_frame_id").as_string();

  broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&StaticTF::msg_send, this));
}

void StaticTF::msg_send() {
  send_tf(left_lidar_frame_id_, robot_frame_id_, 0.0, 0.4, 0.0, 0.0, 0.0, 0);
  send_tf(right_lidar_frame_id_, robot_frame_id_, 0.0, -0.4, 0.0, 0.0, 0.0, 3.14159);
}

void StaticTF::send_tf(const std::string &child_id, const std::string &parent_id, double x, double y, double z, double roll, double pitch, double yaw) {
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = this->get_clock()->now();
  transformStamped.header.frame_id = parent_id;  // 親フレーム
  transformStamped.child_frame_id = child_id;

  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  broadcaster_->sendTransform(transformStamped);
}

}  // namespace UC

RCLCPP_COMPONENTS_REGISTER_NODE(UC::StaticTF)