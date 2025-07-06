

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace Localization {

class ICPNode : public rclcpp::Node {
 public:
  ICPNode(const rclcpp::NodeOptions &options);

 private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);
  void timer_callback();
  Eigen::Vector3d do_icp(pcl::PointCloud<pcl::PointXYZ> &point_cloud);
  void update_robot_pose(Eigen::Vector2d &robot_pos, double &robot_yaw);

  std::vector<Eigen::Vector2d> robot_cloud;  // ロボから見た点群

  Eigen::Vector3d robot_pos;  // ロボットの姿勢//{x,y,theta}

  std::string merged_topic_name;
  std::string merged_frame_id;
  std::string map_frame_id;
  std::string odom_frame_id;

  std_msgs::msg::Header::stamp timestamp_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace Localization