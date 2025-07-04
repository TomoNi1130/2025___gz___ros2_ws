#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

// いくつかのlidarのtopic名をもとにそれらの点群を統合、加工した点群を返す

namespace points_processes {

class PointIntegration : public rclcpp::Node {
 public:
  PointIntegration(const rclcpp::NodeOptions &options);

 private:
  void scan_callback(const std::string &topic_name, sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void send_merged_scan();
  pcl::PointCloud<pcl::PointXYZ> resampler(const pcl::PointCloud<pcl::PointXYZ> &target_points);
  bool point_inserter(const pcl::PointXYZ &point, const pcl::PointXYZ &pre_point, pcl::PointXYZ &new_point, bool &inserted);

  double equalization_point_dis = 0.15;
  double point_dis_threshold = 0.30;
  double total_dis;

  std::vector<std::string> scan_topic_names;
  std::string merged_topic_name;
  std::string merged_frame_id;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subscriptions_;
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> scans_;  // LaserScanの集合体
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_cloud_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  laser_geometry::LaserProjection projector_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace points_processes
