#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <mutex>
#include <optional>
#include <pcl/impl/point_types.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>

// 統合された点群を地図座標系になおし、そこから自己位置推定

namespace Localization {

struct Line {
  Eigen::Vector2d pos, dir;
  Line();
  Line(const Eigen::Vector2d &position, const Eigen::Vector2d &direction);

  static Line from_points(Eigen::Vector2d &a, Eigen::Vector2d &b);
  double distance_to(Eigen::Vector2d &point) const;
  std::optional<Eigen::Vector2d> intersection(const Line &other) const;
};

class LocalizationNode : public rclcpp::Node {
 public:
  LocalizationNode(const rclcpp::NodeOptions &options);

 private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);
  void remove_outlier(pcl::PointCloud<pcl::PointXYZ> &target_cloud, pcl::PointCloud<pcl::PointXYZ> &cloud_out, double threshold);  // ransacを使って直線状になっている点のみにする

  Eigen::Vector2d robot_pos, robot_dir;

  std::string merged_topic_name;
  std::string merged_frame_id;
  std::string map_frame_id;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_print;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_view;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace Localization