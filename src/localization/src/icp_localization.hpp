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
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>

#include "tf2_ros/transform_broadcaster.h"

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
  tf2::Transform do_icp(pcl::PointCloud<pcl::PointXYZ> &target_cloud, std::vector<Line> &planes);                                  // point to planesを使用
  void update_robot_pose(Eigen::Vector2d &robot_pos, double &robot_yaw);
  void transePoints(std::vector<Eigen::Vector2d> &points, Eigen::Vector3d par);

  void visualize_lines(const std::vector<Line> &lines);  // 可視化用の関数

  std::vector<Line> map_lines = {Line(Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0)),
                                 Line(Eigen::Vector2d(-0.5, 0), Eigen::Vector2d(0, 1)),
                                 Line(Eigen::Vector2d(0.0, 5.250), Eigen::Vector2d(1, 0)),
                                 Line(Eigen::Vector2d(0.0, -5.250), Eigen::Vector2d(1, 0)),
                                 Line(Eigen::Vector2d(10.0, 0.0), Eigen::Vector2d(0, 1))};  // 地図上の直線

  Eigen::Vector2d robot_pos = Eigen::Vector2d(0.0, 4.75);  // ロボットの位置
  double robot_yaw = 0.0;                                  // ロボットの位置と姿勢

  std::string merged_topic_name;
  std::string merged_frame_id;
  std::string map_frame_id;
  int ITERATION_NUM = 100;

  std::mutex mtx;
  std::vector<std::thread> threads;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_print;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_view;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace Localization