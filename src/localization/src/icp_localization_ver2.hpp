

#include <Eigen/Dense>
#include <mutex>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace Localization {

struct LineSeg {
  Eigen::Vector2d pos, dir;
  double length;
  LineSeg(const Eigen::Vector2d &p, const Eigen::Vector2d &d, double l)
      : pos(p), dir(d), length(l) {}
  double distance_to(const Eigen::Vector2d &p);
  LineSeg transform(const Eigen::Vector3d &par);
};

class ICPNode : public rclcpp::Node {
 public:
  ICPNode(const rclcpp::NodeOptions &options);

 private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);
  void timer_callback();
  void remove_outliers(std::vector<Eigen::Vector2d> &target_cloud, const double threshold, const int max_line_num, const int max_iterations);
  Eigen::Vector3d do_icp(std::vector<Eigen::Vector2d> &point_cloud, std::vector<LineSeg> &line_segments);
  void ICP(Eigen::Vector3d default_par_init, std::vector<Eigen::Vector2d> &real_points, std::vector<LineSeg> &line_segs, Eigen::Vector3d &best_guess_par, double &global_min_cost);
  void transePoints(std::vector<Eigen::Vector2d> &points, Eigen::Vector3d par);
  void update_robot_pose(Eigen::Vector2d &robot_pos, double &robot_yaw);
  sensor_msgs::msg::PointCloud2 Eigen_to_cloud(const std::vector<Eigen::Vector2d> &points, const std_msgs::msg::Header &header);

  void visualize_line_segments(std::vector<LineSeg> &line_segments);

  std::vector<LineSeg> map_line_segs = {
      LineSeg(Eigen::Vector2d(-0.500, 0.0 - 4.75), Eigen::Vector2d(1, 0), 6.800),
      LineSeg(Eigen::Vector2d(-0.500, 0.0 - 4.75), Eigen::Vector2d(0, 1), 5.250),
      LineSeg(Eigen::Vector2d(-0.500, 5.250 - 4.75), Eigen::Vector2d(1, 0), 10.500),
      LineSeg(Eigen::Vector2d(10.000, 5.250 - 4.75), Eigen::Vector2d(0, -1), 5.025),
      LineSeg(Eigen::Vector2d(6.300, 0.225 - 4.75), Eigen::Vector2d(1, 0), 3.700),
      LineSeg(Eigen::Vector2d(6.300, 0.225 - 4.75), Eigen::Vector2d(0, -1), 0.225),
  };  // 地図上の線分

  std::vector<Eigen::Vector2d> robot_cloud;  // ロボから見た点群
  Eigen::Vector3d robot_pos;                 // ロボットの姿勢//{x,y,theta}
  Eigen::Vector3d new_robot_pos;
  Eigen::Vector3d odom_pos = {0.0, 4.75, 0.0};

  Eigen::Vector2d pre_pos_error;

  std::mutex mtx;
  std::vector<std::thread> threads;

  std::string merged_topic_name;
  std::string merged_frame_id;
  std::string map_frame_id;
  std::string odom_frame_id;

  std_msgs::msg::Header cloud_header_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr robot_yaw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clean_cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_print;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace Localization