#ifndef SET_GOAL_HPP
#define SET_GOAL_HPP

#include <Eigen/Dense>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "interface/msg/move_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace set_goal {

class SetGoal : public rclcpp::Node {
 public:
  SetGoal(const rclcpp::NodeOptions& options);

 private:
  void topic_callback(const sensor_msgs::msg::Joy& msg);
  void dis_callback(const std_msgs::msg::Float64& msg);
  void dir_callback(const geometry_msgs::msg::Vector3& msg);
  void goal_pos_callback(const geometry_msgs::msg::PoseStamped& msg);
  void send_goal();
  void get_tf();

  struct PDgain {
    double P, D;
  };

  PDgain ang_gain = {0.5, 0.12};
  PDgain vel_gain = {0.65, 0.15};
  PDgain dir_gain = {0.30, 0.20};

  bool auto_mode = false;

  double robot_yaw = 0;
  double goal_yaw = -M_PI * 0.5;
  Eigen::Vector2d map_to_goal = {8.5, 0.7};
  Eigen::Vector2d map_to_robot;

  double pre_goal_dis = 0;
  double pre_x_dis = 0;
  double pre_y_dis = 0;
  double pre_angle_error = 0;

  double target_dir;
  double target_roat;
  double target_power;

  Eigen::Vector2d nearest_wall_norm;
  double nearest_wall_distance;

  std::string robot_frame_id_;
  std::string odom_frame_id_;
  std::string map_frame_id_;

  geometry_msgs::msg::TransformStamped robot_stramp;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pos_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wall_dis_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wall_norm_pub_;
  rclcpp::Publisher<interface::msg::MoveMsg>::SharedPtr move_nums_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_print;
  rclcpp::TimerBase::SharedPtr timer_1;
  rclcpp::TimerBase::SharedPtr timer_2;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace set_goal

#endif