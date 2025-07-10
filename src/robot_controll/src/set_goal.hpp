#ifndef SET_GOAL_HPP
#define SET_GOAL_HPP

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace set_goal {

class SetGoal : public rclcpp::Node {
 public:
  SetGoal(const rclcpp::NodeOptions& options);

 private:
  void topic_callback(const sensor_msgs::msg::Joy& msg);
  void timer_callback();

  bool auto_mode = false;

  double robot_yaw = 0;
  Eigen::Vector2d robot_pos;

  Eigen::Vector2d target_dir, target_speed;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace set_goal

#endif