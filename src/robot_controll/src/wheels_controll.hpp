#ifndef WHEELS_CONTROLL_HPP
#define WHEELS_CONTROLL_HPP

#include "interface/msg/move_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

namespace wheels_controll {

class WheelCon : public rclcpp::Node {
 public:
  WheelCon(const rclcpp::NodeOptions& options);

 private:
  void move_callback(const interface::msg::MoveMsg& msg);
  std::vector<float> omuni_controller(const float& direction, const float& velocity, const float& anglar_V);

  double robot_yaw = 0;

  double pre_wheel_speeds[4] = {0, 0, 0, 0};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr FR_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr FL_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr BR_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr BL_publisher_;
  rclcpp::Subscription<interface::msg::MoveMsg>::SharedPtr move_subscription_;
};

}  // namespace wheels_controll

#endif