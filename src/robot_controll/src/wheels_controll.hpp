#ifndef WHEELS_CONTROLL_HPP
#define WHEELS_CONTROLL_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

namespace wheels_controll {

class WheelCon : public rclcpp::Node {
 public:
  WheelCon(const rclcpp::NodeOptions& options);

 private:
  void topic_callback(const sensor_msgs::msg::Joy& msg);
  std::vector<float> omuni_controller(const float& direction, const float& velocity, const float& angular_V);

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr FR_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr FL_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr BR_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr BL_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

}  // namespace wheels_controll

#endif