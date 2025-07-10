#include "wheels_controll.hpp"

namespace wheels_controll {

WheelCon::WheelCon(const rclcpp::NodeOptions& options) : rclcpp::Node("wheels_controll_node", options) {
  FR_publisher_ = this->create_publisher<std_msgs::msg::Float64>("FR_v", 10);
  FL_publisher_ = this->create_publisher<std_msgs::msg::Float64>("FL_v", 10);
  BR_publisher_ = this->create_publisher<std_msgs::msg::Float64>("BR_v", 10);
  BL_publisher_ = this->create_publisher<std_msgs::msg::Float64>("BL_v", 10);
  robot_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>("robot_yaw", 10, std::bind(&WheelCon::set_robot_yaw, this, std::placeholders::_1));
  subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&WheelCon::topic_callback, this, std::placeholders::_1));
}

void WheelCon::topic_callback(const sensor_msgs::msg::Joy& msg) {
  float lx = msg.axes[0];
  float ly = msg.axes[1];
  float rx = -msg.axes[3];
  float direction = atan2(ly, lx) + M_PI * 3.0 / 2.0 + robot_yaw;
  if (direction > M_PI)
    direction -= M_PI * 2.0;
  float velocity = sqrt(lx * lx + ly * ly);
  float angular_v = rx;
  std::vector<float> diff_speeds = omuni_controller(direction, velocity, angular_v);

  std_msgs::msg::Float64 msg_FR, msg_FL, msg_BR, msg_BL;
  msg_FL.data = diff_speeds[0];
  msg_BL.data = diff_speeds[1];
  msg_BR.data = diff_speeds[2];
  msg_FR.data = diff_speeds[3];

  FR_publisher_->publish(msg_FR);
  FL_publisher_->publish(msg_FL);
  BR_publisher_->publish(msg_BR);
  BL_publisher_->publish(msg_BL);
}

void WheelCon::set_robot_yaw(const std_msgs::msg::Float64& msg) {
  robot_yaw = msg.data;
}

std::vector<float> WheelCon::omuni_controller(const float& direction, const float& velocity, const float& angular_v) {
  std::vector<float> return_v;
  float tilt = 60;
  double theta[4] = {1.0 / 4.0 * M_PI, 3.0 / 4.0 * M_PI, -3.0 / 4.0 * M_PI, -1.0 / 4.0 * M_PI};
  for (int i = 0; i < 4; ++i) {
    return_v.push_back(velocity * cos(direction - M_PI / 2.0 + theta[i]) * tilt + tilt / 5.0 * angular_v);
  }
  return return_v;
}

}  // namespace wheels_controll

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(wheels_controll::WheelCon)
