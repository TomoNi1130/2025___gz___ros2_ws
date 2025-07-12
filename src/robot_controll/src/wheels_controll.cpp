#include "wheels_controll.hpp"

namespace wheels_controll {

WheelCon::WheelCon(const rclcpp::NodeOptions& options) : rclcpp::Node("wheels_controll_node", options) {
  FR_publisher_ = this->create_publisher<std_msgs::msg::Float64>("FR_v", 10);
  FL_publisher_ = this->create_publisher<std_msgs::msg::Float64>("FL_v", 10);
  BR_publisher_ = this->create_publisher<std_msgs::msg::Float64>("BR_v", 10);
  BL_publisher_ = this->create_publisher<std_msgs::msg::Float64>("BL_v", 10);
  move_subscription_ = this->create_subscription<interface::msg::MoveMsg>("move_par", 10, std::bind(&WheelCon::move_callback, this, std::placeholders::_1));
}

void WheelCon::move_callback(const interface::msg::MoveMsg& msg) {
  std::vector<float> diff_speeds = omuni_controller(msg.direction, msg.velocity, msg.angular_v);

  std_msgs::msg::Float64 msg_FR, msg_FL, msg_BR, msg_BL;
  double wheel_speed_par = 0.3;
  for (int i = 0; i < 4; ++i) {
    diff_speeds[i] = diff_speeds[i] * wheel_speed_par + pre_wheel_speeds[i] * (1.0 - wheel_speed_par);
    pre_wheel_speeds[i] = diff_speeds[i];
  }
  msg_FL.data = diff_speeds[0];
  msg_BL.data = diff_speeds[1];
  msg_BR.data = diff_speeds[2];
  msg_FR.data = diff_speeds[3];

  FR_publisher_->publish(msg_FR);
  FL_publisher_->publish(msg_FL);
  BR_publisher_->publish(msg_BR);
  BL_publisher_->publish(msg_BL);
}

std::vector<float> WheelCon::omuni_controller(const float& direction, const float& velocity, const float& anglar_v) {
  std::vector<float> return_v;
  float tilt = 60;
  double theta[4] = {1.0 / 4.0 * M_PI, 3.0 / 4.0 * M_PI, -3.0 / 4.0 * M_PI, -1.0 / 4.0 * M_PI};
  for (int i = 0; i < 4; ++i) {
    return_v.push_back(velocity * cos(-direction - M_PI / 2.0 + theta[i]) * tilt + tilt / 10.0 * -anglar_v);
  }
  return return_v;
}

}  // namespace wheels_controll

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(wheels_controll::WheelCon)
