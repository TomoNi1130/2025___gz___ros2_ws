#include "set_goal.hpp"

namespace set_goal {

SetGoal::SetGoal(const rclcpp::NodeOptions& options) : rclcpp::Node("set_goal_node", options) {
  subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&SetGoal::topic_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SetGoal::timer_callback, this));
}

void SetGoal::topic_callback(const sensor_msgs::msg::Joy& msg) {
  if (msg.buttons[0])
    auto_mode = false;
  else if (msg.buttons[1])
    auto_mode = true;
}

void SetGoal::timer_callback() {
}

}  // namespace set_goal

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(set_goal::SetGoal)