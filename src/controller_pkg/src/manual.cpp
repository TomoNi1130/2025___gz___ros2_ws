#include <cmath>
#include <memory>
#include <sensor_msgs/msg/joy.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

class Joy_Sub : public rclcpp::Node {
 public:
  Joy_Sub()
      : Node("Joy_Subscriver") {
    RCLCPP_INFO(this->get_logger(), "setup");
    subscription_ =
        this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Joy_Sub::topic_callback, this, _1));
    FR_publisher_ = this->create_publisher<std_msgs::msg::Float64>("FR_v", 10);
    FL_publisher_ = this->create_publisher<std_msgs::msg::Float64>("FL_v", 10);
    BR_publisher_ = this->create_publisher<std_msgs::msg::Float64>("BR_v", 10);
    BL_publisher_ = this->create_publisher<std_msgs::msg::Float64>("BL_v", 10);
  }

 private:
  double theta[4] = {1.0 / 4.0 * M_PI, 3.0 / 4.0 * M_PI, -3.0 / 4.0 * M_PI, -1.0 / 4.0 * M_PI};

  void topic_callback(sensor_msgs::msg::Joy::UniquePtr msg) {
    std_msgs::msg::Float64 sending_msg[4];

    double vx = msg->axes[0];
    double vy = msg->axes[1];
    double velocity = sqrt(vx * vx + vy * vy);
    double angle = atan2(vy, vx) + M_PI;
    double yaw = msg->axes[3];
    double tilt = 10.0;
    for (int i = 0; i < 4; ++i) {
      sending_msg[i].data = velocity * cos(angle + theta[i]) * tilt - tilt * yaw;
    }
    this->FL_publisher_->publish(sending_msg[0]);
    this->BL_publisher_->publish(sending_msg[1]);
    this->BR_publisher_->publish(sending_msg[2]);
    this->FR_publisher_->publish(sending_msg[3]);
  }
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr FR_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr FL_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr BR_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr BL_publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joy_Sub>());
  rclcpp::shutdown();
  return 0;
}