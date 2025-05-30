#include <string>
#include <vector>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// いくつかのlidarのtopic名をもとにそれらの点群を統合、加工した点群を返す

namespace points_processes {

class PointIntegration : public rclcpp::Node {
 public:
  PointIntegration(const rclcpp::NodeOptions &options);

 private:
  void scan_callback(const std::string &topic_name, sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void send_merged_scan();

  std::vector<std::string> scan_topic_names;
  std::string merged_topic_name;
  std::string merged_frame_id;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subscriptions_;
  std::unordered_map<std::string, sensor_msgs::msg::LaserScan::ConstSharedPtr> scans_;  // LaserScanの集合体
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace points_processes
