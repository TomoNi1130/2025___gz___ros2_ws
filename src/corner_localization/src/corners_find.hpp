#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "interface_pkg/msg/corners_pos.hpp"

namespace corners_find {
class CornersFinder : public rclcpp::Node {
 public:
  CornersFinder(const rclcpp::NodeOptions &options);

 private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<interface_pkg::msg::CornersPos>::SharedPtr publisher_;
};
}  // namespace corners_find