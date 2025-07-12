#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Vector3.hpp>

namespace UC {
class StaticTF : public rclcpp::Node {
 public:
  StaticTF(const rclcpp::NodeOptions &options);

 private:
  void msg_send();
  void send_tf(const std::string &child_id, const std::string &parent_id, double x, double y, double z, double roll, double pitch, double yaw);

  std::string robot_frame_id_;
  std::string odom_frame_id_;
  std::string map_frame_id_;
  std::string left_lidar_frame_id_;
  std::string right_lidar_frame_id_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

}  // namespace UC