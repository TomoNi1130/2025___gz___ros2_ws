#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

class static_tf_about_lidar_to_robot : public rclcpp::Node {
 public:
  static_tf_about_lidar_to_robot()
      : Node("static_tf_about_lidar_to_robot_node") {
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&static_tf_about_lidar_to_robot::msg_send, this));
  }

 private:
  void msg_send() {
    publish_wheel_transform("BL_corner", "map", -0.5, 5.25, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("BR_corner", "map", -0.5, -5.25, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("FL_corner", "map", 10.0, 5.25, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("FR_corner", "map", 10.0, -5.25, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("center_BL_corner", "map", -0.5, 0.0225, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("center_BR_corner", "map", -0.5, -0.0225, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("center_FL_corner", "map", 6.3, 0.0225, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("center_FR_corner", "map", 6.3, -0.0225, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("share_center_BL_corner", "map", 6.3, 0.225, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("share_center_BR_corner", "map", 6.3, -0.225, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("share_center_FL_corner", "map", 10.0, 0.225, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("share_center_FR_corner", "map", 10.0, -0.225, 0.0, 0.0, 0.0, 0.0);
    publish_wheel_transform("robot_origin", "map", 0.0, 4.75, 0.0, 0.0, 0.0, 0.0);
  }
  void publish_wheel_transform(const std::string &child_id, const std::string &parent_id, double x, double y, double z, double roll, double pitch, double yaw) {
    geometry_msgs::msg::TransformStamped transformStamped;

    // 親フレームと子フレームを設定
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = parent_id;  // 親フレーム
    transformStamped.child_frame_id = child_id;

    // 座標変換（位置）
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;

    // 回転（RPY → クォータニオン）
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // 座標変換をパブリッシュ
    broadcaster_->sendTransform(transformStamped);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<static_tf_about_lidar_to_robot>());
  rclcpp::shutdown();
  return 0;
}