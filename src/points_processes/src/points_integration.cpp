#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

using namespace std::chrono_literals;

class Points_Integration : public rclcpp::Node {
 public:
  Points_Integration()
      : Node("turtle_tf2_frame_listener") {
    left_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "left_lidar", 10, std::bind(&Points_Integration::topic_callback, this, _1));
    right_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "right_lidar", 10, std::bind(&Points_Integration::topic_callback, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "lidar_points", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&Points_Integration::send_msg, this));
    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    bool tf_ok = false;
    geometry_msgs::msg::TransformStamped left_to_robot;
    geometry_msgs::msg::TransformStamped right_to_robot;
    while (tf_ok != true)
      try {
        left_to_robot = tf_buffer_->lookupTransform("left_lidar", "robot_base", tf2::TimePointZero);
        robot_to_left_tf.x = -left_to_robot.transform.translation.x;
        robot_to_left_tf.y = -left_to_robot.transform.translation.y;
        robot_to_left_tf.z = -left_to_robot.transform.translation.z;
        right_to_robot = tf_buffer_->lookupTransform("right_lidar", "robot_base", tf2::TimePointZero);
        robot_to_right_tf.x = -right_to_robot.transform.translation.x;
        robot_to_right_tf.y = -right_to_robot.transform.translation.y;
        robot_to_right_tf.z = -right_to_robot.transform.translation.z;
        tf_ok = true;
      } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "wait for tf data...");
        rclcpp::sleep_for(500ms);
      }
    merged_cloud.header.frame_id = "robot_base";
  }

 private:
  void send_msg() {
    points_integration(right_cloud.points, left_cloud.points);
    if (!merged_cloud.points.empty()) {
      merged_cloud.header.frame_id = "robot_base";
      publisher_->publish(merged_cloud);
      merged_cloud.points.clear();
    }
  }
  void points_integration(std::vector<geometry_msgs::msg::Point32> &right_points, std::vector<geometry_msgs::msg::Point32> &left_points) {
    if (right_points.size() == left_points.size()) {
      merged_cloud.points.resize(right_points.size() + left_points.size());
      for (size_t i = 0; i < right_points.size(); i += 1) {
        merged_cloud.points[i].x = left_points[i].x + robot_to_left_tf.y;
        merged_cloud.points[i].y = left_points[i].y + robot_to_left_tf.x;
        merged_cloud.points[i + left_points.size()].x = -(right_points[i].x + robot_to_right_tf.y);
        merged_cloud.points[i + left_points.size()].y = -(right_points[i].y + robot_to_right_tf.x);
      }
    }
  }
  void topic_callback(const sensor_msgs::msg::LaserScan &msg) {
    if (msg.header.frame_id == "robot/robot_base/left_lidar") {
      left_cloud.points.clear();
      double increment = msg.angle_increment;
      geometry_msgs::msg::Point32 point;
      for (size_t i = 0; i < msg.ranges.size(); i++) {
        double angle = i * increment;
        double distance = msg.ranges[i];
        point.x = distance * cos(angle);
        point.y = distance * sin(angle);
        point.z = 0.0;
        left_cloud.points.push_back(point);
      }
    } else if (msg.header.frame_id == "robot/robot_base/right_lidar") {
      right_cloud.points.clear();
      double increment = msg.angle_increment;
      geometry_msgs::msg::Point32 point;
      for (size_t i = 0; i < msg.ranges.size(); i++) {
        double angle = i * increment;
        double distance = msg.ranges[i];
        point.x = distance * cos(angle);
        point.y = distance * sin(angle);
        right_cloud.points.push_back(point);
      }
    }
  }
  geometry_msgs::msg::Vector3 robot_to_left_tf;
  geometry_msgs::msg::Vector3 robot_to_right_tf;
  sensor_msgs::msg::PointCloud right_cloud;
  sensor_msgs::msg::PointCloud left_cloud;
  sensor_msgs::msg::PointCloud merged_cloud;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr left_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr right_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<Points_Integration>());
  rclcpp::shutdown();
  return 0;
}