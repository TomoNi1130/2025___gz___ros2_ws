#include <Eigen/Dense>
#include <limits>

#include "interface_pkg/msg/corners.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

std::string frames[] = {
    "BL_corner",
    "FL_corner",
    "FR_corner",
    "BR_corner",
    "center_BL_corner",
    "center_BR_corner",
    "center_FL_corner",
    "center_FR_corner",
    "share_center_BL_corner",
    "share_center_FL_corner",
    "share_center_FR_corner",
    "share_center_BR_corner",
};

double getYawFromTransform(const geometry_msgs::msg::TransformStamped &transform) {
  tf2::Quaternion q(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  return yaw;  // Yaw角（ラジアン）
}

struct MapCorner {
  Eigen::Vector2d corner_to_map;           // 普遍
  Eigen::Vector2d robot_to_corner;         // ロボから見たロボから角の場所
  Eigen::Vector2d static_robot_to_corner;  // mapから見たロボから角の場所
  bool fresh;
  MapCorner() : corner_to_map(Eigen::Vector2d(0.0, 0.0)), fresh(false) {};
  MapCorner(geometry_msgs::msg::TransformStamped &t) : corner_to_map(Eigen::Vector2d(t.transform.translation.x, t.transform.translation.y)), fresh(false) {};

  bool similar_to(Eigen::Vector2d &robot_to_corner, Eigen::Vector2d &robot_to_map) { return (corner_to_map - (-robot_to_corner + robot_to_map)).norm() < 0.2; }
  Eigen::Vector2d get_robot_map() { return robot_to_corner + corner_to_map; }
};

class CornerLocalization : public rclcpp::Node {
 public:
  CornerLocalization()
      : Node("CornerLocalization_node") {
    subscription_ = this->create_subscription<interface_pkg::msg::Corners>(
        "ransac_corners_position", 10, std::bind(&CornerLocalization::topicCallback, this, std::placeholders::_1));
    corner_print = this->create_publisher<visualization_msgs::msg::Marker>("tracked_corner", 10);
    coments_print = this->create_publisher<visualization_msgs::msg::MarkerArray>("coments_for_corner", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    bool tf_sacsess = false;
    geometry_msgs::msg::TransformStamped t;
    while (!tf_sacsess) {
      try {
        for (std::string frame : frames) {
          t = tf_buffer_->lookupTransform(frame, "map", tf2::TimePointZero);
          map_corners.push_back(MapCorner(t));
        }
        t = tf_buffer_->lookupTransform("robot_origin", "map", tf2::TimePointZero);
        robot_to_map.x() = t.transform.translation.x;
        robot_to_map.y() = t.transform.translation.y;
        robot_angle = getYawFromTransform(t);
        RCLCPP_INFO(this->get_logger(), "get_tf!!");
        tf_sacsess = true;
      } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "waiting for tf...");
        map_corners.clear();
        rclcpp::sleep_for(500ms);
      }
    }
  }

 private:
  void identify_corners(const interface_pkg::msg::Corners &msg)  // msgからコーナーの対応を調べる
  {
    for (size_t i = 0; i < msg.x.size(); i++) {
      double distance = msg.distance[i];
      Eigen::Vector2d lidar_robot_to_corner(msg.x[i], msg.y[i]);
      if (!this_is_known_corner(lidar_robot_to_corner)) {
        Eigen::Vector2d static_robot_to_corner(distance * cos(robot_angle + atan2(msg.y[i], msg.x[i])), distance * sin(robot_angle + atan2(msg.y[i], msg.x[i])));
        Eigen::Vector2d guess_corner_to_map = robot_to_map - static_robot_to_corner;
        for (MapCorner map_corner : map_corners) {
          if ((map_corner.corner_to_map - guess_corner_to_map).norm() < 0.5) {
            map_corner.fresh = true;
            map_corner.robot_to_corner = lidar_robot_to_corner;
            map_corner.static_robot_to_corner = static_robot_to_corner;
            break;
          }
        }
      }
    }
  }
  bool this_is_known_corner(Eigen::Vector2d &target_vector)  // コーナーが既知のコーナーの後継であるかどうか、距離を見て測る。
  {
    double length = target_vector.norm();
    double corner_angle = atan2(target_vector.y(), target_vector.x());
    int nearest_corner_id;
    double smallest_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < map_corners.size(); i++) {
      if ((map_corners[i].robot_to_corner - target_vector).norm() < smallest_distance) {
        smallest_distance = (map_corners[i].robot_to_corner - target_vector).norm();
        nearest_corner_id = i;
      }
    }
    if (smallest_distance < 1.5) {
      map_corners[nearest_corner_id].robot_to_corner = target_vector;
      map_corners[nearest_corner_id].static_robot_to_corner = Eigen::Vector2d(length * cos(robot_angle + corner_angle), length * sin(robot_angle + corner_angle));
      map_corners[nearest_corner_id].fresh = true;
      return true;
    }
    return false;
  }
  void localization(const interface_pkg::msg::Corners &msg)  // 把握しているコーナーmapから見たマップコーナーの位置からロボットの位置と向きを調べる
  {
    // 位置の測定
    Eigen::Vector2d guess_robot_to_map;
    for (size_t i = 0; i < map_corners.size(); i++) {
      if (map_corners[i].fresh) {
        guess_robot_to_map = (guess_robot_to_map + (map_corners[i].static_robot_to_corner + map_corners[i].corner_to_map)) / 2.0;
      }
    }
    RCLCPP_INFO(this->get_logger(), "%f,%f", guess_robot_to_map.x(), guess_robot_to_map.y());
    robot_to_map = guess_robot_to_map;
    // 角度の測定  //２つの角を結ぶベクトルのmapとroboから見た角度を比較する
    int corner_1_id, corner_2_id;
    for (size_t i = 0; i < map_corners.size(); i++) {
      if (map_corners[i].fresh) {
        corner_1_id = i;
      }
    }
    for (size_t i = map_corners.size(); i == 0; i -= 1) {
      if (map_corners[i].fresh) {
        corner_2_id = i;
      }
    }
    Eigen::Vector2d map_look_corner_to_corner = map_corners[corner_1_id].corner_to_map - map_corners[corner_2_id].corner_to_map;
    Eigen::Vector2d robot_look_corner_to_corner = -map_corners[corner_1_id].robot_to_corner + map_corners[corner_2_id].robot_to_corner;
    double map_look_v_angle = atan2(map_look_corner_to_corner.y(), map_look_corner_to_corner.x());
    double robot_look_v_angle = atan2(robot_look_corner_to_corner.y(), robot_look_corner_to_corner.x());
    robot_angle = map_look_v_angle - robot_look_v_angle;
  }
  void topicCallback(const interface_pkg::msg::Corners &msg) {
    RCLCPP_INFO(this->get_logger(), "%f,%f", robot_to_map.x(), robot_to_map.y());
    RCLCPP_INFO(this->get_logger(), "%f", robot_angle);
    for (MapCorner map_corner : map_corners)
      map_corner.fresh = false;
    identify_corners(msg);
    localization(msg);

    RCLCPP_INFO(this->get_logger(), "--------------------");
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::MarkerArray coments;
    marker.header.frame_id = "robot_base";
    marker.ns = "robot_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = marker.scale.x;
    marker.scale.z = marker.scale.y;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;  // Don't forget to set the alpha!

    RCLCPP_INFO(this->get_logger(), "%zu", map_corners.size());
    for (int i = 0; i < map_corners.size(); i++) {
      geometry_msgs::msg::Point print_point;
      print_point.x = map_corners[i].robot_to_corner.x();
      print_point.y = map_corners[i].robot_to_corner.y();
      print_point.z = 0.0;

      visualization_msgs::msg::Marker coment;
      coment.header.frame_id = "robot_base";
      coment.ns = "coment";
      coment.id = i + 1;  // SPHERE_LISTとは異なるユニークなID
      coment.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      coment.action = visualization_msgs::msg::Marker::ADD;

      coment.pose.orientation.x = 0.0;
      coment.pose.orientation.y = 0.0;
      coment.pose.orientation.z = 0.0;
      coment.pose.orientation.w = 1.0;
      coment.scale.z = 0.3;  // 例: 文字の高さ0.3m
      coment.color.r = 1.0;
      coment.color.g = 1.0;
      coment.color.b = 1.0;
      coment.color.a = 1.0;  // 不透明 (白)
      coment.pose.position.x = print_point.x + 0.3;
      coment.pose.position.y = print_point.y;
      coment.pose.position.z = print_point.z;

      coment.text = frames[i];

      // if (map_corners[i].fresh) {
      marker.points.push_back(print_point);
      coments.markers.push_back(coment);
      // }
    }
    coments_print->publish(coments);
    corner_print->publish(marker);
  }
  rclcpp::Subscription<interface_pkg::msg::Corners>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr corner_print;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coments_print;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::vector<MapCorner> map_corners;
  Eigen::Vector2d robot_to_map;
  double robot_angle;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CornerLocalization>());
  rclcpp::shutdown();
  return 0;
}