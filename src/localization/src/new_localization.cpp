#include <Eigen/Dense>
#include <limits>

#include "interface_pkg/msg/corners.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

// 1-0.引き継ぎ
//     最も近い現在追跡しているコーナーとの距離が閾値以下であれば引き継ぐ
// 1-1.特定(引き継ぎがなければ)
//     corner_to_map robot_to_map - robot_to_corner　と　map_cornerのcorner_to_mapと比べる
// 2-0.localization
//   2-1.座標の特定
//       static_robot_to_corner + corner_to_map
//   2-2.向きの特定
//       特定済みの角２つについて、ロボとmapから見た角度を比べる
// 3-0.位置の更新
//    tf

// std::string frames[] = {
//     "FR_corner",
//     "BR_corner",
//     "center_FR_corner",
//     "center_BR_corner",
//     "share_center_FR_corner",
//     "share_center_BR_corner",
// };

std::string frames[] = {
    // 左側スタート用
    "BL_corner",
    "FL_corner",
    "center_BL_corner",
    "center_FL_corner",
    "share_center_BL_corner",
    "share_center_FL_corner",

};

struct MapCorner {
  Eigen::Vector2d corner_to_map;           // 不変
  Eigen::Vector2d robot_to_corner;         // ロボから見たロボからこの角の場所
  Eigen::Vector2d static_robot_to_corner;  // mapから見たロボからこの角の場所
  bool is_fresh;
  MapCorner() : corner_to_map(Eigen::Vector2d::Zero()), robot_to_corner(Eigen::Vector2d::Zero()), static_robot_to_corner(Eigen::Vector2d::Zero()), is_fresh(false) {};
  MapCorner(geometry_msgs::msg::TransformStamped &t) : corner_to_map(Eigen::Vector2d(t.transform.translation.x, t.transform.translation.y)), robot_to_corner(Eigen::Vector2d::Zero()), static_robot_to_corner(Eigen::Vector2d::Zero()), is_fresh(false) {};

  bool similar_to(Eigen::Vector2d &robot_to_corner, Eigen::Vector2d &robot_to_map) { return (corner_to_map - (-robot_to_corner + robot_to_map)).norm() < 0.2; }
  Eigen::Vector2d get_robot_map() { return robot_to_corner + corner_to_map; }
};

double getYawFromTransform(const geometry_msgs::msg::TransformStamped &transform) {
  tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

class CornerLocalization : public rclcpp::Node {
 public:
  CornerLocalization()
      : Node("CornerLocalization_node") {
    subscription_ = this->create_subscription<interface_pkg::msg::Corners>(
        "ransac_corners_position", 10, std::bind(&CornerLocalization::topicCallback, this, std::placeholders::_1));
    corner_print = this->create_publisher<visualization_msgs::msg::Marker>("tracked_corner", 10);
    informations_print = this->create_publisher<visualization_msgs::msg::MarkerArray>("coments_for_corner", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
  // 視覚化
  void visualize_tracking_corner();
  // 処理
  void identify_corners(const interface_pkg::msg::Corners &msg) {
    std::vector<bool> matched_new_corners(msg.x.size(), false);
    for (size_t i = 0; i < msg.x.size(); i++) {
      Eigen::Vector2d new_robot_to_corner(msg.x[i], msg.y[i]);
      double distance = msg.distance[i];
      double angle = atan2(msg.y[i], msg.x[i]);
      // 引き継ぎ
      int nearest_corner_id;
      double smallest_distance = std::numeric_limits<double>::infinity();
      for (size_t j = 0; j < map_corners.size(); j++) {
        if ((map_corners[j].robot_to_corner - new_robot_to_corner).norm() < smallest_distance) {
          nearest_corner_id = j;
          smallest_distance = (map_corners[j].robot_to_corner - new_robot_to_corner).norm();
        }
      }
      double pos_param = 0.8;
      if (smallest_distance < 2.0 && !map_corners[nearest_corner_id].is_fresh) {
        map_corners[nearest_corner_id].robot_to_corner = map_corners[nearest_corner_id].robot_to_corner * (1.0 - pos_param) + new_robot_to_corner * pos_param;
        map_corners[nearest_corner_id].static_robot_to_corner = Eigen::Vector2d(distance * cos(robot_angle + angle), distance * sin(robot_angle + angle));
        map_corners[nearest_corner_id].is_fresh = true;
      } else  // コーナーの特定
      {
        Eigen::Vector2d static_robot_to_corner(distance * cos(robot_angle + angle), distance * sin(robot_angle + angle));
        Eigen::Vector2d guess_corner_to_map = robot_to_map - static_robot_to_corner;
        double smallest_error = std::numeric_limits<double>::infinity();
        int optimal_id;
        for (size_t j = 0; j < map_corners.size(); j++) {
          if ((map_corners[j].corner_to_map - guess_corner_to_map).norm() < smallest_error) {
            smallest_error = (map_corners[j].corner_to_map - guess_corner_to_map).norm();
            optimal_id = j;
          }
        }
        if (smallest_error < 0.1) {
          map_corners[optimal_id].robot_to_corner = new_robot_to_corner;
          map_corners[optimal_id].static_robot_to_corner = static_robot_to_corner;
          map_corners[optimal_id].is_fresh = true;
        }
      }
    }
  }
  void localization() {
    // 座標の特定
    Eigen::Vector2d sun_robot_to_map = Eigen::Vector2d::Zero();
    int num = 0;
    for (MapCorner map_corner : map_corners)
      if (map_corner.is_fresh) {
        sun_robot_to_map += (map_corner.static_robot_to_corner + map_corner.corner_to_map);
        num++;
      }
    double pram = 0.7;
    if (num != 0)
      robot_to_map = robot_to_map * (1.0 - pram) + pram * (sun_robot_to_map / double(num));

    // 向きの特定
    Eigen::Vector2d angle_vector_sum = Eigen::Vector2d::Zero();
    int angle_num = 0;
    for (size_t i = 0; i < map_corners.size(); i++) {
      if (map_corners[i].is_fresh)
        for (size_t j = i + 1; j < map_corners.size(); j++) {
          if (map_corners[j].is_fresh) {
            Eigen::Vector2d map_look_corner_to_corner = map_corners[i].corner_to_map - map_corners[j].corner_to_map;
            RCLCPP_INFO(this->get_logger(), "map_look:%f,%f", map_look_corner_to_corner.x(), map_look_corner_to_corner.y());
            Eigen::Vector2d robot_look_corner_to_corner = -map_corners[i].robot_to_corner + map_corners[j].robot_to_corner;
            RCLCPP_INFO(this->get_logger(), "robot_look:%f,%f", robot_look_corner_to_corner.x(), robot_look_corner_to_corner.y());
            if (map_look_corner_to_corner.norm() > 1e-6 && robot_look_corner_to_corner.norm() > 1e-6) {
              double map_look_v_angle = atan2(map_look_corner_to_corner.y(), map_look_corner_to_corner.x());
              double robot_look_v_angle = atan2(robot_look_corner_to_corner.y(), robot_look_corner_to_corner.x());
              double angle_diff = map_look_v_angle - robot_look_v_angle;
              while (angle_diff <= -M_PI) angle_diff += 2.0 * M_PI;
              while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
              angle_vector_sum += Eigen::Vector2d(cos(angle_diff), sin(angle_diff));
              angle_num++;
            }
          }
        }
    }
    double angle_pram = 0.8;
    if (angle_num != 0)
      if (angle_vector_sum.norm() > 1e-6)  // 合計ベクトルがゼロでないことを確認
        robot_angle = robot_angle * (1 - angle_pram) + atan2(angle_vector_sum.y(), angle_vector_sum.x()) * angle_pram;
  }
  void update_robot_to_map() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform.translation.x = -robot_to_map.x();
    t.transform.translation.y = -robot_to_map.y();
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_angle);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }
  void topicCallback(const interface_pkg::msg::Corners &msg) {
    for (size_t i = 0; i < map_corners.size(); i++)
      map_corners[i].is_fresh = false;
    identify_corners(msg);
    localization();
    update_robot_to_map();
    visualize_tracking_corner();
    RCLCPP_INFO(this->get_logger(), "--------------------");
    for (size_t i = 0; i < map_corners.size(); i++)
      RCLCPP_INFO(this->get_logger(), "%s:(%f,%f)", frames[i].c_str(), map_corners[i].robot_to_corner.x(), map_corners[i].robot_to_corner.y());
    RCLCPP_INFO(this->get_logger(), "%f,%f", robot_to_map.x(), robot_to_map.y());
    RCLCPP_INFO(this->get_logger(), "%f", robot_angle);
  }
  rclcpp::Subscription<interface_pkg::msg::Corners>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr corner_print;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr informations_print;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<MapCorner> map_corners;  // 現実のコーナーと対応したデータのまとまり
  Eigen::Vector2d robot_to_map;
  double robot_angle;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CornerLocalization>());
  rclcpp::shutdown();
  return 0;
}

void CornerLocalization::visualize_tracking_corner() {
  visualization_msgs::msg::Marker tracked_points;
  visualization_msgs::msg::MarkerArray points_informations;

  tracked_points.header.frame_id = "robot_base";
  tracked_points.ns = "tracked_points";
  tracked_points.id = 0;
  tracked_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  tracked_points.action = visualization_msgs::msg::Marker::ADD;
  tracked_points.scale.x = 0.2f;  // サイズ
  tracked_points.scale.y = 0.2f;
  tracked_points.scale.z = 0.2f;
  tracked_points.color.r = 1.0f;  // 配色
  tracked_points.color.g = 1.0f;
  tracked_points.color.b = 1.0f;
  tracked_points.color.a = 1.0;

  int id_num = 1;
  for (size_t i = 0; i < map_corners.size(); i++) {
    if (map_corners[i].is_fresh) {
      geometry_msgs::msg::Point print_point;
      print_point.x = map_corners[i].robot_to_corner.x();
      print_point.y = map_corners[i].robot_to_corner.y();
      print_point.z = 0.0;
      tracked_points.points.push_back(print_point);

      visualization_msgs::msg::Marker information;
      information.header.frame_id = "robot_base";
      information.ns = "informations";
      information.id = id_num;
      information.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      information.action = visualization_msgs::msg::Marker::ADD;
      information.pose.orientation.x = 0.0;
      information.pose.orientation.y = 0.0;
      information.pose.orientation.z = 0.0;
      information.pose.orientation.w = 1.0;
      information.scale.z = 0.3;  // 例: 文字の高さ0.3m
      information.color.r = 1.0;
      information.color.g = 1.0;
      information.color.b = 1.0;
      information.color.a = 1.0;
      information.pose.position.x = print_point.x + 0.3;
      information.pose.position.y = print_point.y;
      information.pose.position.z = print_point.z;

      information.text = frames[i];
      points_informations.markers.push_back(information);
      id_num++;
    }
  }
  informations_print->publish(points_informations);
  corner_print->publish(tracked_points);
}