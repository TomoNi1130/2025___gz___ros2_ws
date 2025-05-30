#include "points_integration.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace points_processes {

PointIntegration::PointIntegration(const rclcpp::NodeOptions &options) : Node("points_integration", options) {
  this->declare_parameter<std::vector<std::string>>("scan_topic_names", std::vector<std::string>());
  this->declare_parameter<std::string>("merged_topic_name", "merged_scan");
  this->declare_parameter<std::string>("merged_frame_id", "robot_base");

  scan_topic_names = this->get_parameter("scan_topic_names").as_string_array();
  merged_topic_name = this->get_parameter("merged_topic_name").as_string();
  merged_frame_id = this->get_parameter("merged_frame_id").as_string();

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PointIntegration::send_merged_scan, this));
  points_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merged_points", 10);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  for (const auto &topic : scan_topic_names) {
    auto callback = [this, topic](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { scan_callback(topic, msg); };
    auto subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS{}, std::move(callback), sub_options);
    subscriptions_.push_back(subscription);
  }
}

void PointIntegration::scan_callback(const std::string &topic_name, sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  auto tfed_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
  try {
    projector_.transformLaserScanToPointCloud(merged_frame_id, *msg, *tfed_points, *tf_buffer_);

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
  }
  scans_[topic_name] = std::make_shared<sensor_msgs::msg::PointCloud2>(*tfed_points);
}

void PointIntegration::send_merged_scan() {
  if (scans_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No scans received yet.");
    return;
  }
  auto merged_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::PointCloud<pcl::PointXYZ> merged_cloud_pcl;
  for (const auto &pair : scans_) {
    sensor_msgs::msg::PointCloud2::SharedPtr scan = pair.second;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*scan, pcl_cloud);
    pcl_cloud = resampler(pcl_cloud);  // 点群のリサンプリング
    merged_cloud_pcl += pcl_cloud;     // 点群の統合
  }
  pcl::toROSMsg(merged_cloud_pcl, *merged_points);
  merged_points->header.frame_id = merged_frame_id;
  points_cloud_publisher_->publish(*merged_points);
}

bool PointIntegration::point_inserter(const pcl::PointXYZ &point, const pcl::PointXYZ &pre_point, pcl::PointXYZ &new_point, bool &inserted) {
  float dx = point.x - pre_point.x;
  float dy = point.y - pre_point.y;
  float point_distance = sqrt(dx * dx + dy * dy);
  if (total_dis + point_distance > point_dis_threshold) {  // 点が遠すぎる->点を維持
    new_point = point;
    inserted = false;                                                // 今の点を維持
  } else if (total_dis + point_distance < equalization_point_dis) {  // 点が近すぎる->点を削除、累積距離を更新
    total_dis += point_distance;
    inserted = false;
    return false;
  } else {  // 間が開いている->間を埋めるように点を追加
    float ratio = (equalization_point_dis - total_dis) / point_distance;
    new_point.x = dx * ratio + pre_point.x;
    new_point.y = dy * ratio + pre_point.y;
    inserted = true;  // 点が追加されているフラグ
  }
  return true;
}

pcl::PointCloud<pcl::PointXYZ> PointIntegration::resampler(const pcl::PointCloud<pcl::PointXYZ> &target_points) {
  pcl::PointCloud<pcl::PointXYZ> resampled_points;  // 修正後の点群
  if (target_points.points.empty())
    return resampled_points;  // 空ならそのまま返す
  pcl::PointXYZ target_point;
  pcl::PointXYZ pre_point;
  pcl::PointXYZ adding_point;

  target_point = target_points.points[0];
  pre_point = target_points.points[0];
  adding_point = target_points.points[0];
  if (std::isfinite(target_point.x)) {
    resampled_points.points.push_back(adding_point);
    for (size_t i = 1; i < target_points.points.size(); i++) {
      target_point = target_points.points[i];
      if (std::isfinite(target_point.x)) {
        bool inserted = false;                                                         // 点を追加した
        bool exist = point_inserter(target_point, pre_point, adding_point, inserted);  // 点を削除しない
        if (exist) {
          resampled_points.points.push_back(adding_point);
          pre_point = adding_point;
          total_dis = 0;
          if (inserted)  // 点を追加したのでもう一度
            i--;
        } else
          pre_point = target_point;
      }
    }
  }
  return resampled_points;
}
}  // namespace points_processes

RCLCPP_COMPONENTS_REGISTER_NODE(points_processes::PointIntegration)