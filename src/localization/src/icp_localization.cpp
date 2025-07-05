#include "icp_localization.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace Localization {

inline double normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Matrix2d getR(double theta) {
  Eigen::Matrix2d R;
  double n_theta = normalize_angle(theta);
  R << cos(n_theta), -sin(n_theta),
      sin(n_theta), cos(n_theta);
  return R;
}

Line::Line(const Eigen::Vector2d &position, const Eigen::Vector2d &direction) : pos(position), dir(direction) {}

double Line::distance_to(const Eigen::Vector2d &p) const {
  Eigen::Vector2d normal(-dir.y(), dir.x());
  normal.normalize();
  return normal.dot(p - pos);
}

double LineSeg::distance_to(const Eigen::Vector2d &point) {
  Eigen::Vector2d v = point - line.pos;
  double t = v.dot(line.dir);
  if (t < 0) {
    return (point - line.pos).norm();
  }
  if (t > length) {
    return (point - (line.pos + line.dir * length)).norm();
  }
  return line.distance_to(point);
}

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions &options) : Node("points_integration", options) {
  this->declare_parameter<std::string>("merged_topic_name", "merged_scan");
  this->declare_parameter<std::string>("merged_frame_id", "robot_base");
  this->declare_parameter<std::string>("map_frame_id", "map");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  merged_topic_name = this->get_parameter("merged_topic_name").as_string();
  merged_frame_id = this->get_parameter("merged_frame_id").as_string();
  map_frame_id = this->get_parameter("map_frame_id").as_string();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(merged_topic_name, rclcpp::SensorDataQoS{}, std::bind(&LocalizationNode::topic_callback, this, std::placeholders::_1));
  marker_print = this->create_publisher<visualization_msgs::msg::Marker>("lines_position", 10);
  points_view = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_look_points", 10);
  icp_points_view = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_map_look_points", 10);
}

void LocalizationNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
  pcl::PointCloud<pcl::PointXYZ> map_pcl_cloud;
  pcl::PointCloud<pcl::PointXYZ> robot_pcl_cloud;
  sensor_msgs::msg::PointCloud2 msg_cloud = *msg_ptr;

  try {
    sensor_msgs::msg::PointCloud2 cloud_transformed;
    tf_buffer_->transform(msg_cloud, cloud_transformed, map_frame_id, tf2::durationFromSec(0.1));
    pcl::fromROSMsg(cloud_transformed, map_pcl_cloud);
    pcl::fromROSMsg(msg_cloud, robot_pcl_cloud);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(rclcpp::get_logger("Transform"), "Transform failed: %s", ex.what());
  }

  // pcl::PointCloud<pcl::PointXYZ> clean_cloud;
  // remove_outlier(robot_pcl_cloud, clean_cloud, 0.015);  // 外れ値を除外

  // icp開始
  // Eigen::Vector3d guess_par = {0.0, 4.75, M_PI / 2.0};  // 初期値
  Eigen::Vector3d guess_par = do_icp(robot_pcl_cloud);
  Eigen::Vector3d pre_par(robot_pos.x(), robot_pos.y(), robot_yaw);  // 前回の推定値
  // if ((guess_par - pre_par).norm() > 1.0) {
  //   guess_par = pre_par;
  // } else {
  //   guess_par.z() = normalize_angle(guess_par.z());  // 角度を正規化
  // }
  // RCLCPP_INFO(this->get_logger(), "%f", test_delta_theta);
  RCLCPP_INFO(this->get_logger(), "ICP guess parameters: x: %f, y: %f, theta: %f", guess_par(0), guess_par(1), guess_par(2));

  pcl::PointCloud<pcl::PointXYZ> icp_cloud = robot_pcl_cloud;
  transePoints(icp_cloud, guess_par);  // 点群を変換

  double alfa = 0.0;  // 平滑化の係数
  robot_pos.x() = robot_pos.x() * alfa + (1 - alfa) * guess_par(0);
  robot_pos.y() = robot_pos.y() * alfa + (1 - alfa) * guess_par(1);
  robot_yaw = guess_par(2);

  // icp終了

  update_robot_pose(robot_pos, robot_yaw);  // ロボットの位置と姿勢を更新

  // // map座標系で点群を表示
  auto robot_look_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
  auto icp_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(robot_pcl_cloud, *robot_look_points);
  pcl::toROSMsg(icp_cloud, *icp_points);
  robot_look_points->header.frame_id = map_frame_id;
  icp_points->header.frame_id = map_frame_id;
  points_view->publish(*robot_look_points);
  icp_points_view->publish(*icp_points);
  visualize_line_segs(map_line_segs);
}

//----------------------

void LocalizationNode::ICP(Eigen::Vector3d default_par_init, std::vector<Eigen::Vector2d> &real_points, Eigen::Vector3d &best_guess_par, double &global_min_cost) {
  Eigen::Vector3d default_par = default_par_init;  // 各スレッドでコピーを使う
  Eigen::Vector3d delta_par;
  Eigen::VectorXd min_R;

  for (int inter = 0; inter < 10; inter++) {
    std::vector<int> min_line_IDs(real_points.size());
    std::vector<Eigen::Vector2d> guess_points = real_points;
    Eigen::VectorXd R;
    R.resize(real_points.size());

    transePoints(guess_points, default_par);
    for (size_t i = 0; i < real_points.size(); i++) {
      int min_line_ID = 0;
      double min_dist = std::numeric_limits<double>::max();
      double min_abs_dist = std::numeric_limits<double>::max();
      for (size_t j = 0; j < map_line_segs.size(); j++) {
        double dist = map_line_segs[j].distance_to(guess_points[i]);
        if (std::abs(dist) < min_abs_dist) {
          min_line_ID = j;
          min_dist = dist;
          min_abs_dist = std::abs(dist);
        }
      }
      min_line_IDs[i] = min_line_ID;
      R(i) = min_dist;
    }

    Eigen::MatrixXd J(real_points.size(), 3);  // ヤコビ行列
    for (size_t i = 0; i < real_points.size(); ++i) {
      Eigen::Vector2d N(map_line_segs[min_line_IDs[i]].line.dir.y(), -map_line_segs[min_line_IDs[i]].line.dir.x());
      N.normalize();
      double x = guess_points[i].x();
      double y = guess_points[i].y();
      double theta = default_par.z();

      double d_theta = N.x() * (-sin(theta) * x - cos(theta) * y) +
                       N.y() * (cos(theta) * x - sin(theta) * y);
      J.row(i) << N.x(), N.y(), d_theta;
    }

    Eigen::MatrixXd Jt = J.transpose();
    Eigen::MatrixXd JtJ = Jt * J;
    Eigen::VectorXd JtR = Jt * R;
    delta_par = JtJ.ldlt().solve(JtR);
    delta_par(2) = normalize_angle(delta_par(2));
    default_par += delta_par;
    default_par(2) = normalize_angle(default_par(2));
    min_R = R;
  }

  double cost = min_R.squaredNorm();
  {
    std::lock_guard<std::mutex> lock(mtx);  // 排他制御で共有変数更新
    double dx = default_par_init.x() - default_par.x();
    double dy = default_par_init.y() - default_par.y();
    double dtheta = normalize_angle(default_par_init.z() - default_par.z());
    double pose_diff = std::sqrt(dx * dx + dy * dy + dtheta * dtheta);
    if (cost < global_min_cost) {  // 位置が大きく変化していない場合のみ更新
      global_min_cost = cost;
      best_guess_par = default_par;
    }
  }
}

//----------------------

Eigen::Vector3d LocalizationNode::do_icp(pcl::PointCloud<pcl::PointXYZ> &point_cloud) {
  std::vector<Eigen::Vector2d> target_points;
  for (const auto &point : point_cloud.points) {
    target_points.emplace_back(point.x, point.y);
  }
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> num_dis(-0.5, 0.5);
  std::uniform_real_distribution<double> angle_dis(-0.0, M_PI);

  Eigen::Vector3d delta_par;
  Eigen::VectorXd min_R;
  Eigen::Vector3d guess_par;
  double min_cost = std::numeric_limits<double>::max();
  double ran_angle = 0.0;
  for (int inter = 0; inter < 50; inter++) {
    Eigen::Vector3d guess_guess_par(robot_pos.x() + num_dis(gen), robot_pos.y() + num_dis(gen), normalize_angle(robot_yaw + angle_dis(gen)));  // 初期値
    threads.push_back(std::thread([this, &guess_guess_par, &target_points, &guess_par, &min_cost]() { this->ICP(guess_guess_par, target_points, guess_par, min_cost); }));
  }
  for (std::thread &t : threads) {
    t.join();
  }
  threads.clear();

  return guess_par;  // 最適化されたパラメータを返す
}

void LocalizationNode::transePoints(std::vector<Eigen::Vector2d> &points, Eigen::Vector3d par) {
  Eigen::Vector2d tra = {par(0), par(1)};
  Eigen::Matrix2d R = getR(par(2));
  for (size_t i = 0; i < points.size(); i++) {
    points[i] = R * points[i] + tra;
  }
}

void LocalizationNode::transePoints(pcl::PointCloud<pcl::PointXYZ> &points, Eigen::Vector3d par) {
  Eigen::Vector2d T = {par(0), par(1)};
  Eigen::Matrix2d R = getR(par(2));
  for (size_t i = 0; i < points.size(); i++) {
    Eigen::Vector2d pt(points[i].x, points[i].y);
    pt = R * pt + T;
    points[i].x = pt.x();
    points[i].y = pt.y();
  }
}

void LocalizationNode::update_robot_pose(Eigen::Vector2d &robot_pos, double &robot_yaw) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = map_frame_id;
  t.child_frame_id = "odom";
  t.transform.translation.x = robot_pos.x();
  t.transform.translation.y = robot_pos.y();
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, robot_yaw);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(t);
}

void LocalizationNode::visualize_lines(const std::vector<Line> &lines) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_id;
  marker.header.stamp = this->now();
  marker.ns = "lines";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;  // 線の太さ
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  for (const auto &line : lines) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = line.pos.x() - 100.0 * line.dir.x();
    p1.y = line.pos.y() - 100.0 * line.dir.y();
    p1.z = 0.0;
    p2.x = line.pos.x() + 100.0 * line.dir.x();
    p2.y = line.pos.y() + 100.0 * line.dir.y();
    p2.z = 0.0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  marker_print->publish(marker);
}

void LocalizationNode::visualize_line_segs(const std::vector<LineSeg> &lines) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_id;
  marker.header.stamp = this->now();
  marker.ns = "line_segs";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;  // 線の太さ
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  for (const auto &line : lines) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = line.line.pos.x();
    p1.y = line.line.pos.y();
    p1.z = 0.0;
    p2.x = line.line.pos.x() + line.length * line.line.dir.x();
    p2.y = line.line.pos.y() + line.length * line.line.dir.y();
    p2.z = 0.0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  marker_print->publish(marker);
}

}  // namespace Localization

RCLCPP_COMPONENTS_REGISTER_NODE(Localization::LocalizationNode)