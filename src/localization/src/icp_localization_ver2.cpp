#include "icp_localization_ver2.hpp"

namespace Localization {

double normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

Eigen::Matrix2d getR(double theta) {
  Eigen::Matrix2d R;
  double n_theta = theta;
  R << cos(n_theta), -sin(n_theta),
      sin(n_theta), cos(n_theta);
  return R;
}

double LineSeg::distance_to(const Eigen::Vector2d &p) {
  Eigen::Vector2d v = p - pos;
  double t = v.dot(dir);
  if (t < 0)
    return (p - pos).norm();
  if (t > length)
    return (p - (pos + dir * length)).norm();
  Eigen::Vector2d normal(-dir.y(), dir.x());
  normal.normalize();
  return normal.dot(p - pos);
}

LineSeg LineSeg::transform(const Eigen::Vector3d &par) {  // odom -> robot
  Eigen::Rotation2Dd R(-par.z());
  Eigen::Vector2d t(par.x(), par.y());
  Eigen::Vector2d new_pos = R * (pos - t);
  Eigen::Vector2d new_dir = R * dir;
  return LineSeg(new_pos, new_dir, length);
}

ICPNode::ICPNode(const rclcpp::NodeOptions &options) : Node("ICP_node", options) {
  this->declare_parameter<std::string>("merged_topic_name", "merged_scan");
  this->declare_parameter<std::string>("merged_frame_id", "robot_base");
  this->declare_parameter<std::string>("map_frame_id", "map");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  merged_topic_name = this->get_parameter("merged_topic_name").as_string();
  merged_frame_id = this->get_parameter("merged_frame_id").as_string();
  map_frame_id = this->get_parameter("map_frame_id").as_string();
  odom_frame_id = this->get_parameter("odom_frame_id").as_string();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(merged_topic_name, rclcpp::SensorDataQoS{}, std::bind(&ICPNode::topic_callback, this, std::placeholders::_1));
  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);
  marker_print = this->create_publisher<visualization_msgs::msg::Marker>("lines_position", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ICPNode::timer_callback, this));

  robot_pos = Eigen::Vector3d(0.0, 0.0, M_PI * 0.0);  // ロボットの初期位置と姿勢
}

void ICPNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
  cloud_header_ = msg_ptr->header;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;  // sensor_msgs::msg::PointCloud2 -> pcl::PointCloud<pcl::PointXYZ>
  pcl::fromROSMsg(*msg_ptr, pcl_cloud);
  robot_cloud.clear();
  robot_cloud.reserve(pcl_cloud.size());
  for (const auto &pt : pcl_cloud.points) {  // pcl::PointCloud<pcl::PointXYZ> -> std::vector<Eigen::Vector2d>
    robot_cloud.emplace_back(pt.x, pt.y);
  }

  std::vector<LineSeg> robot_line_segs;
  for (LineSeg &map_line : map_line_segs) {
    LineSeg transformed_line = map_line.transform(robot_pos);  // odom -> robot
    robot_line_segs.push_back(transformed_line);
  }

  // ICPの実行

  Eigen::Vector3d icp_result = do_icp(robot_cloud, robot_line_segs);

  // RCLCPP_INFO(this->get_logger(), "ICP Result: x: %.2f, y: %.2f, theta: %.2f", icp_result.x(), icp_result.y(), icp_result.z());

  // ロボットの位置更新
  // RCLCPP_INFO(this->get_logger(), "ICP Result: x: %f, y: %f, theta: %f", icp_result.x(), icp_result.y(), icp_result.z());

  // robot_pos = {0.0, 0.0, M_PI * 0.5};

  pre_pos_error = {icp_result.x(), icp_result.y()};
  Eigen::Vector2d pos(icp_result.x(), icp_result.y());
  Eigen::Vector2d new_pos = getR(robot_pos.z()) * pos;
  robot_pos.z() += icp_result.z();
  robot_pos.z() = normalize_angle(robot_pos.z());

  new_robot_pos = {new_pos.x(), new_pos.y(), robot_pos.z()};
  // RCLCPP_INFO(this->get_logger(), "Robot Pos x: %f y: %f yaw: %f", new_robot_pos.x(), new_robot_pos.y(), new_robot_pos.z());
  std::vector<LineSeg> icp_line_segs;

  for (LineSeg &line_seg : robot_line_segs) {
    LineSeg transformed_line = line_seg.transform(icp_result);  // robot -> odom
    icp_line_segs.push_back(transformed_line);
  }
  visualize_line_segments(robot_line_segs);
  visualize_line_segments(icp_line_segs);
}

Eigen::Vector3d ICPNode::do_icp(std::vector<Eigen::Vector2d> &point_cloud, std::vector<LineSeg> &line_segments) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> num_dis(-0.25, 0.25);
  std::uniform_real_distribution<double> angle_dis(-M_PI, M_PI);

  Eigen::Vector3d delta_par;
  Eigen::Vector3d guess_par;
  double min_cost = std::numeric_limits<double>::max();
  // for (int inter = 0; inter < 1; inter++) {
  Eigen::Vector3d guess_guess_par(pre_pos_error.x(), pre_pos_error.y(), 0.0);  // 初期値
  // Eigen::Vector3d guess_guess_par(new_robot_pos.x(), new_robot_pos.y(), 0.0);  // 初期値
  //   // Eigen::Vector3d guess_guess_par(num_dis(gen), num_dis(gen), angle_dis(gen) / 8.0);  // 初期値
  //   threads.push_back(std::thread([this, &guess_guess_par, &point_cloud, &line_segments, &guess_par, &min_cost]() { this->ICP(guess_guess_par, point_cloud, line_segments, guess_par, min_cost); }));
  // }
  // for (std::thread &t : threads) {
  //   t.join();
  // }
  ICP(guess_guess_par, point_cloud, line_segments, guess_par, min_cost);
  // threads.clear();
  return guess_par;
}

//----------------------

void ICPNode::ICP(Eigen::Vector3d default_par_init, std::vector<Eigen::Vector2d> &real_points, std::vector<LineSeg> &line_segs, Eigen::Vector3d &best_guess_par, double &global_min_cost) {
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
      for (size_t j = 0; j < line_segs.size(); j++) {
        double dist = line_segs[j].distance_to(guess_points[i]);
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
      Eigen::Vector2d N(line_segs[min_line_IDs[i]].dir.y(), -line_segs[min_line_IDs[i]].dir.x());
      N.normalize();
      double x = real_points[i].x();
      double y = real_points[i].y();
      double theta = default_par.z();

      double d_theta = N.x() * (-sin(theta) * x - cos(theta) * y) +
                       N.y() * (cos(theta) * x - sin(theta) * y);
      J.row(i) << N.x(), N.y(), d_theta;
    }

    Eigen::MatrixXd Jt = J.transpose();
    Eigen::MatrixXd JtJ = Jt * J;
    Eigen::VectorXd JtR = Jt * R;
    delta_par = JtJ.ldlt().solve(JtR);
    default_par += delta_par;
    default_par(2) = normalize_angle(default_par(2));
    min_R = R;
  }

  double cost = min_R.squaredNorm();
  {
    std::lock_guard<std::mutex> lock(mtx);  // 排他制御で共有変数更新
    if (cost < global_min_cost) {
      global_min_cost = cost;
      best_guess_par = default_par;
    }
  }
}

//----------------------

void ICPNode::transePoints(std::vector<Eigen::Vector2d> &points, Eigen::Vector3d par) {
  Eigen::Vector2d tra = {par(0), par(1)};
  Eigen::Matrix2d R = getR(par(2));
  for (size_t i = 0; i < points.size(); i++) {
    points[i] = R * points[i] + tra;
  }
}

void ICPNode::timer_callback() {
  // 初期位置の設定
  RCLCPP_INFO(this->get_logger(), "Robot Pos x: %f y: %f yaw: %f", new_robot_pos.x(), new_robot_pos.y(), new_robot_pos.z());
  // Eigen::Vector2d pos(new_robot_pos.x(), new_robot_pos.y());
  // Eigen::Vector2d new_pos = getR(new_robot_pos.z()) * pos;
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = cloud_header_.stamp;
  t.header.frame_id = map_frame_id;
  t.child_frame_id = odom_frame_id;
  t.transform.translation.x = new_robot_pos.x();
  t.transform.translation.y = new_robot_pos.y();
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, new_robot_pos.z());
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(t);
}

void ICPNode::visualize_line_segments(std::vector<LineSeg> &line_segments) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = cloud_header_.stamp;
  marker.header.frame_id = "robot_base";
  marker.ns = "line_segs";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;  // 線の太さ
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  for (const auto &line : line_segments) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = line.pos.x();
    p1.y = line.pos.y();
    p1.z = 0.0;
    p2.x = line.pos.x() + line.length * line.dir.x();
    p2.y = line.pos.y() + line.length * line.dir.y();
    p2.z = 0.0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }
  marker_print->publish(marker);
}

}  // namespace Localization

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(Localization::ICPNode)