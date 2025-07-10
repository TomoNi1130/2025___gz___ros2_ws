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
  robot_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("robot_yaw", 10);
  clean_cloud_sub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clean_cloud", 10);
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

  remove_outliers(robot_cloud, 0.08, 5, 200);
  clean_cloud_sub_->publish(Eigen_to_cloud(robot_cloud, cloud_header_));

  Eigen::Vector3d icp_result = do_icp(robot_cloud, robot_line_segs);

  // ロボットの位置更新

  Eigen::Vector2d pos_error(icp_result.x(), icp_result.y());
  if (pos_error.norm() < 10.0) {
    pre_pos_error = pos_error;
    Eigen::Vector2d pos = pos_error;
    Eigen::Vector2d new_pos = getR(robot_pos.z()) * pos;
    robot_pos.z() += icp_result.z();
    robot_pos.z() = normalize_angle(robot_pos.z());

    new_robot_pos = {new_pos.x(), new_pos.y(), robot_pos.z()};
  }
  std::vector<LineSeg> icp_line_segs;

  for (LineSeg &line_seg : robot_line_segs) {
    LineSeg transformed_line = line_seg.transform(icp_result);  // robot -> odom
    icp_line_segs.push_back(transformed_line);
  }
  visualize_line_segments(map_line_segs);
}

void ICPNode::remove_outliers(std::vector<Eigen::Vector2d> &target_cloud, const double threshold, const int max_line_num, const int max_iterations) {
  struct ransac_line {
    Eigen::Vector2d pos, dir;
    ransac_line(Eigen::Vector2d &pos, Eigen::Vector2d &dir) : pos(pos), dir(dir) {}
    double distance_to(const Eigen::Vector2d &p) const {
      Eigen::Vector2d v = p - pos;
      return std::abs(dir.x() * v.y() - dir.y() * v.x()) / dir.norm();
    }
  };
  std::random_device rd;
  std::mt19937 gen(rd());
  int cloud_size = target_cloud.size();
  if (cloud_size == 0)
    return;
  std::uniform_int_distribution<int> ID_dis(0, cloud_size - 1);
  std::vector<bool> inlierbools(cloud_size, false);
  for (int line_num = 0; line_num < max_line_num; line_num++) {
    int best_inlier_count = 0;
    std::vector<int> best_inlierIDs;
    for (int i = 0; i < max_iterations; i++) {
      int guess_1 = ID_dis(gen);
      int guess_2 = ID_dis(gen);
      while (guess_1 == guess_2 || inlierbools[guess_1] == true || inlierbools[guess_2] == true) {
        guess_1 = ID_dis(gen);
        guess_2 = ID_dis(gen);
      }
      ransac_line guess_line(target_cloud[guess_1], target_cloud[guess_2]);
      threads.push_back(std::thread([this, guess_line, threshold, cloud_size, &target_cloud, &inlierbools, &best_inlier_count, &best_inlierIDs]() {
        std::vector<int> inlierIDs;
        int inlier_count = 0;
        for (int pointID = 0; pointID < cloud_size; pointID++) {
          if (guess_line.distance_to(target_cloud[pointID]) < threshold && inlierbools[pointID] != true) {
            inlierIDs.push_back(pointID);
            inlier_count++;
          }
        }
        std::lock_guard<std::mutex> lock(mtx);
        if (inlier_count > best_inlier_count && inlier_count > cloud_size / 8.0) {
          best_inlier_count = inlier_count;
          best_inlierIDs = inlierIDs;
        }
      }));
    }
    for (std::thread &t : threads)
      t.join();
    threads.clear();
    for (int i = 0; i < best_inlier_count; i++) {
      inlierbools[best_inlierIDs[i]] = true;
    }
  }
  std::vector<Eigen::Vector2d> clean_cloud;
  for (int i = 0; i < cloud_size; i++) {
    if (inlierbools[i]) {
      clean_cloud.push_back(target_cloud[i]);
    }
  }
  target_cloud.clear();
  target_cloud = clean_cloud;
}

Eigen::Vector3d ICPNode::do_icp(std::vector<Eigen::Vector2d> &point_cloud, std::vector<LineSeg> &line_segments) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> num_dis(-0.25, 0.25);
  std::uniform_real_distribution<double> angle_dis(-M_PI, M_PI);

  Eigen::Vector3d guess_par;
  double min_cost = std::numeric_limits<double>::max();
  for (int i = 0; i < 10; i++) {
    Eigen::Vector3d guess_guess_par(pre_pos_error.x() + num_dis(gen), pre_pos_error.y() + num_dis(gen), 0.0);  // 初期値
    threads.push_back(std::thread([this, &guess_guess_par, &point_cloud, &line_segments, &guess_par, &min_cost]() { this->ICP(guess_guess_par, point_cloud, line_segments, guess_par, min_cost); }));
  }
  for (std::thread &t : threads)
    t.join();
  threads.clear();

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

  double cost = min_R.array().abs().sum();  // 多少のハズレ値を無視できるようにしたいので二乗しない
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

sensor_msgs::msg::PointCloud2 ICPNode::Eigen_to_cloud(const std::vector<Eigen::Vector2d> &points, const std_msgs::msg::Header &header) {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header = header;
  cloud_msg.height = 1;
  cloud_msg.width = points.size();
  cloud_msg.is_dense = true;
  cloud_msg.is_bigendian = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  for (const auto &pt : points) {
    *iter_x = pt.x();
    *iter_y = pt.y();
    *iter_z = 0.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return cloud_msg;
}

void ICPNode::timer_callback() {
  // 初期位置の設定
  // RCLCPP_INFO(this->get_logger(), "Robot Pos x: %f y: %f yaw: %f", new_robot_pos.x(), new_robot_pos.y(), new_robot_pos.z());
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
  std_msgs::msg::Float64 msg;
  msg.data = new_robot_pos.z();
  robot_yaw_pub_->publish(msg);
}

void ICPNode::visualize_line_segments(std::vector<LineSeg> &line_segments) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = cloud_header_.stamp;
  marker.header.frame_id = map_frame_id;
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