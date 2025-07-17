#include "set_goal.hpp"

namespace set_goal {

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

SetGoal::SetGoal(const rclcpp::NodeOptions& options) : rclcpp::Node("set_goal_node", options) {
  this->declare_parameter<std::string>("robot_frame_id", "robot_base");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("map_frame_id", "map");

  robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  map_frame_id_ = this->get_parameter("map_frame_id").as_string();

  subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&SetGoal::topic_callback, this, std::placeholders::_1));
  goal_pos_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(&SetGoal::goal_pos_callback, this, std::placeholders::_1));
  wall_dis_pub_ = this->create_subscription<std_msgs::msg::Float64>("distance_to_wall", 10, std::bind(&SetGoal::dis_callback, this, std::placeholders::_1));
  wall_norm_pub_ = this->create_subscription<geometry_msgs::msg::Vector3>("norm_v_of_wall", 10, std::bind(&SetGoal::dir_callback, this, std::placeholders::_1));
  move_nums_pub_ = this->create_publisher<interface::msg::MoveMsg>("move_par", 10);
  marker_print = this->create_publisher<visualization_msgs::msg::Marker>("move_marker", 10);
  timer_1 = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SetGoal::send_goal, this));
  timer_2 = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SetGoal::get_tf, this));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void SetGoal::dis_callback(const std_msgs::msg::Float64& msg) {
  nearest_wall_distance = msg.data;
}

void SetGoal::dir_callback(const geometry_msgs::msg::Vector3& msg) {
  nearest_wall_norm.x() = msg.x;
  nearest_wall_norm.y() = msg.y;
  nearest_wall_norm = -nearest_wall_norm.normalized();
}

void SetGoal::topic_callback(const sensor_msgs::msg::Joy& msg) {
  if (msg.buttons[0])
    auto_mode = false;
  else if (msg.buttons[1])
    auto_mode = true;

  target_dir = normalize_angle(atan2(msg.axes[0], msg.axes[1]));
  target_power = sqrt(msg.axes[0] * msg.axes[0] + msg.axes[1] * msg.axes[1]);
  target_roat = msg.axes[3];
}

void SetGoal::goal_pos_callback(const geometry_msgs::msg::PoseStamped& msg) {
  map_to_goal = Eigen::Vector2d(msg.pose.position.x, msg.pose.position.y);
  tf2::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  goal_yaw = yaw;
}

void SetGoal::send_goal() {
  if (auto_mode) {
    Eigen::Vector2d robot_to_goal = getR(-robot_yaw) * (map_to_goal - map_to_robot);
    double goal_dis = robot_to_goal.norm();
    double x_dis = robot_to_goal.x();
    double y_dis = robot_to_goal.y();
    //  角速度のPD (角度の差)
    double angle_error = normalize_angle(goal_yaw - robot_yaw);

    double deriv_angle = (angle_error - pre_angle_error) / 0.1;
    double output_angle = angle_error * ang_gain.P + deriv_angle * ang_gain.D;
    pre_angle_error = angle_error;

    target_roat = output_angle;

    // 進行方向のPD (x,y各軸のゴールまでの距離)
    // x
    double deriv_x = (x_dis - pre_x_dis) / 0.1;
    double output_x = x_dis * dir_gain.P + deriv_x * dir_gain.D;
    pre_x_dis = x_dis;
    // y
    double deriv_y = (y_dis - pre_y_dis) / 0.1;
    double output_y = y_dis * dir_gain.P + deriv_y * dir_gain.D;
    pre_y_dis = y_dis;

    target_dir = atan2(output_y, output_x);

    // 速度のPD (ゴールまでの距離から)
    double deriv_vel = (goal_dis - pre_goal_dis) / 0.1;
    double output_vel = vel_gain.P * goal_dis + vel_gain.D * deriv_vel;
    pre_goal_dis = goal_dis;
    // target_power = std::min(nearest_wall_distance, 1.0);
    target_power = std::min(output_vel, 1.0);
  }
  interface::msg::MoveMsg send_data;
  Eigen::Vector2d direction, output_v(std::cos(target_dir), std::sin(target_dir));
  if (nearest_wall_distance < 2.0) {
    direction = (output_v + nearest_wall_norm * (1.0 - nearest_wall_distance / 2.0) * 0.65).normalized();
  } else {
    direction = output_v;
  }
  target_dir = atan2(direction.y(), direction.x());
  send_data.direction = target_dir;
  send_data.velocity = target_power;
  // send_data.velocity = 0;
  send_data.angular_v = target_roat;
  move_nums_pub_->publish(send_data);
}

void SetGoal::get_tf() {
  try {
    robot_stramp = tf_buffer_->lookupTransform(map_frame_id_, robot_frame_id_, tf2::TimePointZero);

    auto q = robot_stramp.transform.rotation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    robot_yaw = yaw;

    Eigen::Vector2d translation(
        robot_stramp.transform.translation.x,
        robot_stramp.transform.translation.y);
    map_to_robot = translation;

  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
  }
}

}  // namespace set_goal

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(set_goal::SetGoal)