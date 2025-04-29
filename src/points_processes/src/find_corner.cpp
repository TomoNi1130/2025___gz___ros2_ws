#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Vector3.hpp"

#include <random>
#include <mutex>
#include <thread>
#include <Eigen/Dense>
#include <algorithm>
#include <utility>
#include <vector>
#include <optional>

// 線分から角を割り出す

using namespace std::chrono_literals;

struct Line
{
    Eigen::Vector2d pos, dir;
    Line() : pos(Eigen::Vector2d(0, 0)), dir(Eigen::Vector2d(1, 0)) {}
    Line(const Eigen::Vector2d &position, const Eigen::Vector2d &direction) : pos(position), dir(direction) {}

    static Line from_points(Eigen::Vector2d a, Eigen::Vector2d b) { return Line{a, (b - a).normalized()}; }
    double distance_to(Eigen::Vector2d point) const
    {
        Eigen::Vector2d v = point - pos;
        double cross = dir.x() * v.y() - dir.y() * v.x();
        return std::abs(cross);
    }
    double angle_to(const Line &other) const
    {
        return std::acos(dir.dot(other.dir));
    }
};

struct LineSeg
{
    Line line;
    double length;
    LineSeg() : line(Line(Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 0))), length(0.0) {}
    LineSeg(Line &line) : line(line), length(0.0) {}
    void return_points(geometry_msgs::msg::Point &start_p, geometry_msgs::msg::Point &end_p) const
    {
        start_p.x = line.pos.x();
        start_p.y = line.pos.y();
        start_p.z = 0.0;
        double angle = atan2(line.dir.y(), line.dir.x());
        end_p.x = line.pos.x() + length * cos(angle);
        end_p.y = line.pos.y() + length * sin(angle);
        end_p.z = 0.0;
    }
};

struct corner
{
    Eigen::Vector2d robot_look_pos;
    double distance_to_robot;
    corner() : robot_look_pos(Eigen::Vector2d(0, 0)), distance_to_robot(0.0) {}
    corner(const Eigen::Vector2d &position) : robot_look_pos(position), distance_to_robot(0.0) {}
    corner(const Eigen::Vector2d &position, const double distance) : robot_look_pos(position), distance_to_robot(distance) {}
};

struct general_line
{
    double a, b, c;
    general_line(double a, double b, double c) : a(a), b(b), c(c) {}
    general_line to_general_line(const Line &line)
    {
        double A = line.dir.y();
        double B = -line.dir.x();
        double C = line.dir.y() * line.pos.x() - line.dir.x() * line.pos.y();
        return {A, B, C};
    }
};

class RANSAC
{
public:
    RANSAC(const double threshold, const double gap_threshold, rclcpp::Logger logger) : max_iterations(200), distance_threshold(threshold), gap_threshold(gap_threshold), logger(logger) {}
    RANSAC(const int iteration, const double threshold, const double gap_threshold, rclcpp::Logger logger) : max_iterations(iteration), distance_threshold(threshold), gap_threshold(gap_threshold), logger(logger) {}

    std::optional<std::vector<LineSeg>> do_ransac(const sensor_msgs::msg::PointCloud &msg_points, const int max_line_num)
    {
        std::vector<LineSeg> return_line_segments;
        set_points(msg_points);
        std::random_device rd;
        std::mt19937 gen(rd());
        if (points_cloud.size() != 0)
            for (int line_num = 0; line_num < max_line_num; line_num++)
            {
                LineSeg return_line_seg;
                int best_inlier_count = 0;
                std::vector<bool> guess_inliers(points_cloud.size(), false);
                for (int j = 0; j < max_iterations; j++)
                {
                    std::uniform_int_distribution<int> dis(0, points_cloud.size() - 1);
                    int guess_1 = dis(gen);
                    int guess_2 = dis(gen);
                    while (guess_1 == guess_2)
                    {
                        guess_2 = dis(gen);
                    }
                    Line guess_line = guess_line.from_points(points_cloud[guess_1].point, points_cloud[guess_2].point);
                    LineSeg guess_line_seg(guess_line);
                    // scrutinize_guess_line_seg(guess_line_seg, best_inlier_count, return_line_seg, guess_inliers);
                    threads.push_back(std::thread([this, guess_line_seg, &best_inlier_count, &return_line_seg, &guess_inliers]()
                                                  { this->scrutinize_guess_line_seg(guess_line_seg, best_inlier_count, return_line_seg, guess_inliers); }));
                }
                for (std::thread &t : threads)
                {
                    t.join();
                }
                threads.clear();
                if (best_inlier_count > points_cloud.size() / 256)
                {
                    for (int i = 0; i < points_cloud.size(); i++)
                    {
                        if (guess_inliers[i])
                        {
                            points_cloud[i].is_inlier = true;
                        }
                    }
                    return_line_segments.push_back(return_line_seg);
                }
                else
                {
                    return return_line_segments;
                }
            }
        return return_line_segments;
    }

private:
    struct ransac_point
    {
        Eigen::Vector2d point;
        bool is_inlier;
        ransac_point() : is_inlier(false)
        {
            point.x() = 0.0;
            point.y() = 0.0;
        }
        ransac_point(Eigen::Vector2d point) : point(point), is_inlier(false) {}
    };
    void set_points(const sensor_msgs::msg::PointCloud &target_points)
    {
        points_cloud.clear();
        for (int i = 0; i < target_points.points.size(); i++)
            points_cloud.push_back(ransac_point(Eigen::Vector2d(target_points.points[i].x, target_points.points[i].y)));
    }
    void scrutinize_guess_line_seg(LineSeg guess_line_seg, int &best_inlier_count, LineSeg &return_line_seg, std::vector<bool> &guess_inliers)
    {
        struct DistanceAndIDs
        {
            double distance;
            int id;
            DistanceAndIDs(double distance, int id) : distance(distance), id(id) {}
            DistanceAndIDs() : distance(0.0), id(-1) {}
        };
        std::vector<bool> guess_guess_inliers(points_cloud.size(), false);
        std::vector<bool> line_seg_inliers(points_cloud.size(), false);
        int guess_inlier_count = 0;
        int line_seg_inlier_count = 0;
        for (int i = 0; i < points_cloud.size(); i++)
        {
            if (!points_cloud[i].is_inlier && guess_line_seg.line.distance_to(points_cloud[i].point) < distance_threshold)
            {
                guess_guess_inliers[i] = true;
                guess_inlier_count++;
            }
        }
        if (guess_inlier_count != 0) // 長さを出す（それに付随して、向き、初期位置の変更もある）
        {
            // 最も遠い点を出す
            std::vector<DistanceAndIDs> distance_and_ids;
            for (int i = 0; i < guess_guess_inliers.size(); i++)
                if (guess_guess_inliers[i])
                    distance_and_ids.push_back(DistanceAndIDs((points_cloud[i].point - guess_line_seg.line.pos).dot(guess_line_seg.line.dir), i));
            std::sort(distance_and_ids.begin(), distance_and_ids.end(), [](DistanceAndIDs &a, DistanceAndIDs &b)
                      { return a.distance > b.distance; }); // 降順
            bool gap = false;
            for (int i = 0; i < distance_and_ids.size() - 1; i++)
            {
                if (abs(distance_and_ids[i + 1].distance - distance_and_ids[i].distance) > gap_threshold)
                {
                    guess_line_seg.line.pos = points_cloud[distance_and_ids[i].id].point;
                    gap = true;
                    break;
                }
                else
                {
                    line_seg_inliers[distance_and_ids[i].id] = true;
                    line_seg_inlier_count++;
                }
            }
            if (!gap)
                guess_line_seg.line.pos = points_cloud[distance_and_ids[0].id].point; // 最も遠い点を線の開始点にする

            // 最も遠い点から最も遠い点を探す
            std::vector<DistanceAndIDs> distance_and_ids_from_termination_point;
            for (int i = 0; i < line_seg_inliers.size(); i++)
                if (line_seg_inliers[i])
                    distance_and_ids_from_termination_point.push_back(DistanceAndIDs((points_cloud[i].point - guess_line_seg.line.pos).norm(), i));
            if (distance_and_ids_from_termination_point.size() != 0)
            {
                std::sort(distance_and_ids_from_termination_point.begin(), distance_and_ids_from_termination_point.end(), [](DistanceAndIDs &a, DistanceAndIDs &b)
                          { return a.distance > b.distance; }); // 降順
                gap = false;
                for (int i = 0; i < distance_and_ids_from_termination_point.size() - 1; i++)
                {
                    if (abs(distance_and_ids_from_termination_point[i + 1].distance - distance_and_ids_from_termination_point[i].distance) > gap_threshold)
                    {
                        guess_line_seg.line.dir = points_cloud[distance_and_ids_from_termination_point[i].id].point - guess_line_seg.line.pos;
                        guess_line_seg.length = guess_line_seg.line.dir.norm();
                        gap = true;
                        break;
                    }
                }
                if (!gap)
                {
                    guess_line_seg.line.dir = points_cloud[distance_and_ids_from_termination_point[0].id].point - guess_line_seg.line.pos; // 向き
                    guess_line_seg.length = guess_line_seg.line.dir.norm();                                                                // 長さ
                }
            }
        }
        std::lock_guard<std::mutex> lock(mtx);
        if (guess_inlier_count > best_inlier_count)
        {
            best_inlier_count = line_seg_inlier_count;
            return_line_seg = guess_line_seg;
            guess_inliers = line_seg_inliers;
        }
    }
    const int max_iterations;
    const double distance_threshold = 0.1;
    const double gap_threshold = 1.0;
    std::vector<ransac_point> points_cloud;
    std::mutex mtx;
    std::vector<std::thread> threads;
    rclcpp::Logger logger;
};

class CornerFinder : public rclcpp::Node
{
public:
    CornerFinder()
        : Node("CornerFinder")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "lidar_points", 10, std::bind(&CornerFinder::topicCallback, this, std::placeholders::_1));
        corner_print = this->create_publisher<visualization_msgs::msg::Marker>("corners_position", 10);
        marker_print = this->create_publisher<visualization_msgs::msg::Marker>("lines_position", 10);
        RCLCPP_INFO(this->get_logger(), "CornerFinder setup!!");
    }

private:
    // 可視化系
    void visualize_corner(std::vector<corner> &corner_points, const std_msgs::msg::Header &header);
    void visualize_line_segments(std::vector<LineSeg> &lines, const std_msgs::msg::Header &header);

    // 処理系
    std::vector<corner> find_corner(std::vector<LineSeg> line_segments) // 線分の両端をコーナーとしていくつかのコーナーにまとめる。
    {
        std::vector<corner> known_corners;
        for (int i = 0; i < line_segments.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "a");
            geometry_msgs::msg::Point p1, p2;
            line_segments[i].return_points(p1, p2);
            Eigen::Vector2d p1_v(p1.x, p1.y), p2_v(p2.x, p2.y);
            bool new_corner_1 = true;
            bool new_corner_2 = true;
            RCLCPP_INFO(this->get_logger(), "b:%d", known_corners.size());
            for (int j = 0; j < known_corners.size(); j++)
            {
                // RCLCPP_INFO(this->get_logger(), "d");
                if ((known_corners[j].robot_look_pos - p1_v).norm() < 0.3)
                {
                    known_corners[j].robot_look_pos = (known_corners[j].robot_look_pos + p1_v) / 2.0;
                    known_corners[j].distance_to_robot = known_corners[j].distance_to_robot + p1_v.norm();
                    new_corner_1 = false;
                }
                // RCLCPP_INFO(this->get_logger(), "e");
                if ((known_corners[j].robot_look_pos - p2_v).norm() < 0.3)
                {
                    known_corners[j].robot_look_pos = (known_corners[j].robot_look_pos + p2_v) / 2.0;
                    known_corners[j].distance_to_robot = known_corners[j].distance_to_robot + p2_v.norm();
                    new_corner_2 = false;
                }
            }
            RCLCPP_INFO(this->get_logger(), "c");
            if (new_corner_1)
            {
                known_corners.push_back(corner(p1_v, p1_v.norm()));
            }
            if (new_corner_2)
            {
                known_corners.push_back(corner(p2_v, p2_v.norm()));
            }
        }
        RCLCPP_INFO(this->get_logger(), "%d", known_corners.size());
        return known_corners;
    }

    void topicCallback(const sensor_msgs::msg::PointCloud &msg)
    {
        std::optional<std::vector<LineSeg>> ransac_lines = ransac.do_ransac(msg, 5);
        if (ransac_lines.has_value())
        {
            visualize_line_segments(ransac_lines.value(), msg.header);
            std::vector<corner> corners = find_corner(ransac_lines.value());
            visualize_corner(corners, msg.header);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscription_;
    RANSAC ransac{0.02, 2.0, this->get_logger()};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr corner_print;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_print;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CornerFinder>());
    rclcpp::shutdown();
    return 0;
}

void CornerFinder::visualize_corner(std::vector<corner> &corner_points, const std_msgs::msg::Header &header)
{
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "corner_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = marker.scale.x;
    marker.scale.z = marker.scale.y;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    for (const auto &point : corner_points)
    {
        geometry_msgs::msg::Point print_point;
        print_point.x = point.robot_look_pos.x();
        print_point.y = point.robot_look_pos.y();
        print_point.z = 0.0;
        marker.points.push_back(print_point);
    }
    corner_print->publish(marker);
}

void CornerFinder::visualize_line_segments(std::vector<LineSeg> &lines, const std_msgs::msg::Header &header)
{
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "detected_lines";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.01;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    for (auto &line : lines)
    {
        geometry_msgs::msg::Point p1, p2;
        line.return_points(p1, p2);
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }
    marker_print->publish(marker);
}