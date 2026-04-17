#include "draw.hpp"

#include "../tasks/trajectory_normal.hpp"
#include "parser.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace tools
{

// 传入四个角点，画倾斜装甲板轮廓和中轴线。
void draw_box(cv::Mat &img, const std::vector<cv::Point2f> &corners, const cv::Scalar &color, int thickness)
{
    if (corners.size() != 4)
    {
        return;
    }

    draw_point(img, corners, color, thickness);

    // 在倾斜框中画一条“左右中心连线”，更直观看出朝向与倾斜程度
    cv::Point2f left_center = 0.5f * (corners[0] + corners[3]);
    cv::Point2f right_center = 0.5f * (corners[1] + corners[2]);
    cv::line(img, left_center, right_center, color, std::max(1, thickness - 1), cv::LINE_AA);
}

// 传入 Armor，优先使用 armor.corners 画倾斜框。
void draw_box(cv::Mat &img, const Armor &armor, const cv::Scalar &color, int thickness)
{
    if (armor.corners.size() == 4)
    {
        draw_box(img, armor.corners, color, thickness);
    }
}

// 传入 Target，重投影其全部装甲板并逐个画倾斜框。
void draw_box(cv::Mat &img, const Target &target, const armor_task::PnpSolver &pnp_solver, const cv::Scalar &color, int thickness)
{
    const auto all_corners = pnp_solver.reproject_armor(target);
    for (const auto &corners : all_corners)
    {
        if (corners.size() < 4)
        {
            continue;
        }
        draw_box(img, corners, color, thickness);
    }
}

// 传入 AimPoint，重投影后画该瞄准装甲板的倾斜框。
void draw_box(cv::Mat &img, const AimPoint &aim_point, const armor_task::PnpSolver &pnp_solver, const cv::Scalar &color, int thickness, bool is_large)
{
    const auto corners = pnp_solver.reproject_armor(aim_point, is_large);
    if (corners.size() < 4)
    {
        return;
    }
    draw_box(img, corners, color, thickness);
}

// 传入单个像素点，画实心圆点。
void draw_point(cv::Mat &img, const cv::Point &point, const cv::Scalar &color, int radius)
{
    cv::circle(img, point, radius, color, -1);
}

// 传入 AimPoint，重投影后画其中心点。
void draw_point(cv::Mat &img, const AimPoint &aim_point, const armor_task::PnpSolver &pnp_solver, const cv::Scalar &color, int radius, bool is_large)
{
    const auto corners = pnp_solver.reproject_armor(aim_point, is_large);
    if (corners.empty())
    {
        return;
    }
    cv::Point2f sum(0.0F, 0.0F);
    for (const auto &pt : corners)
    {
        sum += pt;
    }
    const cv::Point2f center = sum * (1.0F / static_cast<float>(corners.size()));
    draw_point(img, cv::Point(cvRound(center.x), cvRound(center.y)), color, radius);
}

// 传入整数点集，单点画圆，多点按闭合折线连接绘制。
void draw_point(cv::Mat &img, const std::vector<cv::Point> &points, const cv::Scalar &color, int thickness)
{
    if (points.empty())
    {
        return;
    }
    if (points.size() == 1)
    {
        draw_point(img, points.front(), color, std::max(1, thickness));
        return;
    }
    cv::polylines(img, points, true, color, thickness, cv::LINE_AA);
}

// 传入浮点点集，转换到像素点后按闭合折线绘制。
void draw_point(cv::Mat &img, const std::vector<cv::Point2f> &points, const cv::Scalar &color, int thickness)
{
    std::vector<cv::Point> int_points;
    int_points.reserve(points.size());
    for (const auto &pt : points)
    {
        int_points.emplace_back(cvRound(pt.x), cvRound(pt.y));
    }
    draw_point(img, int_points, color, thickness);
}

// 传入文本和锚点位置，在图像上绘制文字。
void draw_text(cv::Mat &img, const std::string &text, const cv::Point &point, const cv::Scalar &color, double font_scale, int thickness)
{
    cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness, cv::LINE_AA);
}

} // namespace tools

namespace
{

constexpr double kGravity = 9.7833;
constexpr int kTrajectorySamples = 50;
constexpr double kMaxDistanceByAngles = 10.0;
const Eigen::Vector3d kMuzzleInCamera(0.1, 0.1, 0.5);
const cv::Scalar kTrajectoryColor(0, 255, 255);

bool pointInImage(const cv::Mat &img, const cv::Point2f &pt)
{
    return pt.x >= 0 && pt.x < img.cols && pt.y >= 0 && pt.y < img.rows;
}

void loadTransformMatrices(const std::string &config_path, Eigen::Matrix3d &R_camera2gimbal, Eigen::Vector3d &t_camera2gimbal)
{
    R_camera2gimbal = Eigen::Matrix3d::Identity();
    t_camera2gimbal = Eigen::Vector3d::Zero();

    try
    {
        const YAML::Node yaml = YAML::LoadFile(config_path);
        if (yaml["R_camera2gimbal"] && yaml["t_camera2gimbal"])
        {
            const auto r = yaml["R_camera2gimbal"].as<std::vector<double>>();
            const auto t = yaml["t_camera2gimbal"].as<std::vector<double>>();
            if (r.size() == 9 && t.size() == 3)
            {
                R_camera2gimbal = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(r.data());
                t_camera2gimbal = Eigen::Vector3d(t[0], t[1], t[2]);
            }
        }
    }
    catch (...)
    {
        // Keep identity transform when config is unavailable.
    }
}

std::vector<Eigen::Vector3d> ballisticByTarget(const Eigen::Vector3d &start_world, const Eigen::Vector3d &target_world, double bullet_speed)
{
    std::vector<Eigen::Vector3d> points;
    if (bullet_speed <= 0.0)
    {
        return points;
    }

    const Eigen::Vector3d diff = target_world - start_world;
    const double d = std::hypot(diff.x(), diff.y());
    const double h = diff.z();

    armor_task::Trajectory traj(bullet_speed, d, h);
    if (traj.unsolvable)
    {
        return points;
    }

    const double yaw = std::atan2(diff.y(), diff.x());
    const double vx = bullet_speed * std::cos(traj.pitch) * std::cos(yaw);
    const double vy = bullet_speed * std::cos(traj.pitch) * std::sin(yaw);
    const double vz = bullet_speed * std::sin(traj.pitch);

    points.reserve(kTrajectorySamples + 1);
    const double dt = traj.fly_time / static_cast<double>(kTrajectorySamples);
    for (int i = 0; i <= kTrajectorySamples; ++i)
    {
        const double t = std::min(traj.fly_time, i * dt);
        points.emplace_back(start_world.x() + vx * t, start_world.y() + vy * t, start_world.z() + vz * t - 0.5 * kGravity * t * t);
    }
    return points;
}

std::vector<Eigen::Vector3d> ballisticByAngles(const Eigen::Vector3d &start_world, const Eigen::Matrix3d &R_gimbal2world, double yaw_gimbal, double pitch_gimbal, double bullet_speed)
{
    std::vector<Eigen::Vector3d> points;
    if (bullet_speed <= 0.0 || !std::isfinite(yaw_gimbal) || !std::isfinite(pitch_gimbal))
    {
        return points;
    }

    const double cp = std::cos(pitch_gimbal);
    const double sp = std::sin(pitch_gimbal);
    const double cy = std::cos(yaw_gimbal);
    const double sy = std::sin(yaw_gimbal);

    Eigen::Vector3d dir_gimbal(cp * cy, cp * sy, sp);
    const Eigen::Vector3d v0_world = R_gimbal2world * (dir_gimbal * bullet_speed);

    const double horizontal_speed = std::max(1e-3, std::hypot(v0_world.x(), v0_world.y()));
    const double max_fly_time = kMaxDistanceByAngles / horizontal_speed;
    const double dt = max_fly_time / static_cast<double>(kTrajectorySamples);

    points.reserve(kTrajectorySamples + 1);
    for (int i = 0; i <= kTrajectorySamples; ++i)
    {
        const double t = i * dt;
        points.emplace_back(start_world.x() + v0_world.x() * t,
                            start_world.y() + v0_world.y() * t,
                            start_world.z() + v0_world.z() * t - 0.5 * kGravity * t * t);
    }
    return points;
}

std::vector<cv::Point2f> projectWorldToImage(const std::vector<Eigen::Vector3d> &world_points,
                                             const Eigen::Matrix3d &R_camera2gimbal,
                                             const Eigen::Vector3d &t_camera2gimbal,
                                             const Eigen::Matrix3d &R_gimbal2world,
                                             const cv::Mat &camera_matrix,
                                             const cv::Mat &distort_coeffs)
{
    std::vector<cv::Point3f> camera_points;
    camera_points.reserve(world_points.size());

    for (const auto &pt_world : world_points)
    {
        const Eigen::Vector3d pt_camera = R_camera2gimbal.transpose() * (R_gimbal2world.transpose() * pt_world - t_camera2gimbal);
        if (pt_camera.z() > 0.05)
        {
            camera_points.emplace_back(static_cast<float>(pt_camera.x()), static_cast<float>(pt_camera.y()), static_cast<float>(pt_camera.z()));
        }
    }

    std::vector<cv::Point2f> image_points;
    if (camera_points.empty())
    {
        return image_points;
    }

    cv::projectPoints(camera_points, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camera_matrix, distort_coeffs, image_points);
    return image_points;
}

void drawProjectedTrajectory(cv::Mat &img, const std::vector<cv::Point2f> &image_points)
{
    if (image_points.size() < 2)
    {
        return;
    }

    for (size_t i = 0; i + 1 < image_points.size(); ++i)
    {
        const auto &p1 = image_points[i];
        const auto &p2 = image_points[i + 1];
        if (pointInImage(img, p1) && pointInImage(img, p2))
        {
            cv::line(img, p1, p2, kTrajectoryColor, 2, cv::LINE_AA);
        }
    }

    const auto &end = image_points.back();
    if (pointInImage(img, end))
    {
        cv::circle(img, end, 4, cv::Scalar(0, 0, 255), -1);
    }
}

} // namespace

namespace tools
{

// 传入目标三维点，画弹道轨迹投影线。
void drawTrajectory(cv::Mat &img, const Eigen::Vector3d &target_pos, double bullet_speed, const std::string &config_path)
{
    const auto camera_params = loadCameraParameters(config_path);
    const cv::Mat &camera_matrix = camera_params.first;
    const cv::Mat &distort_coeffs = camera_params.second;

    Eigen::Matrix3d R_camera2gimbal;
    Eigen::Vector3d t_camera2gimbal;
    loadTransformMatrices(config_path, R_camera2gimbal, t_camera2gimbal);

    const Eigen::Matrix3d R_gimbal2world = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d start_world = R_gimbal2world * (R_camera2gimbal * kMuzzleInCamera + t_camera2gimbal);

    const auto world_points = ballisticByTarget(start_world, target_pos, bullet_speed);
    const auto image_points = projectWorldToImage(world_points, R_camera2gimbal, t_camera2gimbal, R_gimbal2world, camera_matrix, distort_coeffs);
    drawProjectedTrajectory(img, image_points);
}

// 传入 AimPoint，画目标对应的弹道轨迹投影线。
void drawTrajectory(cv::Mat &img,
                    const AimPoint &aim_point,
                    double bullet_speed,
                    const std::string &config_path,
                    const cv::Mat &camera_matrix,
                    const cv::Mat &distort_coeffs,
                    const Eigen::Matrix3d &R_gimbal2world)
{
    if (!aim_point.valid)
    {
        return;
    }

    Eigen::Matrix3d R_camera2gimbal;
    Eigen::Vector3d t_camera2gimbal;
    loadTransformMatrices(config_path, R_camera2gimbal, t_camera2gimbal);

    const Eigen::Vector3d start_world = R_gimbal2world * (R_camera2gimbal * kMuzzleInCamera + t_camera2gimbal);
    const Eigen::Vector3d target_world = aim_point.xyza.head<3>();

    const auto world_points = ballisticByTarget(start_world, target_world, bullet_speed);
    const auto image_points = projectWorldToImage(world_points, R_camera2gimbal, t_camera2gimbal, R_gimbal2world, camera_matrix, distort_coeffs);
    drawProjectedTrajectory(img, image_points);
}

// 传入 yaw/pitch，按指定射角画弹道轨迹投影线。
void drawTrajectory(cv::Mat &img,
                    double yaw_gimbal,
                    double pitch_gimbal,
                    double bullet_speed,
                    const std::string &config_path,
                    const cv::Mat &camera_matrix,
                    const cv::Mat &distort_coeffs)
{
    Eigen::Matrix3d R_camera2gimbal;
    Eigen::Vector3d t_camera2gimbal;
    loadTransformMatrices(config_path, R_camera2gimbal, t_camera2gimbal);

    const Eigen::Matrix3d R_gimbal2world = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d start_world = R_gimbal2world * (R_camera2gimbal * kMuzzleInCamera + t_camera2gimbal);

    const auto world_points = ballisticByAngles(start_world, R_gimbal2world, yaw_gimbal, pitch_gimbal, bullet_speed);
    const auto image_points = projectWorldToImage(world_points, R_camera2gimbal, t_camera2gimbal, R_gimbal2world, camera_matrix, distort_coeffs);
    drawProjectedTrajectory(img, image_points);
}

} // namespace tools
