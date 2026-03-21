#ifndef DRAW_HPP
#define DRAW_HPP

#include "../include/armor.hpp"
#include "aimer.hpp"
#include "pnp_solver.hpp"
#include "tracker.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace armor_task;

namespace tools
{
// 基础绘图工具函数
void draw_point(cv::Mat &img, const cv::Point &point, const cv::Scalar &color, int radius = 2);
void draw_points(cv::Mat &img, const std::vector<cv::Point> &points, const cv::Scalar &color = cv::Scalar(0, 0, 255), int thickness = 2);
void draw_points(cv::Mat &img, const std::vector<cv::Point2f> &points, const cv::Scalar &color = cv::Scalar(0, 0, 255), int thickness = 2);
void draw_text(cv::Mat &img, const std::string &text, const cv::Point &point, const cv::Scalar &color, double font_scale = 0.5, int thickness = 1);

// 辅助函数
std::vector<cv::Point2f> project3DPointsTo2D(const std::vector<cv::Point3f> &points_3d, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs);
void draw_info_box(cv::Mat &image, const std::string &text, int font_scale = 1, int thickness = 2);
void plotArmors(const ArmorArray &armors, cv::Mat &img);
void plotSingleArmor(const Armor &target, const Armor &armor, cv::Mat &img);
} // namespace tools

// 主要绘图函数
void drawArmorDetection(cv::Mat &img, const ArmorArray &armors);
void drawTargetInfo(cv::Mat &img, const std::vector<Target> &targets, const std::string &tracker_state, const PnpSolver &pnp_solver);
void drawPerformanceInfo(cv::Mat &img, double fps, double detect_time, double track_time);
void drawPerformanceInfo(cv::Mat &img, double fps, double detect_time, double pnp_time, int success_count);
void drawPnPresult(cv::Mat &img, const ArmorArray &armors);
void drawTrajectory(cv::Mat &img, const Eigen::Vector3d &target_pos, double bullet_speed, const std::string &config_path);
void drawTrajectory(cv::Mat &img, const AimPoint &aim_point, double bullet_speed, const std::string &config_path, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs,
                    const Eigen::Matrix3d &R_gimbal2world);
void drawTrajectory(cv::Mat &img, double yaw_gimbal, double pitch_gimbal, double bullet_speed, const std::string &config_path, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs);

#endif // DRAW_HPP
