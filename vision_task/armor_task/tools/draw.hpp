#pragma once

#include "../include/armor.hpp"
#include "../tasks/pnp_solver.hpp"
#include "aimer.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace armor_task;

namespace tools
{
// 传入四个角点，画倾斜装甲板轮廓和中轴线。
void draw_box(cv::Mat &img, const std::vector<cv::Point2f> &corners, const cv::Scalar &color, int thickness = 2);
// 传入 Armor，优先使用 armor.corners 画倾斜框。
void draw_box(cv::Mat &img, const Armor &armor, const cv::Scalar &color, int thickness = 2);
// 传入 Target，重投影其全部装甲板并逐个画倾斜框。
void draw_box(cv::Mat &img, const Target &target, const armor_task::PnpSolver &pnp_solver, const cv::Scalar &color, int thickness = 2);
// 传入 AimPoint，重投影后画该瞄准装甲板的倾斜框。
void draw_box(cv::Mat &img, const AimPoint &aim_point, const armor_task::PnpSolver &pnp_solver, const cv::Scalar &color, int thickness = 2, bool is_large = false);

// 传入单个像素点，画实心圆点。
void draw_point(cv::Mat &img, const cv::Point &point, const cv::Scalar &color, int radius = 2);
// 传入 AimPoint，重投影后画其中心点。
void draw_point(cv::Mat &img, const AimPoint &aim_point, const armor_task::PnpSolver &pnp_solver, const cv::Scalar &color, int radius = 2, bool is_large = false);
// 传入整数点集，单点画圆，多点按闭合折线连接绘制。
void draw_point(cv::Mat &img, const std::vector<cv::Point> &points, const cv::Scalar &color = cv::Scalar(0, 0, 255), int thickness = 2);
// 传入浮点点集，转换到像素点后按闭合折线绘制。
void draw_point(cv::Mat &img, const std::vector<cv::Point2f> &points, const cv::Scalar &color = cv::Scalar(0, 0, 255), int thickness = 2);

// 传入文本和锚点位置，在图像上绘制文字。
void draw_text(cv::Mat &img, const std::string &text, const cv::Point &point, const cv::Scalar &color, double font_scale = 0.5, int thickness = 1);
// 传入目标三维点，画弹道轨迹投影线。
void drawTrajectory(cv::Mat &img, const Eigen::Vector3d &target_pos, double bullet_speed, const std::string &config_path);
// 传入 AimPoint，画目标对应的弹道轨迹投影线。
void drawTrajectory(cv::Mat &img, const AimPoint &aim_point, double bullet_speed, const std::string &config_path, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs,
                    const Eigen::Matrix3d &R_gimbal2world);
// 传入 yaw/pitch，按指定射角画弹道轨迹投影线。
void drawTrajectory(cv::Mat &img, double yaw_gimbal, double pitch_gimbal, double bullet_speed, const std::string &config_path, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs);
}
