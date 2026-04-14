#ifndef DRAW_HPP
#define DRAW_HPP

#include "../include/armor.hpp"
#include "aimer.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace armor_task;

namespace tools
{
void draw_box(cv::Mat &img, const cv::Rect &box, const cv::Scalar &color, int thickness = 2);
void draw_point(cv::Mat &img, const cv::Point &point, const cv::Scalar &color, int radius = 2);
void draw_points(cv::Mat &img, const std::vector<cv::Point> &points, const cv::Scalar &color = cv::Scalar(0, 0, 255), int thickness = 2);
void draw_points(cv::Mat &img, const std::vector<cv::Point2f> &points, const cv::Scalar &color = cv::Scalar(0, 0, 255), int thickness = 2);
void draw_text(cv::Mat &img, const std::string &text, const cv::Point &point, const cv::Scalar &color, double font_scale = 0.5, int thickness = 1);
}

void drawTrajectory(cv::Mat &img, const Eigen::Vector3d &target_pos, double bullet_speed, const std::string &config_path);
void drawTrajectory(cv::Mat &img, const AimPoint &aim_point, double bullet_speed, const std::string &config_path, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs,
                    const Eigen::Matrix3d &R_gimbal2world);
void drawTrajectory(cv::Mat &img, double yaw_gimbal, double pitch_gimbal, double bullet_speed, const std::string &config_path, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs);

#endif // DRAW_HPP
