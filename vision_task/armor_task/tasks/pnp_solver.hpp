#pragma once

#include "armor.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// camera x -> world -y, camera y -> world -z, camera z -> world x

namespace armor_task
{
class Target;
struct AimPoint;

class PnpSolver
{
  public:
    PnpSolver(const std::string &config_path);

    // only debug use
    PnpSolver(const cv::Mat &camera_matrix, const cv::Mat &distortion_coeffs);
    ~PnpSolver() = default;

    bool _solve_pnp(Armor &armor);

    bool Islarge(Armor &armor);

    int solveArmorArray(ArmorArray &armor_array);

    void set_R_gimbal2world(const Eigen::Quaterniond &q);

    Eigen::Matrix3d R_gimbal2imubody_;
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Vector3d t_camera2gimbal_;
    Eigen::Matrix3d R_gimbal2world_;

    void optimize_yaw(Armor &armor) const;

    double armor_reprojection_error(const Armor &armor, double yaw, const double &inclined) const;

    std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d &p_world, double yaw, bool islarge) const;
    std::vector<std::vector<cv::Point2f>> reproject_armor(const Target &target) const;
    std::vector<cv::Point2f> reproject_armor(const AimPoint &aim_point, bool islarge = false) const;
    double SJTU_cost(const std::vector<cv::Point2f> &cv_refs, const std::vector<cv::Point2f> &cv_pts, const double &inclined) const;

    const cv::Mat &camera_matrix() const { return camera_matrix_; }
    const cv::Mat &distort_coeffs() const { return distort_coeffs_; }

  private:
    cv::Mat camera_matrix_;
    cv::Mat distort_coeffs_;
};

}  // namespace armor_task
