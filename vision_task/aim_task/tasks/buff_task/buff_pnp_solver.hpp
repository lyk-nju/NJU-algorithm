#pragma once

#include "../armor_task/pnp_solver.hpp"
#include "buff_aimer.hpp"
#include "buff_target.hpp"

namespace buff_task
{
class BuffPnpSolver
{
  public:
    explicit BuffPnpSolver(const std::string &config_path)
        : solver_(config_path)
    {
    }

    BuffPnpSolver(const cv::Mat &camera_matrix, const cv::Mat &distortion_coeffs)
        : solver_(camera_matrix, distortion_coeffs)
    {
    }

    ~BuffPnpSolver() = default;

    bool solve_pnp(armor_task::Armor &armor) { return solver_.solve_pnp(armor); }
    bool is_large(armor_task::Armor &armor) { return solver_.is_large(armor); }
    int solveArmorArray(armor_task::ArmorArray &armor_array) { return solver_.solveArmorArray(armor_array); }
    void set_R_gimbal2world(const Eigen::Quaterniond &q) { solver_.set_R_gimbal2world(q); }
    void set_camera(const io::CameraInfo &info) { solver_.set_camera(info); }
    void optimize_yaw(armor_task::Armor &armor) const { solver_.optimize_yaw(armor); }

    double armor_reprojection_error(const armor_task::Armor &armor, double yaw, const double &inclined) const
    {
        return solver_.armor_reprojection_error(armor, yaw, inclined);
    }

    std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d &p_world, double yaw, bool is_large) const
    {
        return solver_.reproject_armor(p_world, yaw, is_large);
    }

    std::vector<std::vector<cv::Point2f>> reproject_armor(const BuffTarget &target) const
    {
        return solver_.reproject_armor(static_cast<const armor_task::Target &>(target));
    }

    std::vector<cv::Point2f> reproject_armor(const BuffAimPoint &aim_point, bool is_large = false) const
    {
        return solver_.reproject_armor(static_cast<const armor_task::AimPoint &>(aim_point), is_large);
    }

    double SJTU_cost(const std::vector<cv::Point2f> &cv_refs, const std::vector<cv::Point2f> &cv_pts, const double &inclined) const
    {
        return solver_.SJTU_cost(cv_refs, cv_pts, inclined);
    }

    const cv::Mat &camera_matrix() const { return solver_.camera_matrix(); }
    const cv::Mat &distort_coeffs() const { return solver_.distort_coeffs(); }
    io::CameraInfo as_camera_info(int camera_id = 0, std::string name = {}) const { return solver_.as_camera_info(camera_id, std::move(name)); }

    armor_task::PnpSolver &inner() { return solver_; }
    const armor_task::PnpSolver &inner() const { return solver_; }

  private:
    armor_task::PnpSolver solver_;
};
}
