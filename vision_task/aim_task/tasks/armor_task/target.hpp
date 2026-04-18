#pragma once

#include "../../include/armor.hpp"
#include "ekf.hpp"
#include "math_tools.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

namespace armor_task
{

class Target
{
  public:
    int car_num; // recognized number
    bool jumped;
    int last_id; // debug only

    Target() = default;
    Target(const Armor &armor, std::chrono::steady_clock::time_point t, int armor_num, const Eigen::VectorXd P0_dig);
    // Target(double x, double vyaw, double radius, double h);

    void predict(std::chrono::steady_clock::time_point t);
    void predict(double dt);
    void update(const Armor &armor);

    Eigen::VectorXd ekf_x() const;
    const Ekf &ekf() const;
    std::vector<Eigen::Vector4d> armor_xyza_list() const;

    bool diverged() const;

    // 注意：首次满足收敛条件时会缓�?is_converged_ = true，因此非 const�?
    bool has_converged();

    bool isinit = true;

    bool checkinit();
    bool is_switch_, is_converged_;

  private:
    int armor_num_;
    int switch_count_;
    int update_count_;

    Ekf ekf_;
    std::chrono::steady_clock::time_point t_;

    void update_ypda(const Armor &armor, int id); // yaw pitch distance angle

    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd &x, int id) const;
    Eigen::MatrixXd h_jacobian(const Eigen::VectorXd &x, int id) const;
};

} // namespace armor_task
