#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include "../include/armor.hpp"
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
    Target(
        const Armor &armor,
        std::chrono::steady_clock::time_point t,
        int armor_num,
        const Eigen::VectorXd P0_dig,
        double process_noise_v1 = 100.0,
        double process_noise_v1y = 100.0,
        double process_noise_v2 = 400.0);
    // Target(double x, double vyaw, double radius, double h);

    void predict(std::chrono::steady_clock::time_point t);
    void predict(double dt);
    void update(const Armor &armor);

    Eigen::VectorXd ekf_x() const;
    const Ekf &ekf() const;
    std::vector<Eigen::Vector4d> armor_xyza_list() const;

    bool diverged() const;

    bool converged();

    bool isinit = true;

    bool checkinit();
    bool is_switch_, is_converged_;

  private:
    int armor_num_;
    int switch_count_;
    int update_count_;

    Ekf ekf_;
    std::chrono::steady_clock::time_point t_;

    // 复用矩阵，避免每帧动态分配 (P-04)
    // 使用 MatrixXd 而非固定大小类型，确保传入 EKF 的 const MatrixXd& 参数时无隐式拷贝
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;

    void update_ypda(const Armor &armor, int id); // yaw pitch distance angle

    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd &x, int id) const;
    void h_jacobian(const Eigen::VectorXd &x, int id, Eigen::Ref<Eigen::MatrixXd> H) const;

    double process_noise_v1_ = 100.0;
    double process_noise_v1y_ = 100.0;
    double process_noise_v2_ = 400.0;
};

} // namespace armor_task

#endif // AUTO_AIM__TARGET_HPP