#include "target.hpp"
#include "../tools/math_tools.hpp"

namespace armor_task
{
Target::Target(
    const Armor &armor,
    std::chrono::steady_clock::time_point t,
    int total_armors,
    const Eigen::VectorXd P0_dig,
    double process_noise_v1,
    double process_noise_v1y,
    double process_noise_v2) :
    car_num(armor.car_num),
    jumped(false),
    last_id(0),
    update_count_(0),
    armor_num_(total_armors),
    t_(t),
    is_switch_(false),
    is_converged_(false),
    switch_count_(0),
    process_noise_v1_(process_noise_v1),
    process_noise_v1y_(process_noise_v1y),
    process_noise_v2_(process_noise_v2)
{
    F_.resize(11, 11);
    Q_.resize(11, 11);
    H_.resize(4, 11);
    R_.resize(4, 4);

    auto r = armor.r;
    const Eigen::VectorXd &xyz = armor.p_world;
    const Eigen::VectorXd &ypr = armor.ypr_in_world;

    // 旋转中心的坐标
    auto center_x = xyz[0] + r * std::cos(ypr[0]);
    auto center_y = xyz[1] + r * std::sin(ypr[0]);
    auto center_z = xyz[2];

    Eigen::VectorXd x0(11);
    x0 << center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0; // 初始化预测量
    Eigen::MatrixXd P0 = P0_dig.asDiagonal();

    // 防止夹角求和出现异常值
    auto x_add = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) -> Eigen::VectorXd
    {
        Eigen::VectorXd c = a + b;
        c[6] = tools::limit_rad(c[6]);
        return c;
    };

    ekf_ = Ekf(x0, P0, x_add); // 初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
    auto dt = tools::delta_time(t, t_);
    // std::cout << "dt: " << dt << std::endl;
    predict(dt);
    t_ = t;
}

void Target::predict(double dt)
{
    // dt = 0.03;
    // std::cout << "predict dt: " << dt << std::endl;
    // 状态转移矩阵 (常速度模型)
    F_.setIdentity();
    F_(0, 1) = dt;
    F_(2, 3) = dt;
    F_(4, 5) = dt;
    F_(6, 7) = dt;

    // clang-format on  // Piecewise White Noise Model
    // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
    const double v1 = process_noise_v1_;
    const double v1_y = process_noise_v1y_;
    const double v2 = process_noise_v2_;
    auto a = dt * dt * dt * dt / 4;
    auto b = dt * dt * dt / 2;
    auto c = dt * dt;
    // 预测过程噪声偏差的方差
    Q_.setZero();
    Q_(0, 0) = a * v1;
    Q_(0, 1) = b * v1;
    Q_(1, 0) = b * v1;
    Q_(1, 1) = c * v1;

    Q_(2, 2) = a * v1_y;
    Q_(2, 3) = b * v1_y;
    Q_(3, 2) = b * v1_y;
    Q_(3, 3) = c * v1_y;

    Q_(4, 4) = a * v1;
    Q_(4, 5) = b * v1;
    Q_(5, 4) = b * v1;
    Q_(5, 5) = c * v1;

    Q_(6, 6) = a * v2;
    Q_(6, 7) = b * v2;
    Q_(7, 6) = b * v2;
    Q_(7, 7) = c * v2;

    Q_(8, 8) = 1e-5;
    Q_(9, 9) = 1e-3;
    Q_(10, 10) = 1e-4;

    // 防止夹角求和出现异常值
    auto f = [&](const Eigen::VectorXd &x) -> Eigen::VectorXd
    {
        Eigen::VectorXd x_prior = F_ * x;
        x_prior[6] = tools::limit_rad(x_prior[6]);
        return x_prior;
    };

    //   // 前哨站转速特判
    //   if (this->converged() && this->name == ArmorName::outpost && std::abs(this->ekf_.state()[7]) > 2)
    //     this->ekf_.state()[7] = this->ekf_.state()[7] > 0 ? 2.51 : -2.51;

    ekf_.predict(F_, Q_, f);
}

void Target::update(const Armor &armor)
{
    // 装甲板匹配
    int id = 0;
    auto min_angle_error = 1e10;
    const std::vector<Eigen::Vector4d> &xyza_list = armor_xyza_list();

    std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
    for (int i = 0; i < armor_num_; i++)
    {
        xyza_i_list.push_back({xyza_list[i], i});
    }

    std::sort(xyza_i_list.begin(),
              xyza_i_list.end(),
              [](const std::pair<Eigen::Vector4d, int> &a, const std::pair<Eigen::Vector4d, int> &b)
              {
                  Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
                  Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
                  return ypd1[2] < ypd2[2];
              });

    // 取前3个distance最小的装甲板（若 armor_num_ < 3 则取实际数量）
    const int k = std::min(3, armor_num_);
    for (int i = 0; i < k; i++)
    {
        const auto &xyza = xyza_i_list[i].first;
        Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
        auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) + std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

        if (std::abs(angle_error) < std::abs(min_angle_error))
        {
            id = xyza_i_list[i].second;
            min_angle_error = angle_error;
        }
    }

    if (id != 0) jumped = true;

    if (id != last_id)
    {
        is_switch_ = true;
    }
    else
    {
        is_switch_ = false;
    }

    if (is_switch_) switch_count_++;

    last_id = id;
    update_count_++;

    update_ypda(armor, id);
}

void Target::update_ypda(const Armor &armor, int id)
{
    // 观测jacobi
    h_jacobian(ekf_.state(), id, H_);
    // 减小观测噪声以提高对观测值的依赖
    auto center_yaw = std::atan2(armor.p_world[1], armor.p_world[0]);
    auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
    Eigen::Vector4d R_dig;
    R_dig << 1e-1,
        1e-1,
        (log(std::abs(delta_angle) + 1) + 1),
        (log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 10e-2);
    R_.setZero();
    R_.diagonal() = R_dig;

    // 定义非线性转换函数h: x -> z
    auto h = [&](const Eigen::VectorXd &x) -> Eigen::Vector4d
    {
        Eigen::VectorXd xyz = h_armor_xyz(x, id);
        Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
        auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
        return {ypd[0], ypd[1], ypd[2], angle};
    };
    // std::cout << "Updating with armor id: " << id << std::endl;

    // 防止夹角求差出现异常值
    auto z_subtract = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) -> Eigen::VectorXd
    {
        Eigen::VectorXd c = a - b;
        c[0] = tools::limit_rad(c[0]);
        c[1] = tools::limit_rad(c[1]);
        c[3] = tools::limit_rad(c[3]);
        return c;
    };

    const Eigen::VectorXd &ypd = armor.ypd_in_world;
    const Eigen::VectorXd &ypr = armor.ypr_in_world;
    Eigen::VectorXd z(4);
    z << ypd[0], ypd[1], ypd[2], ypr[0];

    // 打印观测向量
    // std::cout << "Observation z: [" << z[0] << ", " << z[1] << ", " << z[2] << ", " << z[3] << "]" << std::endl;

    ekf_.update(z, H_, R_, h, z_subtract);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.state(); }

const Ekf &Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
    std::vector<Eigen::Vector4d> _armor_xyza_list;

    for (int i = 0; i < armor_num_; i++)
    {
        const auto &x = ekf_.state();
        auto angle = tools::limit_rad(x[6] + i * 2 * CV_PI / armor_num_);
        Eigen::Vector3d xyz = h_armor_xyz(x, i);
        _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
    }
    return _armor_xyza_list;
}

bool Target::diverged() const
{
    const auto &x = ekf_.state();
    auto r_ok = x[8] > 0.05 && x[8] < 0.5;
    auto l_ok = x[8] + x[9] > 0.05 && x[8] + x[9] < 0.5;

    // std::cout << "[Target] r=" << std::fixed << std::setprecision(3) << ekf_.x[8] << ", l=" << std::fixed << std::setprecision(3) << ekf_.x[9] << std::endl;

    if (r_ok && l_ok) return false;
    return true;
}

bool Target::converged()
{
    //   if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    //     is_converged_ = true;
    //   }

    //   //前哨站特殊判断
    //   if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    //     is_converged_ = true;
    //   }
    if (update_count_ > 3 && !this->diverged())
    {
        is_converged_ = true;
    }

    return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd &x, int id) const
{
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

    auto r = (use_l_h) ? x[8] + x[9] : x[8];
    // std::cout<<"r:"<<x[8]<<std::endl;
    // std::cout<<"dl"<<x[9]<<std::endl;
    // std::cout<<"actual r :"<<r<<std::endl;
    // std::cout<<"dh "<<x[10]<<std::endl;
    auto armor_x = x[0] - r * std::cos(angle);
    auto armor_y = x[2] - r * std::sin(angle);
    auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

    //std::cout<<"R:"<< r <<std::endl;
    return {armor_x, armor_y, armor_z};
}

void Target::h_jacobian(const Eigen::VectorXd &x, int id, Eigen::Ref<Eigen::MatrixXd> H) const
{
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

    auto r = (use_l_h) ? x[8] + x[9] : x[8];
    auto dx_da = r * std::sin(angle);
    auto dy_da = -r * std::cos(angle);

    auto dx_dr = -std::cos(angle);
    auto dy_dr = -std::sin(angle);
    auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
    auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

    auto dz_dh = (use_l_h) ? 1.0 : 0.0;

    Eigen::Matrix<double, 4, 11> H_armor_xyza;
    H_armor_xyza << 1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl, 0,
        0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dz_dh,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;

    Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
    Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
    Eigen::Matrix4d H_armor_ypda;
    H_armor_ypda << H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0,
        H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0,
        H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0,
        0, 0, 0, 1;

    H = H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

} // namespace armor_task
