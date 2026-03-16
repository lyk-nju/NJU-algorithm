#include "ekf.hpp"
#include <iostream>
#include <numeric>

// 状态向量：(xc,vxc,yc,vyc,zc,vyz,yaw,w,r,dl,dh)
// a: angle
// w: angular velocity
// dl: r2 - r1
// dh: z2 - z1
/*
其中下标c表示旋转中心（即小车中心）的物理量。yaw为装甲板相对于世界坐标系原点的偏航角，w为旋转的角速度，r为旋转半径，dl为长短轴
 差，dh为高度差。引入dl,dh的原因是，制作规范只要求小车中心对称位置的装甲板高度与半径相同，而相邻装甲板旋转半径和安装高度可能不同。
*/

// 观测向量：（yaw,pitch,distance,orientation_yaw）
/*
此yaw,pitch非彼yaw,pitch。在观测向量建模时，我们取世界坐标系原点建立极坐标系，此处的yaw是目标在极坐标系下水平面的角度，pitch是目标在极坐标系下垂直面的角度，distance是目标与相机的距离,orieantaion_yaw是目标相对于原点的偏航角
*/
Ekf::Ekf(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0, std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add) :
    x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
    data["residual_yaw"] = 0.0;
    data["residual_pitch"] = 0.0;
    data["residual_distance"] = 0.0;
    data["residual_angle"] = 0.0;
    data["nis"] = 0.0;
    data["nees"] = 0.0;
    data["nis_fail"] = 0.0;
    data["nees_fail"] = 0.0;
    data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd Ekf::predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q)
{
    return predict(F, Q, [&](const Eigen::VectorXd &x) { return F * x; });
}

Eigen::VectorXd Ekf::predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
    P = F * P * F.transpose() + Q;
    x = f(x);
    return x;
}

Eigen::VectorXd Ekf::update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
    return update(z, H, R, [&](const Eigen::VectorXd &x) { return H * x; }, z_subtract);
}

Eigen::VectorXd Ekf::update(const Eigen::VectorXd &z,
                            const Eigen::MatrixXd &H,
                            const Eigen::MatrixXd &R,
                            std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
                            std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
    Eigen::VectorXd x_prior = x;
    Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Stable Compution of the Posterior Covariance
    // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

    x = x_add(x, K * z_subtract(z, h(x)));

    /// 卡方检验
    Eigen::VectorXd residual = z_subtract(z, h(x));

    // 新增检验
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    double nis = residual.transpose() * S.inverse() * residual;
    double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

    // 卡方检验阈值（自由度=4，取置信水平95%）
    constexpr double nis_threshold = 0.711;
    constexpr double nees_threshold = 0.711;

    if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
    if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
    total_count_++;
    last_nis = nis;

    recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

    if (recent_nis_failures.size() > window_size)
    {
        recent_nis_failures.pop_front();
    }

    int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
    double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

    data["residual_yaw"] = residual[0];
    data["residual_pitch"] = residual[1];
    data["residual_distance"] = residual[2];
    data["residual_angle"] = residual[3];
    data["nis"] = nis;
    data["nees"] = nees;
    data["recent_nis_failures"] = recent_rate;

    return x;
}
