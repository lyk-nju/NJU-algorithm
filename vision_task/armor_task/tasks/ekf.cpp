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
// #include <Eigen/Dense>
// #include <functional>
// #include <deque>
// #include <map>
// #include <string>
// #include <iostream>

// // 定义常量维度，利用模板参数或直接定义可以启用编译期优化
// constexpr int STATE_DIM = 11; // 状态向量维度
// constexpr int MEAS_DIM = 4;   // 观测向量维度

// // 类型别名，方便书写
// using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
// using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
// using MeasVector  = Eigen::Matrix<double, MEAS_DIM, 1>;
// using MeasMatrix  = Eigen::Matrix<double, MEAS_DIM, STATE_DIM>;
// using MeasNoise   = Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>;

// class Ekf
// {
// public:
//     // 构造函数
//     Ekf(const StateVector &x0, const StateMatrix &P0);

//     // 预测步骤
//     StateVector predict(const StateMatrix &F, const StateMatrix &Q);

//     // 更新步骤
//     StateVector update(const MeasVector &z, 
//                        const MeasMatrix &H, 
//                        const MeasNoise &R,
//                        std::function<MeasVector(const StateVector &)> h_func,
//                        std::function<MeasVector(const MeasVector &, const MeasVector &)> z_subtract_func);

//     // 数据接口
//     std::map<std::string, double> data;

//     // 外部设置状态加法函数（处理角度归一化等）
//     std::function<StateVector(const StateVector &, const StateVector &)> x_add_func;

// private:
//     StateVector x;
//     StateMatrix P;
    
//     // 缓存单位矩阵，避免重复创建
//     const StateMatrix I = StateMatrix::Identity();

//     // 滑动窗口相关变量
//     std::deque<int> recent_nis_failures;
//     int nis_failure_sum = 0; // 维护一个和，避免每次遍历
//     const size_t window_size = 50; // 假设窗口大小为50，可调整

//     long long total_count_ = 0;
//     long long nis_count_ = 0;
//     long long nees_count_ = 0;
// };

// // ================= 实现部分 (通常放在 .cpp) =================

// Ekf::Ekf(const StateVector &x0, const StateMatrix &P0)
//     : x(x0), P(P0)
// {
//     // 初始化 map，避免运行时插入开销
//     data["residual_yaw"] = 0.0;
//     data["residual_pitch"] = 0.0;
//     data["residual_distance"] = 0.0;
//     data["residual_angle"] = 0.0;
//     data["nis"] = 0.0;
//     data["nees"] = 0.0;
//     data["nis_fail"] = 0.0;
//     data["nees_fail"] = 0.0;
//     data["recent_nis_failures"] = 0.0;
// }

// StateVector Ekf::predict(const StateMatrix &F, const StateMatrix &Q)
// {
//     // 默认预测模型：线性 x = F * x
//     // 1. 协方差预测: P = F * P * F^T + Q
//     // 为了极致性能，Eigen 会对这种固定大小矩阵做展开优化
//     P = F * P * F.transpose() + Q;
    
//     // 2. 状态预测
//     x = F * x;
    
//     return x;
// }

// StateVector Ekf::update(const MeasVector &z,
//                         const MeasMatrix &H,
//                         const MeasNoise &R,
//                         std::function<MeasVector(const StateVector &)> h_func,
//                         std::function<MeasVector(const MeasVector &, const MeasVector &)> z_subtract_func)
// {
//     StateVector x_prior = x; // 保存先验用于计算 NEES

//     // 1. 计算残差 y = z - h(x)
//     MeasVector z_pred = h_func(x);
//     MeasVector y = z_subtract_func(z, z_pred);

//     // 2. 计算残差协方差 S = H * P * H^T + R
//     // 使用 noalias() 告诉 Eigen 这里没有混叠，可以直接计算结果
//     MeasNoise S = H * P * H.transpose() + R;

//     // 3. 计算卡尔曼增益 K = P * H^T * S^-1
//     // 优化：使用 LDLT 分解求解线性方程，比 .inverse() 快且数值更稳定
//     // K * S = P * H^T  =>  K = (P * H^T) / S
//     Eigen::Matrix<double, STATE_DIM, MEAS_DIM> P_Ht = P * H.transpose();
//     Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K = S.ldlt().solve(P_Ht.transpose()).transpose();

//     // 4. 更新状态 x = x + K * y
//     // 需要使用 x_add_func 处理角度加法（如有）
//     StateVector dx = K * y;
//     if (x_add_func) {
//         x = x_add_func(x, dx);
//     } else {
//         x += dx;
//     }

//     // 5. 更新协方差 P = (I - K * H) * P
//     // 使用标准形式速度最快。如果数值稳定性极差，才需要用 Joseph Form。
//     // 这里使用标准形式优化版：P -= K * H * P
//     // 或者更稳健的: P = (I - K*H) * P
//     StateMatrix I_KH = I - K * H;
//     P = I_KH * P * I_KH.transpose() + K * R * K.transpose(); // Joseph Form (虽然慢点，但保证P对称正定，对长时间运行很重要)

//     // --- 以下是统计与调试部分 (已优化) ---
    
//     // 计算 NIS: y^T * S^-1 * y
//     // 利用之前分解过的 S 加速求解
//     double nis = y.transpose() * S.ldlt().solve(y);
    
//     // 计算 NEES: (x - x_prior)^T * P^-1 * (x - x_prior)
//     StateVector err_x = x - x_prior; // 这里简单减法，如果是角度需处理
//     double nees = err_x.transpose() * P.ldlt().solve(err_x);

//     constexpr double nis_threshold = 7.779; // 自由度4，90%置信区间 (0.711太小了，建议查表)
//     constexpr double nees_threshold = 19.675; // 自由度11

//     int current_fail = 0;
//     if (nis > nis_threshold) {
//         nis_count_++;
//         data["nis_fail"] = 1.0;
//         current_fail = 1;
//     } else {
//         data["nis_fail"] = 0.0;
//     }

//     if (nees > nees_threshold) {
//         nees_count_++;
//         data["nees_fail"] = 1.0;
//     } else {
//         data["nees_fail"] = 0.0;
//     }

//     total_count_++;

//     // O(1) 滑动窗口计算
//     recent_nis_failures.push_back(current_fail);
//     nis_failure_sum += current_fail;

//     if (recent_nis_failures.size() > window_size) {
//         int removed = recent_nis_failures.front();
//         recent_nis_failures.pop_front();
//         nis_failure_sum -= removed;
//     }

//     data["residual_yaw"] = y[0];
//     data["residual_pitch"] = y[1];
//     data["residual_distance"] = y[2];
//     data["residual_angle"] = y[3];
//     data["nis"] = nis;
//     data["nees"] = nees;
//     data["recent_nis_failures"] = static_cast<double>(nis_failure_sum) / recent_nis_failures.size();

//     return x;
// }