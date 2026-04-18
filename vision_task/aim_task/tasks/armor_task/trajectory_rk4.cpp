#include "trajectory_rk4.hpp"
#include <cmath>
#include <iostream>

namespace armor_task
{

constexpr double g = 9.7833;  // 重力加速度

// 状态向量： [x, y, z, vx, vy, vz]
using State = Eigen::Vector<double, 6>;

// 计算状态导数（考虑重力和空气阻力）
State derivative(const State& state, double k)
{
  double vx = state[3];
  double vy = state[4];
  double vz = state[5];

  double v = std::sqrt(vx*vx + vy*vy + vz*vz);  // 速度大小

  State dstate;
  // 位置导数 = 速度
  dstate[0] = vx;
  dstate[1] = vy;
  dstate[2] = vz;

  // 速度导数 = 加速度（空气阻力 + 重力）
  if (v > 1e-6) {  // 避免除零
    double drag_coeff = k * v;  // 空气阻力系数
    dstate[3] = -drag_coeff * vx;  // x方向空气阻力
    dstate[4] = -drag_coeff * vy;  // y方向空气阻力
    dstate[5] = -drag_coeff * vz - g;  // z方向空气阻力 + 重力
  } else {
    dstate[3] = 0.0;
    dstate[4] = 0.0;
    dstate[5] = -g;
  }

  return dstate;
}

// 四阶龙格库塔法单步积分
State rk4_step(const State& state, double dt, double k)
{
  State k1 = derivative(state, k);
  State k2 = derivative(state + 0.5 * dt * k1, k);
  State k3 = derivative(state + 0.5 * dt * k2, k);
  State k4 = derivative(state + dt * k3, k);

  return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// 模拟弹道轨迹，返回最终状态和飞行时间
std::pair<State, double> simulate_trajectory(double v0, double pitch, double yaw,
                                           double target_x, double target_z,
                                           double k, double dt = 0.01, double max_time = 10.0)
{
  // 初始状态
  State state = State::Zero();
  state[3] = v0 * std::cos(pitch) * std::cos(yaw);  // vx
  state[4] = v0 * std::cos(pitch) * std::sin(yaw);  // vy
  state[5] = v0 * std::sin(pitch);                  // vz

  double time = 0.0;

  // 积分直到越过目标x位置或超时
  while (state[0] < target_x && time < max_time && state[2] >= -1.0) {  // z >= -1.0避免地下弹道
    state = rk4_step(state, dt, k);
    time += dt;
  }

  return {state, time};
}

// 计算给定pitch下的弹道误差（目标高度与实际高度的差）
double trajectory_error(double v0, double pitch, double yaw,
                       double target_x, double target_z, double k)
{
  auto [final_state, fly_time] = simulate_trajectory(v0, pitch, yaw, target_x, target_z, k);

  if (fly_time >= 10.0) {  // 超时
    return 1e10;
  }

  // 返回z方向的误差（实际z - 目标z）
  return final_state[2] - target_z;
}

// 计算导数（数值微分）
double derivative_error(double v0, double pitch, double yaw,
                       double target_x, double target_z, double k, double delta = 1e-6)
{
  double error1 = trajectory_error(v0, pitch, yaw, target_x, target_z, k);
  double error2 = trajectory_error(v0, pitch + delta, yaw, target_x, target_z, k);
  return (error2 - error1) / delta;
}

TrajectoryRK4::TrajectoryRK4(const double v0, const double d, const double h,
                            const double k, const int max_iter)
{
  // 假设水平方向无偏航角（2D问题）
  yaw = 0.0;

  // 牛顿迭代法搜索合适的发射角度
  double pitch = 0.1;  // 初始猜测：约5.7度
  double tolerance = 1e-3;  // 高度误差容限 1mm
  double learning_rate = 0.5;  // 学习率，避免过大步长

  unsolvable = true;

  for (int iter = 0; iter < max_iter; ++iter) {
    double error = trajectory_error(v0, pitch, yaw, d, h, k);
    double deriv = derivative_error(v0, pitch, yaw, d, h, k);

    // 检查导数是否太小（避免除零）
    if (std::abs(deriv) < 1e-6) {
      // 如果导数太小，使用固定步长调整
      if (error > 0) {
        pitch -= 0.01;  // 减小角度
      } else {
        pitch += 0.01;  // 增加角度
      }
    } else {
      // 牛顿迭代步长
      double delta_pitch = -learning_rate * error / deriv;
      pitch += delta_pitch;
    }

    // 限制角度范围
    pitch = std::max(-M_PI/3, std::min(M_PI/2, pitch));

    if (std::abs(error) < tolerance) {
      // 找到解
      auto [final_state, time] = simulate_trajectory(v0, pitch, yaw, d, h, k);
      fly_time = time;

      // 计算初始速度向量
      initial_velocity[0] = v0 * std::cos(pitch) * std::cos(yaw);
      initial_velocity[1] = v0 * std::cos(pitch) * std::sin(yaw);
      initial_velocity[2] = v0 * std::sin(pitch);

      unsolvable = false;
      break;
    }
  }

  if (unsolvable) {
    fly_time = 0.0;
    pitch = 0.0;
    initial_velocity = Eigen::Vector3d::Zero();
  }
}

}  // namespace armor_task