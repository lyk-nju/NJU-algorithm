#pragma once

#include <Eigen/Dense>

namespace armor_task
{

struct TrajectoryRK4
{
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为正，单位：弧度
  double yaw;    // 水平方向角度，单位：弧度
  Eigen::Vector3d initial_velocity;  // 初始速度向量

  // 考虑空气阻力，使用四阶龙格库塔法
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  // k 空气阻力系数，单位：kg/m (通常在0.001-0.01之间)
  // max_iter 最大迭代次数，默认100
  TrajectoryRK4(const double v0, const double d, const double h,
                const double k = 0.005, const int max_iter = 100);
};

}  // namespace armor_task