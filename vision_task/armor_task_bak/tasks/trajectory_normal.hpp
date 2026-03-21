#ifndef ARMOR_TASK__TRAJECTORY_HPP
#define ARMOR_TASK__TRAJECTORY_HPP

namespace armor_task
{
struct Trajectory
{
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为正

  // 不考虑空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  Trajectory(const double v0, const double d, const double h);
};

}  // namespace armor_task

#endif  // ARMOR_TASK__TRAJECTORY_HPP