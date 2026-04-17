#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "../io/structs/structs.hpp"
#include "target.hpp"

// aimer内部只管yaw角的计算，pitch角的计算交给trajectory模块

namespace armor_task
{

struct AimPoint
{
    bool valid;
    Eigen::Vector4d xyza; // a represent yaw
};

class Aimer
{
  public:
    // 不变量：当 aim(targets, ...) 被调用且 targets 非空时，
    //        debug_aim_point 保证在返回前被赋值为本次调用选中的瞄准点
    //        （无论成功/失败——失败时 .valid 会被置为 false）。
    //        targets 为空这条早退分支不写该字段，但上层 AimPipeline
    //        在 targets 为空时不会调用 aim，所以外部观察到的永远是"本次调用的结果"。
    //        AimPipeline::step_impl 的 DebugSnapshot 路径依赖这一不变量。
    AimPoint debug_aim_point;
    explicit Aimer(const std::string &config_path);
    io::Vision2Cboard aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, bool to_now = true);
    io::Vision2Cboard aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed, bool to_now = true);

    double bullet_speed_;
  private:
    double yaw_offset_;
    // std::optional<double> left_yaw_offset_, right_yaw_offset_;
    double pitch_offset_;
    double coming_angle_;
    double leaving_angle_;
    double lock_id_ = -1;
    double high_speed_delay_time_;
    double low_speed_delay_time_;
    double decision_speed_;
    

    AimPoint choose_aim_point(const Target &target);
};
} // namespace armor_task
