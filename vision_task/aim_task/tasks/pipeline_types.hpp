#pragma once

// AimPipeline 的输入/输出数据约定。
//
// 这些类型单独放在此头文件中，避免 aim_pipeline.hpp 直接依赖 io/driver/**，
// 同时便于调度模块复用 FrameBundle，而不引入 tasks <-> io 循环依赖。

#include "../include/armor.hpp"
#include "../io/structs/structs.hpp"
#include "armor_task/aimer.hpp"   // AimPoint
#include "armor_task/target.hpp"  // Target

#include <Eigen/Geometry>
#include <vector>

namespace armor_task
{

/**
 * @brief 传给 AimPipeline 的单帧只读快照。
 *
 * 上层（AutoAimSystem / FrameScheduler）保证：
 * - frame.image 有效；
 * - camera 非空，且 camera->camera_id == frame.camera_id；
 * - camera 内参/畸变参数可用于 PnP 初始化；
 * - gimbal_quat 与 frame.timestamp 对齐。
 */
struct FrameBundle
{
    io::Frame frame;
    const io::CameraInfo *camera = nullptr;
    Eigen::Quaterniond gimbal_quat = Eigen::Quaterniond::Identity();
};

/**
 * @brief 跨帧业务状态，由上层维护并按值传入。
 *
 * Pipeline 不持有该状态的外部可变引用，也不在外部写回。
 */
struct GameState
{
    bool enemy_is_red = true;
    double bullet_speed = 24.0;
};

/**
 * @brief 调试/可视化路径的中间快照。
 *
 * 仅在调用 AimPipeline::step(..., DebugSnapshot &) 时填充。
 */
struct DebugSnapshot
{
    ArmorArray detected_armors;          // Detector 输出
    std::vector<Target> targets;         // Tracker 输出
    AimPoint debug_aim_point{};          // Aimer 选中的瞄准点
};

} // namespace armor_task
