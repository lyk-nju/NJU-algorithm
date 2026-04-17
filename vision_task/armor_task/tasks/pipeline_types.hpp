#pragma once

// AimPipeline 的输入/输出数据契约。
//
// 这些类型刻意放在一个独立头里，使得：
//   1. aim_pipeline.hpp 不需要 include 任何 io/driver/**（保持算法层纯净）；
//   2. 将来 io/scheduler/frame_scheduler.hpp 也可以复用 FrameBundle，而不会产生
//      tasks/ <-> io/ 的循环依赖。
//
// 依赖方向：
//   pipeline_types.hpp --> io/structs/**, armor.hpp, aimer.hpp (纯头)
//   aim_pipeline.hpp   --> pipeline_types.hpp + 各算法类头

#include "../include/armor.hpp"
#include "../io/structs/structs.hpp"
#include "aimer.hpp"   // AimPoint
#include "target.hpp"  // Target

#include <Eigen/Geometry>
#include <vector>

namespace armor_task
{

/**
 * @brief 每帧输入给 AimPipeline 的只读快照。
 *
 * 不变量（由上层 AutoAimSystem / FrameScheduler 保证）：
 *   - frame.image 非空；
 *   - camera 非空，camera->camera_id == frame.camera_id；
 *   - camera->camera_matrix、camera->distort_coeffs 非空（Pipeline 会把它们灌进 PnpSolver）；
 *   - gimbal_quat 已对齐到 frame.timestamp（由上层查询 cboard.gimbal_quat_at）。
 *
 * 生命周期：camera 指向的对象必须在本次 step 返回前保持有效。实务上 Scheduler
 * 持有一张 camera_id -> CameraInfo 的表，单相机场景下该表只有一项。
 */
struct FrameBundle
{
    io::Frame frame;
    const io::CameraInfo *camera = nullptr;
    Eigen::Quaterniond gimbal_quat = Eigen::Quaterniond::Identity();
};

/**
 * @brief 跨帧的业务状态，由上层维护并按值传入 Pipeline。
 *
 * Pipeline 不持有它的副本，也不修改它——保持"算法无外部可变状态"的定位。
 */
struct GameState
{
    bool enemy_is_red = true;
    double bullet_speed = 24.0;
};

/**
 * @brief Pipeline 的生产路径返回值。
 *
 * 刻意不包含任何中间量（ArmorArray / targets / AimPoint 等），以避免
 * 生产路径每帧无意义地拷贝几十字节乃至几 KB 的调试数据。
 * 需要调试数据时请使用 AimPipeline::step 的重载 + DebugSnapshot。
 */
struct AimDecision
{
    io::Vision2Cboard cmd{};
    bool valid_target = false;
};

/**
 * @brief 调试/可视化路径用的中间量快照。
 *
 * 仅当调用 AimPipeline::step(..., DebugSnapshot &) 时才会被填充。
 * 字段被设计为允许复用 buffer：调用方在循环外创建一次，每帧复用。
 */
struct DebugSnapshot
{
    ArmorArray detected_armors;          // Detector 输出
    std::vector<Target> targets;         // Tracker 输出
    AimPoint debug_aim_point{};          // Aimer 选定的瞄准点
};

} // namespace armor_task
