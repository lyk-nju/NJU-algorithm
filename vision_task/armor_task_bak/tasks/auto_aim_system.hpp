#pragma once

#include "../io/serial_manager.hpp"
#include "aimer.hpp"
#include "detector.hpp"
#include "pnp_solver.hpp"
#include "target.hpp"
#include "tracker.hpp"
#include <chrono>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>

namespace armor_task
{

/// IMU 历史数据结构（用于图像-IMU 时间戳同步）
struct IMUTimestamp
{
    std::chrono::steady_clock::time_point time;
    Eigen::Quaterniond quat;
    double yaw;
    double pitch;
};

/// IMU 历史缓冲（按时间戳查询，用于与图像曝光时刻对齐）
class IMUHistory
{
  public:
    void push(const Eigen::Quaterniond &quat, double yaw, double pitch);
    bool query(const std::chrono::steady_clock::time_point &target_time, Eigen::Quaterniond &out_quat, double &out_yaw, double &out_pitch);
    size_t size() const;

  private:
    std::deque<IMUTimestamp> buffer_;
    mutable std::mutex mutex_;
    static constexpr size_t MAX_SIZE = 100;
};

/// processFrame 的返回结果（供调用方用于显示、串口发送等）
struct ProcessResult
{
    io::Command cmd;
    bool is_switching;
    std::vector<Target> targets;
    ArmorArray armors;
    AimPoint aim_point;
    double detect_time_ms;
    double track_time_ms;
    double aim_time_ms;
    std::string tracker_state;
};

/**
 * @brief 自瞄算法统一入口（纯算法，无 ROS/串口/键盘依赖）
 *
 * 封装 Detector、PnpSolver、Tracker、Aimer 及 IMU 同步逻辑，
 * 供 auto_aimer（独立部署）和 nju_autoaim_node（TUP 集成）共用。
 */
class AutoAimSystem
{
  public:
    AutoAimSystem(const std::string &yolo_model_path, const std::string &config_path, double bullet_speed);
    ~AutoAimSystem() = default;

    /// 更新 IMU/云台姿态（由外部 IMU 数据源周期性调用）
    void updateImu(const Eigen::Quaterniond &quat, double yaw, double pitch);

    /// 处理一帧图像，返回瞄准控制量与中间结果
    ProcessResult processFrame(const cv::Mat &img, std::chrono::steady_clock::time_point image_timestamp);

    bool usingTensorRt() const { return detector_.usingTensorRt(); }
    const char *backendName() const { return detector_.backendName(); }

    /// 获取 PnP 解算器（用于弹道绘制等，如 drawTrajectory 需要 R_gimbal2world_）
    const PnpSolver &getPnpSolver() const { return pnp_solver_; }

  private:
    Detector detector_;
    PnpSolver pnp_solver_;
    Tracker tracker_;
    Aimer aimer_;
    IMUHistory imu_history_;
    double bullet_speed_;
    Eigen::Quaterniond last_quat_; /// 最近一次成功查询的 IMU 四元数，用于 query 失败时的降级
};

} // namespace armor_task
