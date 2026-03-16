#include "auto_aim_system.hpp"
#include <algorithm>

namespace armor_task
{

void IMUHistory::push(const Eigen::Quaterniond &quat, double yaw, double pitch)
{
    std::lock_guard<std::mutex> lock(mutex_);
    IMUTimestamp data;
    data.time = std::chrono::steady_clock::now();
    data.quat = quat;
    data.yaw = yaw;
    data.pitch = pitch;
    buffer_.push_back(data);
    if (buffer_.size() > MAX_SIZE)
    {
        buffer_.pop_front();
    }
}

bool IMUHistory::query(const std::chrono::steady_clock::time_point &target_time, Eigen::Quaterniond &out_quat, double &out_yaw, double &out_pitch)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty()) return false;

    if (target_time <= buffer_.front().time)
    {
        out_quat = buffer_.front().quat;
        out_yaw = buffer_.front().yaw;
        out_pitch = buffer_.front().pitch;
        return true;
    }
    if (target_time >= buffer_.back().time)
    {
        out_quat = buffer_.back().quat;
        out_yaw = buffer_.back().yaw;
        out_pitch = buffer_.back().pitch;
        return true;
    }

    auto it = std::lower_bound(buffer_.begin(), buffer_.end(), target_time,
                              [](const IMUTimestamp &data, const std::chrono::steady_clock::time_point &t) { return data.time < t; });

    if (it != buffer_.begin())
    {
        auto prev = std::prev(it);
        auto dt_prev = std::abs(std::chrono::duration_cast<std::chrono::microseconds>(prev->time - target_time).count());
        auto dt_curr = std::abs(std::chrono::duration_cast<std::chrono::microseconds>(it->time - target_time).count());
        if (dt_prev < dt_curr)
        {
            it = prev;
        }
    }

    out_quat = it->quat;
    out_yaw = it->yaw;
    out_pitch = it->pitch;
    return true;
}

size_t IMUHistory::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
}

AutoAimSystem::AutoAimSystem(const std::string &yolo_model_path, const std::string &config_path, double bullet_speed)
    : detector_(yolo_model_path),
      pnp_solver_(config_path),
      tracker_(config_path, pnp_solver_),
      aimer_(config_path),
      bullet_speed_(bullet_speed),
      last_quat_(1.0, 0.0, 0.0, 0.0)
{
}

void AutoAimSystem::updateImu(const Eigen::Quaterniond &quat, double yaw, double pitch)
{
    imu_history_.push(quat, yaw, pitch);
}

ProcessResult AutoAimSystem::processFrame(const cv::Mat &img, std::chrono::steady_clock::time_point image_timestamp)
{
    ProcessResult result;
    result.aim_point.valid = false;
    result.aim_point.xyza.setZero();
    result.detect_time_ms = 0.0;
    result.track_time_ms = 0.0;
    result.aim_time_ms = 0.0;
    result.tracker_state = tracker_.state();
    result.is_switching = false;

    // 1. 更新云台到世界坐标系的旋转（从 IMU 历史查询图像时刻的 IMU）
    Eigen::Quaterniond synced_quat;
    double synced_yaw, synced_pitch;
    if (imu_history_.query(image_timestamp, synced_quat, synced_yaw, synced_pitch))
    {
        pnp_solver_.set_R_gimbal2world(synced_quat);
        last_quat_ = synced_quat;
    }
    else if (imu_history_.size() > 0)
    {
        // 降级：使用最近一次成功的 IMU
        pnp_solver_.set_R_gimbal2world(last_quat_);
    }
    else
    {
        // 无 IMU 数据时使用单位四元数
        pnp_solver_.set_R_gimbal2world(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
    }

    // 2. 检测
    auto detect_start = std::chrono::steady_clock::now();
    ArmorArray armors = detector_.detect(img);
    auto detect_end = std::chrono::steady_clock::now();
    result.detect_time_ms = std::chrono::duration<double, std::milli>(detect_end - detect_start).count();
    result.armors = armors;

    // 3. 追踪
    auto track_start = std::chrono::steady_clock::now();
    std::vector<Target> targets = tracker_.track(armors, track_start);
    auto track_end = std::chrono::steady_clock::now();
    result.track_time_ms = std::chrono::duration<double, std::milli>(track_end - track_start).count();
    result.targets = targets;
    result.tracker_state = tracker_.state();

    if (!targets.empty())
    {
        result.is_switching = targets[0].is_switch_;
    }

    // 4. 瞄准
    if (!targets.empty())
    {
        auto aim_start = std::chrono::steady_clock::now();
        std::list<Target> target_list(targets.begin(), targets.end());
        result.cmd = aimer_.aim(target_list, track_end, bullet_speed_);
        result.aim_point = aimer_.debug_aim_point;
        result.aim_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - aim_start).count();
    }
    else
    {
        result.cmd.valid = false;
        result.cmd.shoot = false;
        result.cmd.yaw = 0.0f;
        result.cmd.pitch = 0.0f;
    }

    return result;
}

} // namespace armor_task
