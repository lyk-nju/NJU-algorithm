#pragma once

#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <mutex>

namespace armor_task
{

struct IMUTimestamp
{
    std::chrono::steady_clock::time_point time;
    Eigen::Quaterniond quat;
    double yaw;
    double pitch;
};

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

} // namespace armor_task
