#include "imu_history.hpp"

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

} // namespace armor_task
