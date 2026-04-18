#pragma once

#include "../../interface/i_cboard.hpp"
#include "../../structs/structs.hpp"

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <serial/serial.h>
#include <string>
#include <thread>

namespace io
{
class Cboard : public ICboard
{
public:
    explicit Cboard(const std::string &config_path);
    ~Cboard() override;

    PlayerMode mode() const override;
    JudgerData judger() const override;

    /**
     * 查询 t 时刻的云台四元数（slerp 插值），非破坏性。
     *
     * 内部维护一个按时间升序的环形缓冲（最近 kQuatBufferSize 条 IMU 采样）。
     * 若 t 落在缓冲区 [front, back] 内做 slerp；越界则 clamp 到端点；
     * 缓冲区为空（未收到任何 IMU 数据）时返回 Identity。
     */
    Eigen::Quaterniond gimbal_quat_at(std::chrono::steady_clock::time_point t) const override;

    void send(const io::Vision2Cboard &vision2cboard) override;

    /// 串口是否已成功打开
    bool is_connected() const override;

    /// 是否至少收到过一条 IMU 四元数采样
    bool has_quat_data() const override;

private:
    struct TimedQuat
    {
        std::chrono::steady_clock::time_point time;
        Eigen::Quaterniond quat;
    };

    bool read(uint8_t *buffer, std::size_t size);
    void read_thread();
    void reconnect();
    bool configure_and_open();

    Cboard2Vision rx_data_;
    PlayerMode mode_ = PlayerMode::MANUAL;
    JudgerData judge_;
    mutable std::mutex data_mutex_;

    serial::Serial serial_;
    std::string port_;
    uint32_t baudrate_ = 115200;
    serial::Timeout timeout_ = serial::Timeout::simpleTimeout(20);

    std::thread thread_;
    std::atomic<bool> quit_{false};

    mutable std::mutex quat_mutex_;
    std::deque<TimedQuat> quat_buffer_;
    static constexpr std::size_t kQuatBufferSize = 1000;
};
} // namespace io
