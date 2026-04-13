#pragma once

#include "../dataframe/base_cmd.hpp"
#include "../thread_safe_queue.hpp"
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <cstdint>
#include <serial/serial.h>
#include <string>
#include <thread>
#include <tuple>

namespace io
{
class Cboard
{
public:
    explicit Cboard(const std::string &config_path);
    ~Cboard();

    PlayerMode mode() const;
    JudgerData judger() const;
    Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);
    bool set_enemy(JudgerData judger_data);

    void send(const io::Vision2Cboard &vision2cboard);

private:
    using TimedQuaternion = std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>;

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

    tools::ThreadSafeQueue<TimedQuaternion> data_queue_{1000};
};
} // namespace io
