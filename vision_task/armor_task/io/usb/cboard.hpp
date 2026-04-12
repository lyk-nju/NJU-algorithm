#pragma once

#include "../dataframe/base_cmd.hpp"
#include "thread_safety.hpp"
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
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
    bool read(uint8_t *buffer, std::size_t size);
    void read_thread();
    void reconnect();
    bool configure_and_open();

    serial::Serial serial_;
    std::string port_;
    uint32_t baudrate_ = 115200;
    serial::Timeout timeout_ = serial::Timeout::simpleTimeout(20);

    std::thread thread_;
    std::atomic<bool> quit_{false};

    tools::ThreadSafeQueue<std::tuple<Cboard2Vision, std::chrono::steady_clock::time_point>> data_queue_{4000};
};
} // namespace io
