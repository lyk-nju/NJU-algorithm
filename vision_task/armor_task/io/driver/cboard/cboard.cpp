#include "cboard.hpp"
#include "../../algorithm/codec.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace io
{
Cboard::Cboard(const std::string &config_path)
{
    port_ = "/dev/ttyACM0";
    baudrate_ = 115200;

    YAML::Node cfg = YAML::LoadFile(config_path);
    if (cfg["send_port"])
    {
        port_ = cfg["send_port"].as<std::string>();
    }
    if (cfg["baudrate"])
    {
        baudrate_ = cfg["baudrate"].as<uint32_t>();
    }

    // Cboard 构造即开启 read_thread，无需 main 中手动开启
    configure_and_open();
    thread_ = std::thread(&Cboard::read_thread, this);
}

Cboard::~Cboard()
{
    quit_.store(true);
    if (thread_.joinable())
    {
        thread_.join();
    }
    if (serial_.isOpen())
    {
        serial_.close();
    }
}

PlayerMode Cboard::mode() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return mode_;
}

JudgerData Cboard::judger() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return judge_;
}

Eigen::Quaterniond Cboard::gimbal_quat_at(std::chrono::steady_clock::time_point t) const
{
    std::lock_guard<std::mutex> lock(quat_mutex_);
    if (quat_buffer_.empty())
    {
        return Eigen::Quaterniond::Identity();
    }

    // 越界 clamp：避免外推产生发散
    if (t <= quat_buffer_.front().time) return quat_buffer_.front().quat;
    if (t >= quat_buffer_.back().time) return quat_buffer_.back().quat;

    // 二分查找第一个 time >= t 的采样
    auto it_next = std::lower_bound(
        quat_buffer_.begin(), quat_buffer_.end(), t,
        [](const TimedQuat &s, const std::chrono::steady_clock::time_point &tt) {
            return s.time < tt;
        });
    auto it_prev = std::prev(it_next);

    const double dt = std::chrono::duration<double>(it_next->time - it_prev->time).count();
    if (dt <= 0.0)
    {
        return it_prev->quat;
    }
    const double k = std::chrono::duration<double>(t - it_prev->time).count() / dt;
    return it_prev->quat.slerp(k, it_next->quat).normalized();
}

// 向下位机发送不另开线程，解算完了再发送
void Cboard::send(const io::Vision2Cboard &vision2cboard)
{
    if (!serial_.isOpen()) reconnect();

    try
    {
        serial_.write(io::codec::encode(vision2cboard));
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Cboard] Write failed: " << e.what() << std::endl;
        reconnect();
    }
}

bool Cboard::is_connected() const
{
    return serial_.isOpen();
}

bool Cboard::has_quat_data() const
{
    std::lock_guard<std::mutex> lock(quat_mutex_);
    return !quat_buffer_.empty();
}

bool Cboard::read(uint8_t *buffer, std::size_t size)
{
    try
    {
        return serial_.read(buffer, size) == size;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Cboard] Read failed: " << e.what() << std::endl;
        reconnect();
        return false;
    }
}

void Cboard::read_thread()
{
    int error_count = 0;
    while (!quit_.load())
    {
        if (error_count > 5000)
        {
            error_count = 0;
            reconnect();
            continue;
        }

        if (!serial_.isOpen())
        {
            reconnect();
            continue;
        }

        std::string line;
        line.reserve(256);
        uint8_t byte = 0;
        bool got_newline = false;
        while (!quit_.load())
        {
            if (!read(&byte, 1))
            {
                ++error_count;
                break;
            }
            if (byte == '\n')
            {
                got_newline = true;
                break;
            }
            if (byte != '\r')
            {
                line.push_back(static_cast<char>(byte));
                if (line.size() > 512)
                {
                    ++error_count;
                    line.clear();
                    break;
                }
            }
        }
        if (!got_newline || line.empty())
        {
            continue;
        }

        if (!io::codec::decode(line, rx_data_))
        {
            ++error_count;
            continue;
        }

        error_count = 0;
        const auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            mode_ = rx_data_.mode_;
            judge_ = rx_data_.judge_;
        }
        {
            std::lock_guard<std::mutex> lock(quat_mutex_);
            quat_buffer_.push_back({
                now,
                Eigen::Quaterniond(
                    rx_data_.gimbal_data_.w,
                    rx_data_.gimbal_data_.x,
                    rx_data_.gimbal_data_.y,
                    rx_data_.gimbal_data_.z)});
            if (quat_buffer_.size() > kQuatBufferSize)
            {
                quat_buffer_.pop_front();
            }
        }
    }
}

void Cboard::reconnect()
{
    while (!quit_.load())
    {
        if (serial_.isOpen())
            serial_.close();

        if (configure_and_open())
        {
            {
                std::lock_guard<std::mutex> lock(quat_mutex_);
                quat_buffer_.clear();
            }
            std::cerr << "[Cboard] Reconnected to " << port_ << std::endl;
            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

bool Cboard::configure_and_open()
{
    try
    {
        serial_.setPort(port_);
        serial_.setBaudrate(baudrate_);
        serial_.setBytesize(serial::eightbits);
        serial_.setParity(serial::parity_none);
        serial_.setStopbits(serial::stopbits_one);
        serial_.setFlowcontrol(serial::flowcontrol_none);
        serial_.setTimeout(timeout_);
        serial_.open();
        return serial_.isOpen();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Cboard] Open serial failed (" << port_ << "): " << e.what() << std::endl;
        return false;
    }
}
} // namespace io
