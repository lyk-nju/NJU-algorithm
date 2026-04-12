#include "cboard.hpp"
#include "unpack.hpp"

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

    //cboard对象构造即开启read_thread，无需main中手动开启
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
    std::tuple<Cboard2Vision, std::chrono::steady_clock::time_point> latest{};
    if (data_queue_.peek_latest(latest))
    {
        return std::get<0>(latest).mode_;
    }
    return PlayerMode::MANUAL;
}

JudgerData Cboard::judger() const
{
    std::tuple<Cboard2Vision, std::chrono::steady_clock::time_point> latest{};
    if (data_queue_.peek_latest(latest))
    {
        return std::get<0>(latest).judge_;
    }
    return {};
}

bool Cboard::set_enemy(){
    const JudgerData judger_data = this->judger();
    if(judger_data.self_id > 11)
    {
        return true; // 红色
    }
    return false; // 蓝色
}

Eigen::Quaterniond Cboard::q(std::chrono::steady_clock::time_point t)
{
    std::tuple<Cboard2Vision, std::chrono::steady_clock::time_point> a;
    if (!data_queue_.pop(a))
    {
        return Eigen::Quaterniond::Identity();
    }

    std::tuple<Cboard2Vision, std::chrono::steady_clock::time_point> b;
    if (!data_queue_.front(b))
    {
        const auto &data_a = std::get<0>(a);
        return Eigen::Quaterniond(data_a.w, data_a.x, data_a.y, data_a.z);
    }

    while (true)
    {
        const auto &[data_a, t_a] = a;
        const auto &[data_b, t_b] = b;
        const Eigen::Quaterniond q_a(data_a.w, data_a.x, data_a.y, data_a.z);
        const Eigen::Quaterniond q_b(data_b.w, data_b.x, data_b.y, data_b.z);

        if (t <= t_a) return q_a;
        if (t <= t_b)
        {
            const double k =
                std::chrono::duration<double>(t - t_a).count() /
                std::chrono::duration<double>(t_b - t_a).count();
            return q_a.slerp(k, q_b).normalized();
        }

        if (!data_queue_.pop(a) || !data_queue_.front(b)) return q_b;
    }
}

//向下位机发送不另开线程，解算完了再发送
void Cboard::send(const io::Vision2Cboard &vision2cboard)
{
    if (!serial_.isOpen()) reconnect();

    try
    {
        serial_.write(unpack::encode(vision2cboard));
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Cboard] Write failed: " << e.what() << std::endl;
        reconnect();
    }
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

        Cboard2Vision parsed{};
        if (!unpack::decode(line, parsed))
        {
            ++error_count;
            continue;
        }

        error_count = 0;
        const auto now = std::chrono::steady_clock::now();
        data_queue_.push(std::make_tuple(parsed, now));
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
            data_queue_.clear();
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
