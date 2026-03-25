#include "serial_manager.hpp"
#include <Eigen/Dense>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace io
{
namespace
{
bool is_port_accessible(const std::string &port)
{
    return !port.empty() && std::filesystem::exists(port) && std::filesystem::is_character_file(port);
}

std::string resolve_port(const std::string &preferred_port)
{
    if (is_port_accessible(preferred_port))
    {
        return preferred_port;
    }

    if (preferred_port.find("ttyACM") != std::string::npos)
    {
        return check_port();
    }

    return preferred_port;
}
} // namespace

std::string check_port()
{
    const char *candidates[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"};
    for (const char *candidate : candidates)
    {
        if (std::filesystem::exists(candidate) && std::filesystem::is_character_file(candidate))
        {
            return candidate;
        }
    }

    return "/dev/ttyACM0";
}

int USB::open_port(const std::string &port)
{
    return open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
}

void USB::configure_port(int fd, struct termios &tty)
{
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        throw std::runtime_error("Error getting serial attributes");
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        throw std::runtime_error("Error setting serial attributes");
    }

    tcflush(fd, TCIOFLUSH);
}

void USB::close_ports_locked()
{
    if (send_fd >= 0)
    {
        close(send_fd);
        send_fd = -1;
    }
    if (receive_fd >= 0)
    {
        close(receive_fd);
        receive_fd = -1;
    }
}

bool USB::reopen_ports_locked()
{
    close_ports_locked();
    receive_buffer_.clear();

    const std::string resolved_send_port = resolve_port(send_port_);
    const std::string resolved_receive_port = resolve_port(receive_port_);

    int new_send_fd = open_port(resolved_send_port);
    if (new_send_fd < 0)
    {
        return false;
    }

    int new_receive_fd = open_port(resolved_receive_port);
    if (new_receive_fd < 0)
    {
        close(new_send_fd);
        return false;
    }

    try
    {
        configure_port(new_send_fd, send_tty);
        configure_port(new_receive_fd, receive_tty);
    }
    catch (...)
    {
        close(new_send_fd);
        close(new_receive_fd);
        throw;
    }

    send_fd = new_send_fd;
    receive_fd = new_receive_fd;
    send_port_ = resolved_send_port;
    receive_port_ = resolved_receive_port;

    std::cout << "Serial ports opened successfully: " << send_port_ << " (send), " << receive_port_ << " (receive)" << std::endl;
    return true;
}

USB::USB(const std::string &send_port, const std::string &receive_port)
    : send_port_(send_port), receive_port_(receive_port)
{
    if (!reopen_ports_locked())
    {
        std::string error_msg = "Error opening serial ports: " + send_port_ + " (send), " + receive_port_ + " (receive)";
        if (errno != 0)
        {
            error_msg += " (errno: " + std::to_string(errno) + " - " + strerror(errno) + ")";
        }
        throw std::runtime_error(error_msg);
    }
}

USB::~USB()
{
    std::lock_guard<std::mutex> lock(serial_mutex_);
    close_ports_locked();
}

bool USB::send_command(const Command &cmd)
{
    base_Command empty_command;
    return send_command(cmd, empty_command);
}

bool USB::send_command(const Command &cmd, const base_Command &base_command)
{
    int valid_i = cmd.valid ? 1 : 0;
    int shoot_i = cmd.shoot ? 1 : 0;

    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(6);

    ss << valid_i << "," << shoot_i << "," << cmd.yaw << "," << cmd.pitch << "," << base_command.v_x << "," << base_command.v_y << "," << base_command.w_yaw;
    ss << "\n";

    const std::string msg = ss.str();
    std::lock_guard<std::mutex> lock(serial_mutex_);

    for (int attempt = 0; attempt < 2; ++attempt)
    {
        if (send_fd < 0 && !reopen_ports_locked())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        ssize_t n_written = write(send_fd, msg.c_str(), msg.length());
        if (n_written == static_cast<ssize_t>(msg.length()))
        {
            return true;
        }

        if (n_written < 0)
        {
            std::cerr << "Serial send failed on " << send_port_ << ": " << strerror(errno) << ", trying to reopen" << std::endl;
        }
        else
        {
            std::cerr << "Serial send incomplete on " << send_port_ << ": " << n_written << "/" << msg.length() << ", trying to reopen" << std::endl;
        }

        if (!reopen_ports_locked())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    return false;
}

bool USB::receive_all(Eigen::Quaterniond &quat, double &yaw, double &pitch, JudgerData &judger_data)
{
    char tmp_buf[1024];
    ssize_t n = 0;

    {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (receive_fd < 0 && !reopen_ports_locked())
        {
            return false;
        }

        n = read(receive_fd, tmp_buf, sizeof(tmp_buf) - 1);
        if (n > 0)
        {
            tmp_buf[n] = '\0';
            receive_buffer_.append(tmp_buf);
        }
        else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            std::cerr << "Serial receive failed on " << receive_port_ << ": " << strerror(errno) << ", trying to reopen" << std::endl;
            reopen_ports_locked();
            return false;
        }
    }

    if (receive_buffer_.size() > 4096)
    {
        receive_buffer_.clear();
        return false;
    }

    size_t search_pos = receive_buffer_.size();
    std::string line;
    bool found_valid_line = false;
    size_t line_end_pos = std::string::npos;

    while (search_pos > 0)
    {
        size_t last_n = receive_buffer_.rfind('\n', search_pos - 1);
        if (last_n == std::string::npos)
        {
            if (receive_buffer_.size() > 100)
            {
                receive_buffer_.clear();
            }
            return false;
        }

        size_t current_line_end = last_n;
        size_t current_line_start = 0;
        size_t prev_n = std::string::npos;
        if (last_n > 0)
        {
            prev_n = receive_buffer_.rfind('\n', last_n - 1);
        }
        if (prev_n != std::string::npos)
        {
            current_line_start = prev_n + 1;
        }

        size_t len = current_line_end - current_line_start;
        if (len > 10)
        {
            line = receive_buffer_.substr(current_line_start, len);
            line_end_pos = current_line_end;
            found_valid_line = true;
            break;
        }

        search_pos = last_n;
    }

    if (!found_valid_line)
    {
        return false;
    }

    receive_buffer_.erase(0, line_end_pos + 1);

    judger_data.game_time = -1;
    judger_data.self_hp = -1;
    judger_data.self_id = -1;

    std::vector<double> nums;
    nums.reserve(16);

    size_t start = 0;
    while (start < line.size())
    {
        size_t comma_pos = line.find(',', start);
        std::string token;
        if (comma_pos == std::string::npos)
        {
            token = line.substr(start);
            start = line.size();
        }
        else
        {
            token = line.substr(start, comma_pos - start);
            start = comma_pos + 1;
        }

        size_t first = token.find_first_not_of(" \t\r");
        size_t last = token.find_last_not_of(" \t\r");
        if (first == std::string::npos)
        {
            continue;
        }
        token = token.substr(first, last - first + 1);

        try
        {
            nums.push_back(std::stod(token));
        }
        catch (...)
        {
            return false;
        }
    }

    if (nums.size() < 6)
    {
        return false;
    }

    quat = Eigen::Quaterniond(nums[0], nums[1], nums[2], nums[3]);
    yaw = nums[4];
    pitch = nums[5];

    if (nums.size() >= 7)
    {
        judger_data.game_time = static_cast<int>(nums[6]);
    }
    if (nums.size() >= 8)
    {
        judger_data.self_hp = static_cast<int>(nums[7]);
    }
    if (nums.size() >= 9)
    {
        judger_data.self_id = static_cast<int>(nums[8]);
    }

    return true;
}

bool USB::receive_quaternion(Eigen::Quaterniond &quat, double &yaw, double &pitch)
{
    JudgerData dummy;
    return receive_all(quat, yaw, pitch, dummy);
}
} // namespace io
