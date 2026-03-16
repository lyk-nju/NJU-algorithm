#include "serial_manager.hpp"
#include <Eigen/Dense>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace io
{
USB::USB(const std::string &send_port, const std::string &receive_port)
{
    // Initialize send port
    send_fd = open(send_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (send_fd < 0)
    {
        std::string error_msg = "Error opening send serial port: " + send_port + " (errno: " + std::to_string(errno) + " - " + strerror(errno) + ")";
        throw std::runtime_error(error_msg);
    }

    memset(&send_tty, 0, sizeof send_tty);
    if (tcgetattr(send_fd, &send_tty) != 0)
    {
        close(send_fd);
        throw std::runtime_error("Error getting send serial attributes");
    }

    cfsetospeed(&send_tty, B115200);
    cfsetispeed(&send_tty, B115200);

    send_tty.c_cflag = (send_tty.c_cflag & ~CSIZE) | CS8;
    send_tty.c_iflag &= ~IGNBRK;
    send_tty.c_lflag = 0;
    send_tty.c_oflag = 0;
    send_tty.c_cc[VMIN] = 0;
    send_tty.c_cc[VTIME] = 5;

    send_tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    send_tty.c_cflag |= (CLOCAL | CREAD);
    send_tty.c_cflag &= ~(PARENB | PARODD);
    send_tty.c_cflag &= ~CSTOPB;
    send_tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(send_fd, TCSANOW, &send_tty) != 0)
    {
        close(send_fd);
        throw std::runtime_error("Error setting send serial attributes");
    }

    // Initialize receive port
    receive_fd = open(receive_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (receive_fd < 0)
    {
        close(send_fd);
        std::string error_msg = "Error opening receive serial port: " + receive_port + " (errno: " + std::to_string(errno) + " - " + strerror(errno) + ")";
        throw std::runtime_error(error_msg);
    }

    memset(&receive_tty, 0, sizeof receive_tty);
    if (tcgetattr(receive_fd, &receive_tty) != 0)
    {
        close(send_fd);
        close(receive_fd);
        throw std::runtime_error("Error getting receive serial attributes");
    }

    cfsetospeed(&receive_tty, B115200);
    cfsetispeed(&receive_tty, B115200);

    receive_tty.c_cflag = (receive_tty.c_cflag & ~CSIZE) | CS8;
    receive_tty.c_iflag &= ~IGNBRK;
    receive_tty.c_lflag = 0;
    receive_tty.c_oflag = 0;
    receive_tty.c_cc[VMIN] = 0;
    receive_tty.c_cc[VTIME] = 5;

    receive_tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    receive_tty.c_cflag |= (CLOCAL | CREAD);
    receive_tty.c_cflag &= ~(PARENB | PARODD);
    receive_tty.c_cflag &= ~CSTOPB;
    receive_tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(receive_fd, TCSANOW, &receive_tty) != 0)
    {
        close(send_fd);
        close(receive_fd);
        throw std::runtime_error("Error setting receive serial attributes");
    }

    std::cout << "Serial ports opened successfully: " << send_port << " (send), " << receive_port << " (receive)" << std::endl;
}

USB::~USB()
{
    if (send_fd >= 0)
    {
        close(send_fd);
    }
    if (receive_fd >= 0)
    {
        close(receive_fd);
    }
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

    ss << valid_i << "," << shoot_i << "," << cmd.yaw << "," << cmd.pitch;

    for (double v : base_command.values)
    {
        ss << "," << v;
    }

    ss << "\n";

    std::string msg = ss.str();
    ssize_t n_written = write(send_fd, msg.c_str(), msg.length());
    return n_written == static_cast<ssize_t>(msg.length());
}

bool USB::receive_all(Eigen::Quaterniond &quat, double &yaw, double &pitch, JudgerData &judger_data)
{
    static std::string buffer;
    char tmp_buf[1024];

    // 1. 读取数据
    ssize_t n = read(receive_fd, tmp_buf, sizeof(tmp_buf) - 1);
    if (n > 0)
    {
        tmp_buf[n] = '\0';
        buffer.append(tmp_buf);
    }
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
        return false;
    }

    // 2. 缓冲区防爆
    if (buffer.size() > 4096)
    {
        buffer.clear();
        return false;
    }

    // 3. 从后往前扫描，寻找有效行
    size_t search_pos = buffer.size();
    std::string line;
    bool found_valid_line = false;
    size_t line_end_pos = std::string::npos;

    while (search_pos > 0)
    {
        size_t last_n = buffer.rfind('\n', search_pos - 1);
        if (last_n == std::string::npos)
        {
            if (buffer.size() > 100) buffer.clear();
            return false;
        }

        size_t current_line_end = last_n;
        size_t current_line_start = 0;
        size_t prev_n = buffer.rfind('\n', last_n - 1);
        if (prev_n != std::string::npos)
        {
            current_line_start = prev_n + 1;
        }

        size_t len = current_line_end - current_line_start;
        if (len > 10) // 过滤空行/过短行
        {
            line = buffer.substr(current_line_start, len);
            line_end_pos = current_line_end;
            found_valid_line = true;
            break;
        }
        else
        {
            search_pos = last_n;
        }
    }

    if (!found_valid_line) return false;

    // 4. 清除已处理数据
    buffer.erase(0, line_end_pos + 1);

    // 5. 解析数据：w,x,y,z,yaw,pitch,game_time,self_hp
    judger_data.game_time = 0;
    judger_data.self_hp = 0;

    std::vector<double> nums;
    nums.reserve(16); // 预留一点空间

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

        // 去掉可能的空白
        size_t first = token.find_first_not_of(" \t\r");
        size_t last = token.find_last_not_of(" \t\r");
        if (first == std::string::npos)
        {
            continue; // 全是空白，跳过
        }
        token = token.substr(first, last - first + 1);

        try
        {
            nums.push_back(std::stod(token));
        }
        catch (...)
        {
            return false; // 某个字段不是合法数字
        }
    }

    if (nums.size() < 6)
    {
        // 不足 6 个值，格式错误
        return false;
    }

    // 前 4 个：四元数，接着 yaw/pitch
    quat = Eigen::Quaterniond(nums[0], nums[1], nums[2], nums[3]);
    yaw = nums[4];
    pitch = nums[5];

    // 第 7、8 个：比赛时间、自身血量
    if (nums.size() >= 7)
        judger_data.game_time = static_cast<int>(nums[6]);
    if (nums.size() >= 8)
        judger_data.self_hp = static_cast<int>(nums[7]);

    return true;
}

bool USB::receive_quaternion(Eigen::Quaterniond &quat, double &yaw, double &pitch)
{
    JudgerData dummy;
    return receive_all(quat, yaw, pitch, dummy);
}
} // namespace io