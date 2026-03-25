#ifndef SERIAL_MANAGER_HPP
#define SERIAL_MANAGER_HPP

#include <Eigen/Dense>
#include <mutex>
#include <string>
#include <termios.h>
#include <vector>

namespace io
{

std::string check_port();

struct Command
{
    bool valid = false; // 指令是否有效
    bool shoot = false; // 是否开火
    float yaw = 0.0f;   // 目标yaw角度 (弧度, radians)
    float pitch = 0.0f; // 目标pitch角度 (弧度, radians)
};

struct base_Command
{
    float v_x = 0.0f;  
    float v_y = 0.0f;
    float w_yaw = 0.0f;
};

struct JudgerData
{
    int game_time = 0; // 比赛时间（如剩余秒数，由串口协议约定）
    int self_hp = 0;   // 自身血量
    int self_id = 0; 
};

struct AimerData
{
    bool cmd_valid = false;
    int game_time = 0; // 比赛时间（如剩余秒数，由串口协议约定）
    int self_hp = 0; 
};

struct imu_data
{
    float q1;
    float q2;
    float q3;
    float q4;
};

class USB
{
  private:
    int send_fd = -1;
    int receive_fd = -1;
    struct termios send_tty;
    struct termios receive_tty;
    std::string send_port_;
    std::string receive_port_;
    std::string receive_buffer_;
    std::mutex serial_mutex_;

    int open_port(const std::string &port);
    void configure_port(int fd, struct termios &tty);
    void close_ports_locked();
    bool reopen_ports_locked();

  public:
    
    USB(const std::string &send_port, const std::string &receive_port);
    ~USB();

    // 字符串版本的发送和接收（唯一保留的发送接口）
    bool send_command(const Command &cmd);
    bool send_command(const Command &cmd, const base_Command &base_command);
    bool receive_quaternion(Eigen::Quaterniond &quat, double &yaw, double &pitch);
    bool receive_all(Eigen::Quaterniond &quat, double &yaw, double &pitch, JudgerData &judger_data);
};
} // namespace io
#endif
