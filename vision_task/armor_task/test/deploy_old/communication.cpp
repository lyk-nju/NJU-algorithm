/*
communication.cpp:
测试目标：
    测试上位机与下位机是否能够正常通信

测试内容：
    1. 上位机向下位机发送控制指令，查看下位机是否能正常接受并执行
    2. 下位机向上位机发送状态信息，查看上位机是否能正常接受并显示
*/

#include "../../io/keyboard_manager.hpp"
#include "../../io/serial_manager.hpp"
#include "../../tools/parser.hpp"
#include "../../io/algorithm/ros2_transfer.hpp"
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <thread>
#include <yaml-cpp/yaml.h>

int main(int argc, char *argv[])
{
    std::cout << "=== Communication Test ===" << std::endl;

    // 从配置文件读取串口路径
    auto [send_port, receive_port] = load_ports_from_config();

    // 命令行参数可以覆盖配置文件
    if (argc >= 3)
    {
        send_port = argv[1];
        receive_port = argv[2];
        std::cout << "Using command line ports: " << send_port << " (send), " << receive_port << " (receive)" << std::endl;
    }
    else if (argc == 2)
    {
        send_port = argv[1];
        receive_port = argv[1];
        std::cout << "Using single command line port: " << send_port << " (shared send/receive)" << std::endl;
    }
    else
    {
        std::cout << "Usage: " << argv[0] << " [send_port] [receive_port]" << std::endl;
        std::cout << "       " << argv[0] << " [shared_port]" << std::endl;
        std::cout << "  (Ports can be specified via command line or ../config/deploy.yaml)" << std::endl;
    }

    // 初始化键盘管理器（多线程处理键盘输入）
    io::KeyboardManager keyboard_manager(5.0f); // 每次调整5度
    keyboard_manager.init();

    try
    {
        // 初始化 USB 通信
        io::USB usb(send_port, receive_port);

        io::KeyboardManager::print_controls();
        std::cout << "\nStarting interactive control..." << std::endl;

        // 控制标志（用于键盘管理器）
        std::atomic<bool> running(true);

        // 启动键盘输入处理线程（保留多线程特性）
        keyboard_manager.start(running);

        // 主循环：接收数据并处理命令发送
        auto last_send_time = std::chrono::steady_clock::now();
        const auto send_interval = std::chrono::milliseconds(50); // 每50ms发送一次
        io::Command last_cmd = keyboard_manager.get_command();

        geometry_msgs::msg::Twist move_twist;
        move_twist.linear.x = 1.0;
        move_twist.linear.y = 0.0;
        move_twist.linear.z = 0.0;
        move_twist.angular.x = 0.0;
        move_twist.angular.y = 0.0;
        move_twist.angular.z = 0.0;

        geometry_msgs::msg::Twist stop_twist;

        const auto base_command_duration = std::chrono::milliseconds(500);
        const auto base_command_start_time = std::chrono::steady_clock::now();
        bool base_command_zeroed = false;

        auto print_base_command = [](const io::base_command &cmd, const std::string &label)
        {
            std::cout << label;
            std::cout << "v_x" << cmd.v_x << "v_y" << cmd.v_y << "w_yaw:" << cmd.w_yaw;
            std::cout << std::endl;
        };

        print_base_command(io::transfer::from_cmd_vel(move_twist), "Base command payload (first 0.5s): ");
        print_base_command(io::transfer::from_cmd_vel(stop_twist), "Base command payload (after 0.5s): ");

        while (running)
        {
            // 获取当前命令
            io::Command cmd = keyboard_manager.get_command();
            bool cmd_changed = (cmd.yaw != last_cmd.yaw || cmd.pitch != last_cmd.pitch);

            // 定期发送命令（即使没有按键，也保持发送当前值）
            auto now = std::chrono::steady_clock::now();
            io::base_command base_cmd = io::transfer::from_cmd_vel((now - base_command_start_time) < base_command_duration ? move_twist : stop_twist);
            if (!base_command_zeroed && (now - base_command_start_time) >= base_command_duration)
            {
                base_command_zeroed = true;
                std::cout << "Base command reached 0.5s, clearing velocity to zero." << std::endl;
            }
            if (cmd_changed || (now - last_send_time) >= send_interval)
            {
                bool success = usb.send_command(cmd, base_cmd);
                // 打印实际发送的数据格式
                {
                    int valid_i = cmd.valid ? 1 : 0;
                    int shoot_i = cmd.shoot ? 1 : 0;
                    std::ostringstream ss;
                    ss.setf(std::ios::fixed);
                    ss.precision(6);
                    ss << valid_i << "," << shoot_i << "," << cmd.yaw << "," << cmd.pitch;
                    ss << "," << base_cmd.v_x << "," << base_cmd.v_y << "," << base_cmd.w_yaw;
                    ss << "\n";
                    std::cout << "  [实际发送] " << ss.str();
                }
                if (cmd_changed)
                {
                    if (success)
                    {
                        std::cout << "  Command sent (string): yaw=" << std::fixed << std::setprecision(3) << cmd.yaw << " rad (" << (cmd.yaw * 180.0 / 3.14159265) << " deg), pitch=" << cmd.pitch << " rad ("
                                  << (cmd.pitch * 180.0 / 3.14159265) << " deg)" << std::endl;
                    }
                    else
                    {
                        std::cout << "  Failed to send command" << std::endl;
                    }
                }
                last_cmd = cmd;
                last_send_time = now;
            }

            // 尝试接收数据（非阻塞），同时打印比赛时间和剩余血量
            Eigen::Quaterniond quat;
            double yaw_rad, pitch_rad;
            io::JudgerData judger_data;
            bool received = usb.receive_all(quat, yaw_rad, pitch_rad, judger_data);
            if (received)
            {
                // 获取接收数据的时间戳（高精度）
                auto receive_time = std::chrono::steady_clock::now();

                // 计算从程序启动到现在的毫秒数
                static auto program_start = std::chrono::steady_clock::now();
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(receive_time - program_start).count();

                // 同时获取系统时间用于显示
                auto time_now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(time_now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_now.time_since_epoch()) % 1000;
                std::stringstream ss;
                ss << std::put_time(std::localtime(&time_t), "%H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count();

                std::cout << "  [" << ss.str() << "] [Timestamp: " << elapsed_ms << "ms] Received IMU data (string): "
                          << "q(w=" << std::fixed << std::setprecision(4) << quat.w() << ", "
                          << "x=" << quat.x() << ", "
                          << "y=" << quat.y() << ", "
                          << "z=" << quat.z() << ") | "
                          << "yaw=" << yaw_rad << " rad (" << (yaw_rad * 180.0 / 3.14159265) << " deg), "
                          << "pitch=" << pitch_rad << " rad (" << (pitch_rad * 180.0 / 3.14159265) << " deg), "
                          << "game_time=" << judger_data.game_time << ", self_hp=" << judger_data.self_hp << std::endl;

                Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0);
                std::cout << "  [" << ss.str() << "] Calculated Euler (deg) ZYX: yaw=" << (euler[0] * 180.0 / 3.14159265) << " pitch=" << (euler[1] * 180.0 / 3.14159265) << " roll=" << (euler[2] * 180.0 / 3.14159265)
                          << std::endl;
                std::cout << "  [" << ss.str() << "] 裁判信息: 比赛时间=" << judger_data.game_time << ", 剩余血量=" << judger_data.self_hp << std::endl;
            }

            // 短暂休眠，避免CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "\n=== Communication Test Finished ===" << std::endl;

        // 停止键盘管理器并恢复终端设置
        keyboard_manager.stop();
        keyboard_manager.restore();
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Please check:" << std::endl;
        std::cerr << "  1. Serial ports are correct: " << send_port << ", " << receive_port << std::endl;
        std::cerr << "  2. Permissions: sudo chmod 666 " << send_port << " " << receive_port << std::endl;
        std::cerr << "  3. Lower computer is connected and powered on" << std::endl;

        // 停止键盘管理器并恢复终端设置
        keyboard_manager.stop();
        keyboard_manager.restore();
        return 1;
    }
}
