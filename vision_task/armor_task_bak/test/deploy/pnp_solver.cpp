/*
pnp_solver.cpp：
测试目标：
    查看pnp_solver是否能正确解算出装甲板位姿，并保持与下位机保持通信

测试内容：
    1. camera_pub (单开一个线程或程序)
    2. camera_sub
    3. communication
    4. detector
    5. pnp_solver
    记录加入pnp_solver后装甲板识别与姿态解算的运行帧率及姿态解算结果，并保存至实验日志pnp_solver.log中
*/

#include "../../tasks/pnp_solver.hpp"
#include "../../io/keyboard_manager.hpp"
#include "../../io/ros2_manager.hpp"
#include "../../io/serial_manager.hpp"
#include "../../tasks/detector.hpp"
#include "../../tools/draw.hpp"
#include "../../tools/pharser.hpp"
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace armor_task;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 从配置文件加载配置
    std::string test_config_path = "../config/deploy_test.yaml";
    if (argc > 1) test_config_path = argv[1]; // 允许通过命令行参数指定配置文件路径

    TestConfig test_config = load_deploy_test_config(test_config_path);

    // 允许命令行参数覆盖配置文件中的值
    std::string yolo_model_path = test_config.yolo_model_path;
    std::string config_path = test_config.config_path;
    std::string send_port = test_config.send_port;
    std::string receive_port = test_config.receive_port;

    // 打开日志文件
    std::ofstream log_file("../test/deploy/log/pnp_solver.log", std::ios::app);
    if (!log_file.is_open())
    {
        std::cerr << "Warning: Cannot open log file, logging to console only" << std::endl;
    }

    // 1、创建 ROS2 图像接受节点
    auto ros_node = std::make_shared<ROS2Manager>();
    std::thread spin_thread([&]() { rclcpp::spin(ros_node); });

    // 2、配置串口通信（已从配置文件加载）
    io::USB *usb = nullptr;
    try
    {
        usb = new io::USB(send_port, receive_port);
        std::cout << "Communication initialized with ports: " << send_port << ", " << receive_port << std::endl;
    }
    catch (...)
    {
        std::cout << "Warning: Communication not available, continuing without it" << std::endl;
    }

    // 3、初始化键盘管理器（多线程处理键盘输入）
    io::KeyboardManager keyboard_manager(5.0f); // 每次调整5度
    keyboard_manager.init();

    // 4、初始化检测器
    Detector detector(yolo_model_path);

    // 5、初始化 PnP 解算器
    PnpSolver pnp_solver(config_path);

    Eigen::Quaterniond identity_quat(1.0, 0.0, 0.0, 0.0); // w=1, x=0, y=0, z=0
    pnp_solver.set_R_gimbal2world(identity_quat);
    std::cout << "Set gimbal to identity rotation (no rotation)" << std::endl;

    // 控制标志
    std::atomic<bool> running(true);

    io::KeyboardManager::print_controls();
    std::cout << "\nStarting PnP solver test loop..." << std::endl;

    // 启动键盘输入处理线程（保留多线程特性）
    keyboard_manager.start(running, [&running]() { running = false; });

    // 帧率统计
    int frame_count = 0;
    auto last_time = std::chrono::steady_clock::now();
    auto last_send_time = last_time;
    const auto send_interval = std::chrono::milliseconds(50); // 每50ms发送一次命令

    while (running && rclcpp::ok())
    {
        cv::Mat img;
        if (ros_node->get_img(img))
        {
            frame_count++;

            // step1: 检测阶段
            auto detect_start = std::chrono::steady_clock::now();
            ArmorArray armors = detector.detect(img);
            auto detect_end = std::chrono::steady_clock::now();
            double detect_time = std::chrono::duration<double, std::milli>(detect_end - detect_start).count();

            // step2: PnP 解算阶段
            // if (usb != nullptr) // 更新云台到世界坐标系的旋转矩阵（从下位机接收IMU四元数）
            // {
            //     try
            //     {
            //         Eigen::Quaterniond quat = usb->receive_quaternion();
            //         pnp_solver.set_R_gimbal2world(quat);
            //     }
            //     catch (...)
            //     {
            //         // 没有接收到数据或接收失败，继续使用上一次的旋转矩阵
            //     }
            // }
            auto pnp_start = std::chrono::steady_clock::now();
            int success_count = pnp_solver.solveArmorArray(armors);
            auto pnp_end = std::chrono::steady_clock::now();
            double pnp_time = std::chrono::duration<double, std::milli>(pnp_end - pnp_start).count();

            // 获取当前命令值（用于日志）
            io::Command cmd = keyboard_manager.get_command();
            float current_yaw = cmd.yaw;
            float current_pitch = cmd.pitch;

            // 显示结果
            cv::Mat display_frame = img.clone();

            // 绘制函数绘制装甲板检测结果
            drawArmorDetection(display_frame, armors);

            // 计算并显示帧率
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - last_time).count();
            double fps = 0.0;

            if (elapsed >= 1.0)
            {
                fps = frame_count / elapsed;
            }

            // 绘制函数显示性能信息
            drawPerformanceInfo(display_frame, fps, detect_time, pnp_time, success_count);

            // 显示键盘控制的 yaw/pitch
            std::string yaw_text = "My Yaw: " + std::to_string(current_yaw).substr(0, 5) + " deg";
            std::string pitch_text = "My Pitch: " + std::to_string(current_pitch).substr(0, 5) + " deg";

            if (elapsed >= 1.0)
            {
                // 记录到日志（只记录装甲板相关信息）
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");

                std::string log_entry = "[" + ss.str() + "]\n";

                // 记录每个成功解算的装甲板信息
                for (const auto &armor : armors)
                {
                    if (armor.p_camera.norm() > 0)
                    {
                        log_entry += "  Armor ID:" + std::to_string(armor.detect_id) + " Num:" + std::to_string(armor.car_num) + " Pos:(" + std::to_string(armor.p_camera.x()).substr(0, 4) + "," +
                                     std::to_string(armor.p_camera.y()).substr(0, 4) + "," + std::to_string(armor.p_camera.z()).substr(0, 4) + ")" + " Yaw:" + std::to_string(armor.yaw).substr(0, 5) + "\n";
                    }
                }

                // 只在有装甲板信息时才写入日志
                if (log_entry.length() > ss.str().length() + 3) // 检查是否有装甲板信息（除了时间戳和换行符）
                {
                    if (log_file.is_open())
                    {
                        log_file << log_entry;
                        log_file.flush();
                    }
                }

                // 控制台输出仍然显示系统信息（用于调试）
                std::string console_output = "[" + ss.str() + "] FPS: " + std::to_string(fps).substr(0, 5) + ", Detected: " + std::to_string(armors.size()) + ", PnP Success: " + std::to_string(success_count) +
                                             ", Detect Time: " + std::to_string(detect_time).substr(0, 5) + "ms" + ", PnP Time: " + std::to_string(pnp_time).substr(0, 5) + "ms\n";
                std::cout << console_output;

                frame_count = 0;
                last_time = current_time;
            }

            // 显示PnP解算结果
            drawPnPresult(display_frame, armors);

            // 定期发送命令（使用字符串协议）
            if (usb && (current_time - last_send_time) >= send_interval)
            {
                io::Command cmd_to_send = keyboard_manager.get_command();
                usb->send_command(cmd_to_send);
                last_send_time = current_time;
            }

            cv::imshow("PnP Solver Test", display_frame);
            char key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27)
            {
                running = false;
                break;
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    // 清理
    running = false;
    keyboard_manager.stop(); // 停止键盘输入处理线程

    if (log_file.is_open())
    {
        log_file.close();
    }

    if (usb)
    {
        delete usb;
    }

    keyboard_manager.restore(); // 恢复终端设置
    rclcpp::shutdown();
    spin_thread.join();
    cv::destroyAllWindows();

    std::cout << "\nPnP solver test finished." << std::endl;
    return 0;
}
