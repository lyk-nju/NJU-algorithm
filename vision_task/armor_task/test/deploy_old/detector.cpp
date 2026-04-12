/*
detector.cpp：
测试目标：
    查看detector是否能从相机图像中识别出装甲板，并同时与下位机保持通信

测试内容：
    1. camera_pub (单开一个线程或程序)
    2. camera_sub
    3. communication
    4. detector
    记录加入detector后装甲板识别的运行帧率，并保存至实验日志detector.log中
*/
#include "../../tasks/detector.hpp"
#include "../../include/structures.hpp"
#include "../../io/serial_manager.hpp"
#include "../../io/ros2_manager.hpp"
#include "../../io/keyboard_manager.hpp"
#include "../../tools/pharser.hpp"
#include "../../tools/draw.hpp"
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
    if (argc > 1) test_config_path = argv[1];  // 允许通过命令行参数指定配置文件路径
    
    TestConfig test_config = load_deploy_test_config(test_config_path);
    
    // 允许命令行参数覆盖配置文件中的值
    std::string yolo_model_path = test_config.yolo_model_path;
    std::string send_port = test_config.send_port;
    std::string receive_port = test_config.receive_port;


    // 打开日志文件
    std::ofstream log_file("../test/deploy/log/detector.log", std::ios::app);
    if (!log_file.is_open())
    {
        std::cerr << "Warning: Cannot open log file, logging to console only" << std::endl;
    }


    // 1、配置串口通信（已从配置文件加载）
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

    // 2、创建 ROS2 图像接受节点
    auto ros_node = std::make_shared<ROS2Manager>();
    std::thread spin_thread([&]() { rclcpp::spin(ros_node); });

    // 3、初始化键盘管理器（多线程处理键盘输入）
    io::KeyboardManager keyboard_manager(5.0f); // 每次调整5度
    keyboard_manager.init();

    // 4、初始化 Detector
    Detector detector(yolo_model_path);


    // 控制标志
    std::atomic<bool> running(true);

    io::KeyboardManager::print_controls();
    std::cout << "\nStarting detection loop..." << std::endl;

    // 启动键盘输入处理线程（保留多线程特性）
    keyboard_manager.start(running, [&running]() { running = false; });

    // 帧率统计
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    auto last_fps_time = start_time;
    auto last_send_time = start_time;
    const auto send_interval = std::chrono::milliseconds(50); // 每50ms发送一次命令
    double fps = 0.0;

    // 主循环：图像处理和检测
    while (running && rclcpp::ok())
    {
        cv::Mat img;
        if (ros_node->get_img(img))
        {
            if (img.empty())
            {
                continue;
            }

            frame_count++;
            auto frame_start = std::chrono::steady_clock::now();

            // step1: 检测阶段
            auto detect_start = std::chrono::steady_clock::now();
            ArmorArray armors = detector.detect(img);
            auto detect_end = std::chrono::steady_clock::now();
            double detect_time = std::chrono::duration<double, std::milli>(detect_end - detect_start).count();

            cv::Mat display_frame = img.clone();

            // 绘制检测结果
            drawArmorDetection(display_frame, armors);

            // 计算FPS
            auto current_time = std::chrono::steady_clock::now();
            auto fps_duration = std::chrono::duration<double>(current_time - last_fps_time).count();

            if (fps_duration > 0.5)
            { // 每0.5秒更新一次FPS
                fps = frame_count / std::chrono::duration<double>(current_time - start_time).count();
                last_fps_time = current_time;
            }

            // 显示性能信息
            drawPerformanceInfo(display_frame, fps, detect_time, 0.0);

            // 获取当前命令值（用于显示和日志）
            io::Command cmd = keyboard_manager.get_command();
            float current_yaw = cmd.yaw;
            float current_pitch = cmd.pitch;

            // 显示键盘控制信息
            std::string yaw_text = "Yaw: " + std::to_string(current_yaw).substr(0, 5) + " deg";
            std::string pitch_text = "Pitch: " + std::to_string(current_pitch).substr(0, 5) + " deg";

            // 定期记录日志（每秒一次）
            auto log_duration = std::chrono::duration<double>(current_time - start_time).count();
            if (log_duration >= 1.0)
            {
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");

                std::string log_entry = "[" + ss.str() + "] FPS: " + std::to_string(fps).substr(0, 5) + ", Detected: " + std::to_string(armors.size()) + ", Detect Time: " + std::to_string(detect_time).substr(0, 5) +
                                        "ms" + ", Yaw: " + std::to_string(current_yaw).substr(0, 5) + ", Pitch: " + std::to_string(current_pitch).substr(0, 5) + "\n";

                if (log_file.is_open())
                {
                    log_file << log_entry;
                    log_file.flush();
                }

                std::cout << log_entry;

                // 重置统计
                frame_count = 0;
                start_time = current_time;
            }

            // 定期发送命令（使用字符串协议）
            if (usb && (current_time - last_send_time) >= send_interval)
            {
                io::Command cmd_to_send = keyboard_manager.get_command();
                usb->send_command(cmd_to_send);
                last_send_time = current_time;
            }

            // 显示结果
            cv::imshow("Detector Test", display_frame);

            // 处理键盘输入
            char key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27)
            {
                running = false;
                break;
            }
        }
        else
        {
            // 如果没有新图像，短暂休眠
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

    std::cout << "\nDetector test finished." << std::endl;
    return 0;
}
