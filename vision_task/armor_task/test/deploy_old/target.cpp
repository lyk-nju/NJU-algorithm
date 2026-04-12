/*
target.cpp：
测试目标：
    查看target是否能正确预测装甲板位姿，并保持与下位机保持通信

测试内容：
    1. camera_pub (单开一个线程或程序)
    2. camera_sub
    3. communication
    4. detector
    5. pnp_solver
    6. target
    记录加入target后装甲板识别与姿态解算的运行帧率、装甲板实际位置及预测结果，并保存至实验日志target.log中

实验方法：
    1. 相机静止，目标装甲板保持平动
    2. 相机平动，目标装甲板保持静止
    3. 相机静止，目标装甲板旋转
    4. 相机小幅度旋转，目标装甲板保持静止
*/

#include "../../io/keyboard_manager.hpp"
#include "../../io/ros2_manager.hpp"
#include "../../io/serial_manager.hpp"
#include "../../tasks/detector.hpp"
#include "../../tasks/pnp_solver.hpp"
#include "../../tasks/tracker.hpp"
#include "../../tools/draw.hpp"
#include "../../tools/pharser.hpp"
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
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
    std::ofstream log_file("../test/deploy/log/target.log", std::ios::app);
    if (!log_file.is_open())
    {
        std::cerr << "Warning: Cannot open log file, logging to console only" << std::endl;
    }

    // 1、创建 ROS2 图像接受节点
    auto ros_node = std::make_shared<ROS2Manager>();
    std::thread spin_thread([&]() { rclcpp::spin(ros_node); });

    // 2、配置串口通信
    io::USB *usb = nullptr;
    try
    {
        usb = new io::USB(send_port, receive_port);
        // std::cout << "Communication initialized with ports: " << send_port << ", " << receive_port << std::endl;
    }
    catch (...)
    {
        // std::cout << "Warning: Communication not available, continuing without it" << std::endl;
    }

    // 3、初始化键盘管理器（多线程处理键盘输入）
    io::KeyboardManager keyboard_manager(5.0f); // 每次调整5度
    keyboard_manager.init();

    // 4、初始化检测器

    Detector detector(yolo_model_path);

    // 5、初始化 PnP 解算器
    PnpSolver pnp_solver(config_path);

    // 6、初始化追踪器
    Tracker tracker(config_path, pnp_solver);

    // 控制标志
    std::atomic<bool> running(true);

    io::KeyboardManager::print_controls();
    // std::cout << "\nStarting target tracking test loop..." << std::endl;

    // 启动键盘输入处理线程（保留多线程特性）
    keyboard_manager.start(running);

    // 帧率统计
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    auto last_fps_time = start_time;
    auto last_send_time = start_time;
    const auto send_interval = std::chrono::milliseconds(50); // 每50ms发送一次命令
    double fps = 0.0;
    io::Command last_cmd = keyboard_manager.get_command();
    std::mutex imu_mutex;
    bool imu_has_data = false;
    Eigen::Quaterniond imu_quat(1.0, 0.0, 0.0, 0.0);
    double imu_yaw = 0.0;
    double imu_pitch = 0.0;

    std::thread imu_thread;
    if (usb != nullptr)
    {
        imu_thread = std::thread(
            [&]()
            {
                while (running)
                {
                    try
                    {
                        Eigen::Quaterniond quat;
                        double yaw = 0.0;
                        double pitch = 0.0;
                        if (usb->receive_quaternion(quat, yaw, pitch))
                        {
                            std::lock_guard<std::mutex> lock(imu_mutex);
                            imu_quat = quat;
                            imu_yaw = yaw;
                            imu_pitch = pitch;
                            imu_has_data = true;
                        }
                    }
                    catch (...)
                    {
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            });
    }

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

            // step2: 更新云台到世界坐标系的旋转矩阵（从下位机接收IMU四元数）
            if (usb != nullptr)
            {
                std::lock_guard<std::mutex> lock(imu_mutex);
                if (imu_has_data)
                {
                    pnp_solver.set_R_gimbal2world(imu_quat);
                    Eigen::Vector3d euler = pnp_solver.R_gimbal2world_.eulerAngles(2, 1, 0);
                    std::cout << "[IMU] yaw=" << std::fixed << std::setprecision(4) << imu_yaw << " pitch=" << imu_pitch << std::endl;
                    std::cout << "[HOST] yaw=" << std::fixed << std::setprecision(4) << euler[0] << " pitch=" << euler[1] << std::endl;
                }
            }

            // step3: 追踪阶段（tracker 内部会自动进行 PnP 解算）
            auto track_start = std::chrono::steady_clock::now();
            std::vector<Target> targets = tracker.track(armors, frame_start);
            auto track_end = std::chrono::steady_clock::now();
            double track_time = std::chrono::duration<double, std::milli>(track_end - track_start).count();

            if (!armors.empty())
            {
                // std::cout << "\n=== PnP Results (after tracking) ===" << std::endl;
                for (size_t i = 0; i < armors.size(); ++i)
                {
                    const auto &armor = armors[i];
                    if (armor.p_camera.norm() > 0) // 只显示解算成功的装甲板
                    {
                        // std::cout << "[Detected Armor " << i << "] detect_id:" << armor.detect_id << " car_num:" << armor.car_num << std::endl;
                        // std::cout << "  p_world:  (" << std::fixed << std::setprecision(4) << armor.p_world.x() << ", " << armor.p_world.y() << ", " << armor.p_world.z() << ")" << std::endl;
                        // std::cout << "  ypr_in_world: yaw=" << std::fixed << std::setprecision(2) << armor.ypr_in_world(0) * 180 / CV_PI << "° "
                        //           << "pitch=" << armor.ypr_in_world(1) * 180 / CV_PI << "°" << std::endl;
                    }
                }

                // 输出 Target 的 EKF 状态，重点关注 y 方向
                for (size_t i = 0; i < targets.size(); ++i)
                {
                    const auto &target = targets[i];
                    if (target.isinit)
                    {
                        Eigen::VectorXd ekf_x = target.ekf_x();
                        if (ekf_x.size() >= 9)
                        {
                            // std::cout << "\n[Target " << i << "] EKF State:" << std::endl;
                            // std::cout << "  Center: x=" << std::fixed << std::setprecision(4) << ekf_x[0] << " y=" << ekf_x[2] << " z=" << ekf_x[4] << std::endl;
                            // std::cout << "  Velocity: vx=" << std::fixed << std::setprecision(4) << ekf_x[1] << " vy=" << ekf_x[3] << " vz=" << ekf_x[5] << std::endl;
                            // std::cout << "  Yaw: " << std::fixed << std::setprecision(2) << ekf_x[6] * 180 / CV_PI << "° | YawV: " << ekf_x[7] * 180 / CV_PI << "°/s" << std::endl;
                            // std::cout << "  Radius: " << std::fixed << std::setprecision(4) << ekf_x[8] << "m" << std::endl;
                            // std::cout << "  Matched_ID: " << target.last_id << std::endl;

                            // 计算 y 方向偏移
                            double y_deviation = std::abs(ekf_x[2]);
                            if (y_deviation > 0.05)
                            {
                                // std::cout << "  ⚠️  Y-axis deviation: " << std::fixed << std::setprecision(4) << y_deviation << "m (>5cm)" << std::endl;
                            }
                        }
                    }
                }
                // std::cout << "====================================\n" << std::endl;
            }

            // 准备显示帧
            cv::Mat display_frame = img.clone();

            // 绘制检测结果
            drawArmorDetection(display_frame, armors);

            // 绘制Target详细信息（包括预测的装甲板）
            drawTargetInfo(display_frame, targets, tracker.state(), pnp_solver);

            // 计算FPS
            auto current_time = std::chrono::steady_clock::now();
            auto fps_duration = std::chrono::duration<double>(current_time - last_fps_time).count();

            if (fps_duration > 0.5)
            { // 每0.5秒更新一次FPS
                fps = frame_count / std::chrono::duration<double>(current_time - start_time).count();
                last_fps_time = current_time;
            }

            // 显示性能信息（注意：PnP 解算时间包含在 track_time 中）
            drawPerformanceInfo(display_frame, fps, detect_time, track_time);

            // 获取当前命令值（用于日志）
            // io::Command cmd = keyboard_manager.get_command();
            // float current_yaw = cmd.yaw;
            // float current_pitch = cmd.pitch;

            // 定期记录日志（每秒一次，只记录装甲板信息）
            auto log_duration = std::chrono::duration<double>(current_time - start_time).count();
            if (log_duration >= 1.0)
            {
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");

                std::string log_entry = "[" + ss.str() + "]\n";

                // 记录每个检测到的装甲板实际位置
                for (const auto &armor : armors)
                {
                    if (armor.p_camera.norm() > 0)
                    {
                        // 使用 ypr_in_world 而不是 armor.yaw（armor.yaw 已被注释掉）
                        double yaw_deg = armor.ypr_in_world(0) * 180.0 / CV_PI;
                        double pitch_deg = armor.ypr_in_world(1) * 180.0 / CV_PI;

                        log_entry += "  Detected Armor ID:" + std::to_string(armor.detect_id) + " Num:" + std::to_string(armor.car_num) + " Pos:(" + std::to_string(armor.p_camera.x()).substr(0, 6) + "," +
                                     std::to_string(armor.p_camera.y()).substr(0, 6) + "," + std::to_string(armor.p_camera.z()).substr(0, 6) + ")" + " Yaw:" + std::to_string(yaw_deg).substr(0, 6) + "deg" +
                                     " Pitch:" + std::to_string(pitch_deg).substr(0, 6) + "deg\n";
                    }
                }

                // 记录每个追踪目标的预测装甲板信息
                for (size_t i = 0; i < targets.size(); ++i)
                {
                    const auto &target = targets[i];
                    if (target.isinit)
                    {
                        // 获取 EKF 状态并记录中心点
                        Eigen::VectorXd ekf_x = target.ekf_x();
                        if (ekf_x.size() >= 9)
                        {
                            Eigen::Vector3d center(ekf_x[0], ekf_x[2], ekf_x[4]);
                            Eigen::Vector3d velocity(ekf_x[1], ekf_x[3], ekf_x[5]);

                            log_entry += "  Target[" + std::to_string(i) + "] CarNum:" + std::to_string(target.car_num) + " Center:(" + std::to_string(center.x()).substr(0, 6) + "," +
                                         std::to_string(center.y()).substr(0, 6) + "," + std::to_string(center.z()).substr(0, 6) + ")" + " Vel:(" + std::to_string(velocity.x()).substr(0, 5) + "," +
                                         std::to_string(velocity.y()).substr(0, 5) + "," + std::to_string(velocity.z()).substr(0, 5) + ")" + " Yaw:" + std::to_string(ekf_x[6] * 180 / CV_PI).substr(0, 6) + "deg\n";
                        }

                        // 获取预测的所有装甲板位置
                        auto xyza_list = target.armor_xyza_list();
                        for (size_t j = 0; j < xyza_list.size(); ++j)
                        {
                            const auto &xyza = xyza_list[j];
                            log_entry += "    Armor[" + std::to_string(j) + "] Pos:(" + std::to_string(xyza(0)).substr(0, 6) + "," + std::to_string(xyza(1)).substr(0, 6) + "," + std::to_string(xyza(2)).substr(0, 6) +
                                         ") Yaw:" + std::to_string(xyza(3) * 180 / CV_PI).substr(0, 6) + "deg\n";
                        }
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
                std::string console_output = "[" + ss.str() + "] FPS: " + std::to_string(fps).substr(0, 5) + ", Detected: " + std::to_string(armors.size()) + ", Tracking: " + std::to_string(targets.size()) +
                                             ", State: " + tracker.state() + ", Detect: " + std::to_string(detect_time).substr(0, 5) + "ms" + ", Track: " + std::to_string(track_time).substr(0, 5) + "ms\n";
                // std::cout << console_output;

                // 重置统计
                frame_count = 0;
                start_time = current_time;
            }

            // 定期发送命令（即使没有追踪到目标也发送，保持通信）
            if (usb && (current_time - last_send_time) >= send_interval)
            {
                io::Command cmd_to_send = keyboard_manager.get_command();
                bool cmd_changed = (cmd_to_send.yaw != last_cmd.yaw || cmd_to_send.pitch != last_cmd.pitch || cmd_to_send.valid != last_cmd.valid || cmd_to_send.shoot != last_cmd.shoot);
                // 如果有追踪目标，可以根据预测结果调整命令
                // 这里先使用键盘控制的值，后续可以根据target预测结果进行优化
                if (cmd_changed || (current_time - last_send_time) >= send_interval)
                {
                    usb->send_command(cmd_to_send);
                    last_cmd = cmd_to_send;
                    last_send_time = current_time;
                }
            }

            cv::imshow("Target Tracking Test", display_frame);
            // OpenCV 的 waitKey 也可以处理 'q' 键退出
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

    if (imu_thread.joinable())
    {
        imu_thread.join();
    }

    keyboard_manager.restore(); // 恢复终端设置
    rclcpp::shutdown();
    spin_thread.join();
    cv::destroyAllWindows();

    // std::cout << "\nTarget tracking test finished." << std::endl;
    return 0;
}
