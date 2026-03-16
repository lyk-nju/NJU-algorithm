
/*
auto_aimer.cpp：
测试目标：
    测试完整的自动瞄准系统，包括检测、PnP解算、追踪和瞄准

测试内容：
    1. camera_pub (单开一个线程或程序)
    2. camera_sub
    3. communication
    4. detector
    5. pnp_solver
    6. tracker
    7. aimer
    记录装甲板检测和预测信息，并保存至实验日志auto_aimer.log中
*/

#include "../../io/keyboard_manager.hpp"
#include "../../io/ros2_manager.hpp"
#include "../../io/serial_manager.hpp"
#include "../../tasks/auto_aim_system.hpp"
#include "../../tools/draw.hpp"
#include "../../tools/pharser.hpp"
#include "../../tools/visualizer.hpp"
#include "../../tools/transfer.hpp"
#include <atomic>
#include <chrono>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <optional>
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
    double bullet_speed = test_config.bullet_speed;
    std::string send_port = test_config.send_port;
    std::string receive_port = test_config.receive_port;

    // 打开日志文件
    std::ofstream log_file("../test/deploy/log/auto_aimer.log", std::ios::app);

    // 1、创建 ROS2 节点
    auto ros_node = std::make_shared<ROS2Manager>();
    std::thread spin_thread([&]() { rclcpp::spin(ros_node); });

    // 2、配置串口通信
    io::USB *usb = nullptr;
    try
    {
        usb = new io::USB(send_port, receive_port);
    }
    catch (...)
    {
        std::cout << "Warning: Communication not available, continuing without it" << std::endl;
    }

    // 3、初始化键盘管理器（多线程处理键盘输入）
    io::KeyboardManager keyboard_manager(5.0f); // 每次调整5度
    keyboard_manager.init();

    // 4、初始化自瞄系统（封装检测、PnP、追踪、瞄准及 IMU 同步）
    AutoAimSystem auto_aim_system(yolo_model_path, config_path, bullet_speed);

    // 5、加载相机参数（用于弹道绘制）
    auto camera_params = loadCameraParameters(config_path);
    cv::Mat camera_matrix = camera_params.first;
    cv::Mat distort_coeffs = camera_params.second;

    // 9、初始化3D可视化器
    Visualizer3D visualizer(ros_node, "world");

    // 控制标志
    std::atomic<bool> running(true);

    // 目标切换冷却机制（防止摆头时提前开火）
    std::chrono::steady_clock::time_point last_switch_time = std::chrono::steady_clock::now();
    const double shoot_cooldown_ms = 250.0; // 目标切换后的开火冷却时间(ms)，可根据云台响应速度调整

    io::KeyboardManager::print_controls();
    // 启动键盘输入处理线程（保留多线程特性）
    keyboard_manager.start(running);

    // 帧率统计
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    auto last_fps_time = start_time;
    auto last_send_time = start_time;
    const auto send_interval = std::chrono::milliseconds(10); // 每10ms发送一次命令
    double fps = 0.0;

    // 自动瞄准相关变量
    std::optional<io::Command> latest_autoaim_cmd;
    AimPoint latest_aim_point{};
    double latest_autoaim_time_ms = 0.0;
    bool has_latest_imu = false;
    double latest_imu_yaw = 0.0;
    double latest_imu_pitch = 0.0;

    // 上下位机通信线程
    std::mutex cmd_mutex;
    io::Command latest_cmd_to_send;
    std::string latest_cmd_source = "keyboard";
    bool has_latest_cmd = false;

    // imu 接收线程和命令发送线程
    std::mutex imu_mutex;
    bool imu_has_data = false;
    Eigen::Quaterniond imu_quat(1.0, 0.0, 0.0, 0.0);
    std::thread imu_thread;
    std::thread send_thread;
    if (usb != nullptr)
    {
        imu_thread = std::thread(
            [&]()
            {
                // IMU接收线程的频率统计
                int imu_receive_count = 0;
                auto imu_receive_start_time = std::chrono::steady_clock::now();
                auto program_start_time = std::chrono::steady_clock::now(); // 记录程序启动时间，用于计算相对时间戳

                while (running)
                {
                    try
                    {
                        Eigen::Quaterniond quat;
                        double imu_yaw = 0.0;
                        double imu_pitch = 0.0;
                        if (usb->receive_quaternion(quat, imu_yaw, imu_pitch))
                        {
                            imu_receive_count++;

                            // 获取接收时刻的时间戳
                            auto receive_time = std::chrono::steady_clock::now();
                            auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(receive_time - program_start_time).count();

                            // 获取系统时间（精确到毫秒）
                            auto sys_time = std::chrono::system_clock::now();
                            auto sys_time_t = std::chrono::system_clock::to_time_t(sys_time);
                            auto sys_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(sys_time.time_since_epoch()).count() % 1000;

                            // 每10次打印一次详细数据（避免输出过多）
                            if (imu_receive_count % 10 == 0)
                            {
                                std::ostringstream oss;
                                oss << std::put_time(std::localtime(&sys_time_t), "%H:%M:%S");
                                oss << "." << std::setfill('0') << std::setw(3) << sys_time_ms;
                            }

                            // 存入自瞄系统的 IMU 历史缓冲（用于图像-IMU 时间戳同步）
                            auto_aim_system.updateImu(quat, imu_yaw, imu_pitch);

                            // 同时更新最新值（用于日志显示）
                            {
                                std::lock_guard<std::mutex> lock(imu_mutex);
                                imu_quat = quat;
                                latest_imu_yaw = imu_yaw;
                                latest_imu_pitch = imu_pitch;
                                imu_has_data = true;
                                has_latest_imu = true;
                            }

                            // 每1秒统计一次IMU接收频率
                            auto now = std::chrono::steady_clock::now();
                            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - imu_receive_start_time).count();
                            if (elapsed >= 1000)
                            {
                                double imu_receive_hz = imu_receive_count * 1000.0 / elapsed;
                                // std::cout << "[IMU Thread] Actual receive freq: " << std::fixed << std::setprecision(1) << imu_receive_hz << " Hz"
                                //           << " (received " << imu_receive_count << " packets in " << elapsed << "ms)" << std::endl;
                                imu_receive_count = 0;
                                imu_receive_start_time = now;
                            }
                        }
                    }
                    catch (...)
                    {
                    }
                    // 移除sleep以提高IMU接收频率
                    // std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            });
        send_thread = std::thread(
            [&]()
            {
                auto last_send_time_thread = std::chrono::steady_clock::now();
                while (running)
                {
                    io::Command cmd_to_send;
                    std::string cmd_source;
                    bool has_cmd = false;
                    {
                        std::lock_guard<std::mutex> lock(cmd_mutex);
                        if (has_latest_cmd)
                        {
                            cmd_to_send = latest_cmd_to_send;
                            cmd_source = latest_cmd_source;
                            has_cmd = true;
                        }
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (has_cmd && (now - last_send_time_thread) >= send_interval)
                    {
                        // 若有 /cmd_vel，则一起打包进 base_Command 下发
                        geometry_msgs::msg::Twist cmd_vel;
                        if (ros_node->get_cmd_vel(cmd_vel))
                        {
                            io::base_Command base_cmd = tools::from_cmd_vel(cmd_vel);
                            usb->send_command(cmd_to_send, base_cmd);
                        }
                        else
                        {
                            usb->send_command(cmd_to_send);
                        }
                        {
                            int valid_i = cmd_to_send.valid ? 1 : 0;
                            int shoot_i = cmd_to_send.shoot ? 1 : 0;
                            std::ostringstream ss;
                            ss.setf(std::ios::fixed);
                            ss.precision(6);
                            ss << cmd_source << " valid=" << valid_i << " shoot=" << shoot_i << " yaw=" << cmd_to_send.yaw << " pitch=" << cmd_to_send.pitch;
                            {
                                std::lock_guard<std::mutex> lock(imu_mutex);
                                if (has_latest_imu)
                                {
                                    ss << " imu_yaw=" << latest_imu_yaw << " imu_pitch=" << latest_imu_pitch;
                                }
                            }
                            ss << "\n";
                            std::cout << ss.str();
                        }
                        last_send_time_thread = now;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            });
    }
    io::Command last_cmd_to_send;
    io::Command last_keyboard_cmd;
    std::string last_cmd_source = "none";
    bool has_last_cmd = false;
    bool has_last_keyboard = false;

    while (running && rclcpp::ok())
    {
        cv::Mat img;
        std::chrono::steady_clock::time_point image_timestamp;

        // 使用带时间戳的图像获取方法
        if (ros_node->get_img_with_timestamp(img, image_timestamp))
        {
            if (img.empty())
            {
                continue;
            }

            frame_count++;
            auto frame_start = std::chrono::steady_clock::now();

            // 自瞄处理（检测 + PnP + 追踪 + 瞄准，IMU 同步在 AutoAimSystem 内部完成）
            ProcessResult result = auto_aim_system.processFrame(img, image_timestamp);

            // 目标切换冷却（防止摆头时提前开火）
            bool is_switching = result.is_switching;
            static bool last_is_switching = false;
            if (last_is_switching && !is_switching)
            {
                last_switch_time = std::chrono::steady_clock::now();
            }
            last_is_switching = is_switching;

            auto time_since_switch = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - last_switch_time).count();

            // 根据目标切换状态决定是否开火
            if (!result.targets.empty())
            {
                io::Command auto_cmd = result.cmd;
                bool can_shoot = false;
                if (is_switching)
                {
                    can_shoot = false;
                }
                else if (result.tracker_state == "tracking")
                {
                    can_shoot = (time_since_switch >= shoot_cooldown_ms);
                }
                auto_cmd.shoot = can_shoot;

                latest_autoaim_cmd = auto_cmd;
                latest_aim_point = result.aim_point;
                latest_autoaim_time_ms = result.aim_time_ms;

                // 通过 ROS2 话题发布 vision 结果（此时暂不附加 judge 数据）
                io::JudgerData empty_judge_data;
                ros_node->publish_vision_result(auto_cmd, empty_judge_data);
            }
            else
            {
                // 没有目标时发布 invalid 指令
                io::Command invalid_cmd;
                invalid_cmd.valid = false;
                invalid_cmd.shoot = false;
                invalid_cmd.yaw = 0.0f;
                invalid_cmd.pitch = 0.0f;

                latest_autoaim_cmd.reset();
                latest_aim_point.valid = false;
                latest_aim_point.xyza.setZero();
                latest_autoaim_time_ms = 0.0;

                io::JudgerData empty_judge_data;
                ros_node->publish_vision_result(invalid_cmd, empty_judge_data);
            }

            const auto &armors = result.armors;
            const auto &targets = result.targets;
            double detect_time = result.detect_time_ms;
            double track_time = result.track_time_ms;

            // 准备显示帧
            cv::Mat display_frame = img.clone();

            // 绘制检测结果
            drawArmorDetection(display_frame, armors);

            // 绘制Target详细信息（包括预测的装甲板）
            drawTargetInfo(display_frame, targets, result.tracker_state, auto_aim_system.getPnpSolver());

            // 绘制弹道轨迹（如果有效）
            if (latest_aim_point.valid)
            {
                drawTrajectory(display_frame, latest_aim_point, bullet_speed, config_path, camera_matrix, distort_coeffs, auto_aim_system.getPnpSolver().R_gimbal2world_);
            }

            // 发布3D可视化数据到RViz2
            // 发布相机位置（世界坐标系）
            // visualizer.publishCameraPosition(pnp_solver);
            // std::cout << "[RViz2] Published camera position" << std::endl;

            // 发布装甲板位置（世界坐标系）
            if (!armors.empty())
            {
                visualizer.publishArmorsWorld(armors);
                // std::cout << "[RViz2] Published " << armors.size() << " armors" << std::endl;
            }

            // 发布目标预测位置
            if (!targets.empty())
            {
                // visualizer.publishTargets(targets);

                // // 打印中心位置和装甲板位置
                // const auto &target = targets[0];
                // if (target.isinit)
                // {
                //     Eigen::VectorXd ekf_x = target.ekf_x();
                //     if (ekf_x.size() >= 9)
                //     {
                //         // std::cout << "[EKF Center] World: (" << ekf_x[0] << ", " << ekf_x[2] << ", " << ekf_x[4] << ")"
                //         //           << " | yaw: " << ekf_x[6] << " rad | r: " << ekf_x[8] << "m" << std::endl;

                //         // 打印4个装甲板位置
                //         auto xyza_list = target.armor_xyza_list();
                //         // for (size_t i = 0; i < xyza_list.size(); ++i)
                //         // {
                //         //     std::cout << "  [Armor " << i << "] World: (" << xyza_list[i][0] << ", " << xyza_list[i][1] << ", " << xyza_list[i][2] << ") | angle: " << xyza_list[i][3] << " rad" << std::endl;
                //         // }

                //         // 验证：计算4个装甲板的几何中心
                //         double avg_x = 0, avg_y = 0, avg_z = 0;
                //         for (const auto &xyza : xyza_list)
                //         {
                //             avg_x += xyza[0];
                //             avg_y += xyza[1];
                //             avg_z += xyza[2];
                //         }
                //         avg_x /= xyza_list.size();
                //         avg_y /= xyza_list.size();
                //         avg_z /= xyza_list.size();
                //         std::cout << "[Armor Center] Avg: (" << avg_x << ", " << avg_y << ", " << avg_z << ")" << std::endl;
                //         std::cout << "[Diff] EKF-Avg: (" << (ekf_x[0] - avg_x) << ", " << (ekf_x[2] - avg_y) << ", " << (ekf_x[4] - avg_z) << ")" << std::endl;
                //     }
                // }
            }
            // else
            // {
            //     std::cout << "[RViz2] No targets to publish" << std::endl;
            // }

            // // 发布瞄准点（如果有效）
            // if (latest_aim_point.valid)
            // {
            //     Eigen::Vector3d aim_xyz = latest_aim_point.xyza.head<3>();
            //     visualizer.publishAimPoint(aim_xyz, {0.0, 1.0, 0.0});
            // }

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

            // 显示自动瞄准状态
            int aim_state_y = display_frame.rows - 100;
            if (aim_state_y < 40) aim_state_y = 40;
            int aim_angle_y = aim_state_y + 25;
            int aim_meta_y = aim_angle_y + 25;
            cv::Scalar aim_color = (latest_autoaim_cmd && latest_autoaim_cmd->valid) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

            std::string aim_state_text = "AutoAim: ";
            if (!latest_autoaim_cmd)
            {
                aim_state_text += "No target";
            }
            else if (latest_autoaim_cmd->valid)
            {
                aim_state_text += latest_autoaim_cmd->shoot ? "Shoot" : "Locked";
            }
            else
            {
                aim_state_text += "Invalid";
            }
            tools::draw_text(display_frame, aim_state_text, cv::Point(10, aim_state_y), aim_color, 0.55, 2);

            if (latest_autoaim_cmd)
            {
                std::ostringstream angles_ss;
                angles_ss << std::fixed << std::setprecision(2);
                angles_ss << "Yaw:" << latest_autoaim_cmd->yaw << "deg  ";
                angles_ss << "Pitch:" << latest_autoaim_cmd->pitch << "deg";
                tools::draw_text(display_frame, angles_ss.str(), cv::Point(10, aim_angle_y), aim_color, 0.5, 1);

                std::ostringstream meta_ss;
                meta_ss << std::fixed << std::setprecision(2);
                meta_ss << "AimT:" << latest_autoaim_time_ms << "ms";
                if (latest_aim_point.valid)
                {
                    Eigen::Vector3d aim_xyz = latest_aim_point.xyza.head<3>();
                    meta_ss << " Dist:" << aim_xyz.norm() << "m";
                }
                tools::draw_text(display_frame, meta_ss.str(), cv::Point(10, aim_meta_y), aim_color, 0.5, 1);
            }
            else
            {
                tools::draw_text(display_frame, "Yaw/Pitch: N/A", cv::Point(10, aim_angle_y), cv::Scalar(200, 200, 200), 0.5, 1);
                tools::draw_text(display_frame, "AimT: --", cv::Point(10, aim_meta_y), cv::Scalar(200, 200, 200), 0.5, 1);
            }

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
                    if (armor.p_camera.norm() > 0) // 只有 tracker 解算过的 armor 才有 p_camera
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
                    if (1)
                    {
                        // 获取目标的 EKF 状态（包含中心位置）
                        Eigen::VectorXd ekf_x = target.ekf_x();
                        if (ekf_x.size() >= 9)
                        {
                            // ekf_x[0]=center_x, ekf_x[2]=center_y, ekf_x[4]=center_z
                            // ekf_x[1]=vx, ekf_x[3]=vy, ekf_x[5]=vz
                            // ekf_x[6]=yaw, ekf_x[7]=yaw_v, ekf_x[8]=r
                            Eigen::Vector3d center(ekf_x[0], ekf_x[2], ekf_x[4]);
                            Eigen::Vector3d velocity(ekf_x[1], ekf_x[3], ekf_x[5]);
                            double yaw_rad = ekf_x[6];
                            double yaw_v = ekf_x[7];

                            log_entry += "  Target[" + std::to_string(i) + "] CarNum:" + std::to_string(target.car_num) + " Center:(" + std::to_string(center.x()).substr(0, 6) + "," +
                                         std::to_string(center.y()).substr(0, 6) + "," + std::to_string(center.z()).substr(0, 6) + ")" + " Vel:(" + std::to_string(velocity.x()).substr(0, 5) + "," +
                                         std::to_string(velocity.y()).substr(0, 5) + "," + std::to_string(velocity.z()).substr(0, 5) + ")" + " Yaw:" + std::to_string(yaw_rad * 180 / CV_PI).substr(0, 6) + "deg" +
                                         " YawV:" + std::to_string(yaw_v * 180 / CV_PI).substr(0, 6) + "deg/s\n";
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
                                             ", State: " + result.tracker_state + ", Detect: " + std::to_string(detect_time).substr(0, 5) + "ms" + ", Track: " + std::to_string(track_time).substr(0, 5) + "ms";

                // 添加冷却时间信息
                auto time_since_last_switch = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - last_switch_time).count();
                if (time_since_last_switch < shoot_cooldown_ms * 2) // 在冷却期或刚结束时显示
                {
                    console_output += ", Cooldown: " + std::to_string(time_since_last_switch).substr(0, 5) + "ms";
                }

                if (latest_autoaim_cmd && latest_autoaim_cmd->valid)
                {
                    console_output += ", Aim: " + std::to_string(latest_autoaim_time_ms).substr(0, 5) + "ms";
                    console_output += ", Yaw: " + std::to_string(latest_autoaim_cmd->yaw * 180.0 / CV_PI).substr(0, 5) + "deg";
                    console_output += ", Pitch: " + std::to_string(latest_autoaim_cmd->pitch * 180.0 / CV_PI).substr(0, 5) + "deg";
                }
                console_output += "\n";
                // std::cout << console_output;

                // 重置统计
                frame_count = 0;
                start_time = current_time;
            }

            // 更新要发送的命令（实际发送在独立线程中完成）
            if (usb)
            {
                io::Command cmd_to_send;
                std::string cmd_source = "keyboard";
                io::Command keyboard_cmd = keyboard_manager.get_command();
                bool keyboard_changed = false;
                if (!has_last_keyboard)
                {
                    keyboard_changed = true;
                }
                else
                {
                    keyboard_changed = (keyboard_cmd.valid != last_keyboard_cmd.valid) || (keyboard_cmd.shoot != last_keyboard_cmd.shoot) || (keyboard_cmd.yaw != last_keyboard_cmd.yaw) ||
                                       (keyboard_cmd.pitch != last_keyboard_cmd.pitch);
                }
                last_keyboard_cmd = keyboard_cmd;
                has_last_keyboard = true;
                if (latest_autoaim_cmd && latest_autoaim_cmd->valid)
                {
                    // 基于自瞄结果提供 valid / yaw / pitch；shoot 完全由本节点 cooldown 决定，不再来自 decision
                    cmd_to_send = *latest_autoaim_cmd;
                    cmd_source = keyboard_cmd.shoot ? "auto_aim+p" : "auto_aim";

                    last_cmd_to_send = cmd_to_send;
                    last_cmd_source = cmd_source;
                    has_last_cmd = true;
                }
                else if (keyboard_changed)
                {
                    cmd_to_send = keyboard_cmd;
                    cmd_source = "keyboard";
                    last_cmd_to_send = cmd_to_send;
                    last_cmd_source = cmd_source;
                    has_last_cmd = true;
                }
                else if (has_last_cmd)
                {
                    cmd_to_send = last_cmd_to_send;
                    cmd_source = last_cmd_source;
                }
                else
                {
                    cmd_to_send = keyboard_cmd;
                    cmd_source = "keyboard";
                    last_cmd_to_send = cmd_to_send;
                    last_cmd_source = cmd_source;
                    has_last_cmd = true;
                }

                {
                    std::lock_guard<std::mutex> lock(cmd_mutex);
                    latest_cmd_to_send = cmd_to_send;
                    latest_cmd_source = cmd_source;
                    has_latest_cmd = true;
                }
            }

            // 显示结果
            cv::imshow("Auto Aimer Test", display_frame);

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

    if (imu_thread.joinable())
    {
        imu_thread.join();
    }
    if (send_thread.joinable())
    {
        send_thread.join();
    }

    keyboard_manager.restore(); // 恢复终端设置
    rclcpp::shutdown();
    spin_thread.join();
    cv::destroyAllWindows();

    // std::cout << "\nAuto aimer test finished." << std::endl;
    return 0;
}
