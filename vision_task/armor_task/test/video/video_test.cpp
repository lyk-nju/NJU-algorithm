#include "../include/armor.hpp"
#include "../tools/draw.hpp"
#include "../tools/pharser.hpp"
#include "../tools/visualizer.hpp"
#include "aimer.hpp"
#include "detector.hpp"
#include "pnp_solver.hpp"
#include "tracker.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <list>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <sstream>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace armor_task;

int main(int argc, char *argv[])
{
    // 初始化 ROS2（用于3D可视化）
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("video_test_node");
    std::thread spin_thread([&]() { rclcpp::spin(ros_node); });

    try
    {
        // 从配置文件加载配置
        std::string test_config_path = "../config/video_test.yaml";
        if (argc > 1) test_config_path = argv[1]; // 允许通过命令行参数指定配置文件路径

        TestConfig test_config = load_video_test_config(test_config_path);

        // 允许命令行参数覆盖配置文件中的值
        std::string model_path = test_config.yolo_model_path;
        std::string config_path = test_config.config_path;
        std::string input_video_path = test_config.video_path;
        double bullet_speed = test_config.bullet_speed;

        std::cout << "Video source : " << input_video_path << std::endl;
        std::cout << "YOLO model   : " << model_path << std::endl;
        std::cout << "Config path  : " << config_path << std::endl;
        std::cout << "Bullet speed : " << bullet_speed << " m/s" << std::endl;

        // 加载相机参数
        auto camera_params = loadCameraParameters(config_path);
        cv::Mat camera_matrix = camera_params.first;
        cv::Mat distort_coeffs = camera_params.second;
        std::cout << "Camera parameters loaded from config" << std::endl;

        // 实例化功能组件
        std::cout << "Initializing components..." << std::endl;
        Detector detector(model_path);
        PnpSolver pnp_solver(config_path);

        // 在视频测试中，设置固定的云台姿态（单位四元数，表示云台坐标系 = 世界坐标系）
        // 模拟每次更新 R_gimbal2world_，但使用固定值
        Eigen::Quaterniond fixed_quat(1.0, 0.0, 0.0, 0.0); // 单位四元数 (w, x, y, z)
        pnp_solver.set_R_gimbal2world(fixed_quat);
        std::cout << "Initialized R_gimbal2world with identity quaternion (gimbal = world)" << std::endl;

        Tracker tracker(config_path, pnp_solver);
        Aimer aimer(config_path);

        // 初始化3D可视化器
        Visualizer3D visualizer(ros_node, "world");
        std::cout << "3D Visualizer initialized. Start RViz2 to view markers." << std::endl;

        // 打开视频
        cv::VideoCapture cap(input_video_path);
        if (!cap.isOpened())
        {
            std::cerr << "Error: Cannot open video file: " << input_video_path << std::endl;
            return -1;
        }

        // 获取视频信息
        int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        double video_fps = cap.get(cv::CAP_PROP_FPS);
        int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));

        std::cout << "Video Info:" << std::endl;
        std::cout << "  Resolution: " << frame_width << "x" << frame_height << std::endl;
        std::cout << "  FPS: " << video_fps << std::endl;
        std::cout << "  Total Frames: " << total_frames << std::endl;

        cv::Mat frame;
        int frame_count = 0;
        auto start_time = std::chrono::steady_clock::now();
        auto last_fps_time = start_time;
        double fps = 0.0;

        // 10fps播放控制
        constexpr double target_fps = 10.0;
        constexpr double target_frame_time_ms = 1000.0 / target_fps; //

        std::cout << "Starting processing..." << std::endl;
        std::cout << "Press 'q' to quit, 'p' to pause/resume, SPACE to step frame" << std::endl;
        std::cout << "Target playback speed: " << target_fps << " fps" << std::endl;

        // 随机选择30帧进行temp_lost测试
        std::set<int> temp_lost_frames;
        std::srand(std::time(nullptr));
        while (temp_lost_frames.size() < 70)
        {
            int random_frame = std::rand() % total_frames + 1;
            temp_lost_frames.insert(random_frame);
        }
        std::cout << "\n=== TEMP_LOST TEST ===" << std::endl;
        std::cout << "Will simulate temp_lost on 30 random frames: ";
        for (int f : temp_lost_frames)
        {
            std::cout << f << " ";
        }
        std::cout << "\n========================\n" << std::endl;

        bool paused = false;
        std::optional<io::Command> latest_autoaim_cmd;
        AimPoint latest_aim_point{};
        double latest_autoaim_time_ms = 0.0;

        while (true)
        {
            auto frame_start = std::chrono::steady_clock::now();
            double frame_start_s = std::chrono::duration<double>(frame_start.time_since_epoch()).count();
            std::cout << "frame_start: " << frame_start_s << " s" << std::endl;

            if (!paused)
            {
                if (!cap.read(frame))
                {
                    std::cout << "End of video or failed to read frame" << std::endl;
                    break;
                }
                frame_count++;
            }

            if (frame.empty()) continue;

            cv::Mat display_frame = frame.clone();

            if (!paused)
            {
                // 检测阶段
                std::cout << "Processing frame " << frame_count << "/" << total_frames << std::endl;
                auto detect_start = std::chrono::steady_clock::now();
                ArmorArray detected_armors = detector.detect(frame);
                auto detect_end = std::chrono::steady_clock::now();
                double detect_time = std::chrono::duration<double, std::milli>(detect_end - detect_start).count();

                // 模拟temp_lost: 如果当前帧在随机选择的帧中,清空检测结果
                bool is_temp_lost_frame = temp_lost_frames.find(frame_count) != temp_lost_frames.end();
                if (is_temp_lost_frame)
                {
                    std::cout << "\n*** [TEMP_LOST SIMULATION] Frame " << frame_count << " - Clearing detections ***\n" << std::endl;
                    detected_armors.clear();
                }

                // 模拟更新云台到世界坐标系的旋转矩阵（视频测试中使用固定值）
                // 在视频测试中，相机坐标系 = 世界坐标系，所以使用单位四元数
                pnp_solver.set_R_gimbal2world(fixed_quat);

                // 追踪阶段（内部会进行 PnP 解算）
                auto track_start = std::chrono::steady_clock::now();
                auto targets = tracker.track(detected_armors, frame_start);
                std::cout << "targets.size(): " << targets.size() << std::endl;

                // 调试输出：追踪后的装甲板信息（已经过 PnP 解算）
                if (!detected_armors.empty())
                {
                    std::cout << "\n=== PnP Results (after tracking) | Frame " << frame_count << " ===" << std::endl;
                    for (size_t i = 0; i < detected_armors.size(); ++i)
                    {
                        const auto &armor = detected_armors[i];
                        if (armor.p_camera.norm() > 0) // 只显示解算成功的装甲板
                        {
                            std::cout << " | Armor " << i << "] ID:" << armor.car_num << " | p_world: (" << armor.p_world.x() << ", " << armor.p_world.y() << ", " << armor.p_world.z() << ")"
                                      << " | yaw: " << armor.ypr_in_world(0) * 180 / CV_PI << "°" << std::endl;
                        }
                    }
                    std::cout << "====================================\n" << std::endl;
                }
                // for (const auto &target : targets)
                // {
                //     Eigen::VectorXd ekf_x = target.ekf_x();
                //     if (ekf_x.size() >= 5)
                //     {
                //         std::cout << "target car_num: " << target.car_num << ", pos: (" << ekf_x[0] << ", " << ekf_x[2] << ", " << ekf_x[4] << ")" << std::endl;
                //     }
                // }
                auto track_end = std::chrono::steady_clock::now();
                double track_time = std::chrono::duration<double, std::milli>(track_end - track_start).count();

                if (!targets.empty())
                {
                    std::list<Target> target_list(targets.begin(), targets.end());
                    auto aim_start = std::chrono::steady_clock::now();
                    io::Command auto_cmd = aimer.aim(target_list, frame_start, bullet_speed);
                    auto aim_end = std::chrono::steady_clock::now();
                    latest_autoaim_time_ms = std::chrono::duration<double, std::milli>(aim_end - aim_start).count();
                    latest_autoaim_cmd = auto_cmd;
                    latest_aim_point = aimer.debug_aim_point;

                    if (frame_count % 30 == 0)
                    {
                        double yaw_deg = auto_cmd.yaw * 180.0 / CV_PI;
                        double pitch_deg = auto_cmd.pitch * 180.0 / CV_PI;
                        // std::ostringstream aim_log;
                        // aim_log << std::boolalpha << "[AutoAim] valid=" << auto_cmd.valid << " shoot=" << auto_cmd.shoot;
                        // aim_log << std::fixed << std::setprecision(2);
                        // aim_log << " yaw=" << yaw_deg << "deg pitch=" << pitch_deg << "deg";
                        // aim_log << " latency=" << latest_autoaim_time_ms << "ms";
                        // std::cout << "\n" << aim_log.str() << std::endl;

                        if (latest_aim_point.valid)
                        {
                            Eigen::Vector3d aim_xyz = latest_aim_point.xyza.head<3>();
                            std::ostringstream point_log;
                            point_log << std::fixed << std::setprecision(2);
                            point_log << "[AutoAim] aim_xyz(m)=(" << aim_xyz.x() << ", " << aim_xyz.y() << ", " << aim_xyz.z() << ") dist=" << aim_xyz.norm();
                            // std::cout << point_log.str() << std::endl;
                        }
                    }
                }
                else
                {
                    latest_autoaim_cmd.reset();
                    latest_aim_point.valid = false;
                    latest_aim_point.xyza.setZero();
                    latest_autoaim_time_ms = 0.0;
                }

                // 绘制检测结果
                drawArmorDetection(display_frame, detected_armors);

                // 绘制Target详细信息
                drawTargetInfo(display_frame, targets, tracker.state(), pnp_solver);

                // 如果是temp_lost模拟帧，在屏幕上显示警告
                if (is_temp_lost_frame)
                {
                    cv::putText(display_frame, "*** TEMP_LOST SIMULATION ***", cv::Point(display_frame.cols / 2 - 200, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 3);
                }

                // 绘制弹道轨迹（使用 AimPoint，基于目标位置）
                if (latest_aim_point.valid)
                {
                    drawTrajectory(display_frame, latest_aim_point, bullet_speed, config_path, camera_matrix, distort_coeffs, pnp_solver.R_gimbal2world_);
                }

                // 发布3D可视化数据到RViz2
                // 发布相机位置（世界坐标系）
                visualizer.publishCameraPosition(pnp_solver);

                // 发布装甲板位置（世界坐标系）
                // visualizer.publishArmorsWorld(detected_armors);

                // 发布目标预测位置
                if (!targets.empty())
                {
                    std::cout << "publishTargets" << std::endl;
                    visualizer.publishTargets(targets);
                }

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
                    fps = 1.0 / std::chrono::duration<double>(current_time - frame_start).count();
                    last_fps_time = current_time;
                }

                // std::cout << "fps is " << fps << std::endl;

                // // 显示性能信息
                drawPerformanceInfo(display_frame, fps, detect_time, track_time);

                // 控制台输出
                if (frame_count % 30 == 0)
                {
                    // std::cout << "\rProcessing frame " << frame_count << "/" << total_frames << " | FPS: " << std::setprecision(3) << fps << " | Detected: " << detected_armors.size() << " | Tracking: " <<
                    // targets.size()
                    //           << " | State: " << tracker.state() << std::flush;
                }
            }

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
            cv::putText(display_frame, aim_state_text, cv::Point(10, aim_state_y), cv::FONT_HERSHEY_SIMPLEX, 0.55, aim_color, 2);

            if (latest_autoaim_cmd)
            {
                std::ostringstream angles_ss;
                angles_ss << std::fixed << std::setprecision(2);
                angles_ss << "Yaw:" << latest_autoaim_cmd->yaw * 180.0 / CV_PI << "deg  ";
                angles_ss << "Pitch:" << latest_autoaim_cmd->pitch * 180.0 / CV_PI << "deg";
                cv::putText(display_frame, angles_ss.str(), cv::Point(10, aim_angle_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, aim_color, 1);

                std::ostringstream meta_ss;
                meta_ss << std::fixed << std::setprecision(2);
                meta_ss << "AimT:" << latest_autoaim_time_ms << "ms";
                if (latest_aim_point.valid)
                {
                    Eigen::Vector3d aim_xyz = latest_aim_point.xyza.head<3>();
                    meta_ss << " Dist:" << aim_xyz.norm() << "m";
                }
                cv::putText(display_frame, meta_ss.str(), cv::Point(10, aim_meta_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, aim_color, 1);
            }
            else
            {
                cv::putText(display_frame, "Yaw/Pitch: N/A", cv::Point(10, aim_angle_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
                cv::putText(display_frame, "AimT: --", cv::Point(10, aim_meta_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
            }

            std::string progress = "Frame: " + std::to_string(frame_count) + "/" + std::to_string(total_frames);
            cv::putText(display_frame, progress, cv::Point(10, display_frame.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            // 显示结果
            cv::imshow("Auto Aim Test - Target Visualization", display_frame);
            std::string save_dir = "/home/lai/大学/RoboMaster/NJU_RMVision/images/"; // 你想要保存的路径

            std::string filename = save_dir + "frame_" + std::to_string(frame_count) + ".png";

            // 保存
            cv::imwrite(filename, display_frame);

            // 10fps播放控制：计算处理耗时，如果小于目标时间则等待
            if (!paused)
            {
                auto frame_end = std::chrono::steady_clock::now();
                double frame_processing_time_ms = std::chrono::duration<double, std::milli>(frame_end - frame_start).count();
                double wait_time_ms = target_frame_time_ms - frame_processing_time_ms;

                // 如果处理时间超过目标时间，立即显示下一帧；否则等待剩余时间
                int wait_key_ms = static_cast<int>(std::max(1.0, wait_time_ms));
                char key = cv::waitKey(wait_key_ms) & 0xFF;

                // 处理键盘输入
                if (key == 'q' || key == 27)
                { // 'q' 或 ESC 退出
                    break;
                }
                else if (key == 'p')
                { // 'p' 暂停/恢复
                    paused = !paused;
                    std::cout << (paused ? "\nPaused" : "\nResumed") << std::endl;
                }
            }
            else
            {
                // 暂停模式下，等待任意按键
                char key = cv::waitKey(0) & 0xFF;
                if (key == 'q' || key == 27)
                { // 'q' 或 ESC 退出
                    break;
                }
                else if (key == 'p')
                { // 'p' 暂停/恢复
                    paused = !paused;
                    std::cout << (paused ? "\nPaused" : "\nResumed") << std::endl;
                }
                else if (key == ' ')
                { // 空格键单步执行
                    if (cap.read(frame))
                    {
                        frame_count++;
                    }
                }
            }
        }

        // 计算总体统计信息
        auto end_time = std::chrono::steady_clock::now();
        auto total_duration = std::chrono::duration<double>(end_time - start_time).count();
        double avg_fps = frame_count / total_duration;

        std::cout << "\n\nProcessing completed!" << std::endl;
        std::cout << "Total frames processed: " << frame_count << std::endl;
        std::cout << "Total time: " << std::setprecision(3) << total_duration << " seconds" << std::endl;
        std::cout << "Average FPS: " << std::setprecision(3) << avg_fps << std::endl;

        // 清理资源
        cap.release();
        cv::destroyAllWindows();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        spin_thread.join();
        return -1;
    }

    // 清理 ROS2
    rclcpp::shutdown();
    spin_thread.join();

    std::cout << "OpenCV CUDA devices found: " << cv::cuda::getCudaEnabledDeviceCount() << std::endl;
    std::cout << "OpenCV CUDA devices found: " << cv::cuda::getCudaEnabledDeviceCount() << std::endl;

    return 0;
}