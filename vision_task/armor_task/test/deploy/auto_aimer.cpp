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
#include "../../tools/parser.hpp"
#include "../../tools/transfer.hpp"
#include "../../tools/visualizer.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
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

namespace
{
enum class DrawMode
{
    off = 0,
    full = 1,
};

const char *drawModeName(DrawMode mode)
{
    switch (mode)
    {
    case DrawMode::off: return "OFF";
    case DrawMode::full: return "FULL";
    }
    return "UNKNOWN";
}
} // namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const bool has_display = []()
    {
        const char *display = std::getenv("DISPLAY");
        if (display != nullptr && display[0] != '\0') return true;

        const char *wayland_display = std::getenv("WAYLAND_DISPLAY");
        return wayland_display != nullptr && wayland_display[0] != '\0';
    }();

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
    std::unique_ptr<io::USB> usb;
    std::string correct_port = io::check_port();
    if (correct_port.empty())
    {
        std::cout << "Warning: No /dev/ttyACM0 or /dev/ttyACM1 found, continuing without communication" << std::endl;
    }
    else
    {
        std::cout << "Using serial port: " << correct_port << std::endl;
        try
        {
            usb = std::make_unique<io::USB>(correct_port, correct_port);
        }
        catch (const std::exception &e)
        {
            std::cout << "Warning: Communication not available, continuing without it: " << e.what() << std::endl;
        }
    }

    // 3、初始化键盘管理器（多线程处理键盘输入）
    io::KeyboardManager keyboard_manager(5.0f); // 每次调整5度
    keyboard_manager.init();

    AutoAimSystem auto_aim(yolo_model_path, config_path, bullet_speed);
    auto_aim.updateImu(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), 0.0, 0.0);

    // 4、加载相机参数（用于弹道绘制）
    auto camera_params = loadCameraParameters(config_path);
    cv::Mat camera_matrix = camera_params.first;
    cv::Mat distort_coeffs = camera_params.second;

    // 5、初始化3D可视化器（当前禁用）
    //Visualizer3D visualizer(ros_node, "world");

    // 控制标志
    std::atomic<bool> running(true);

    // 目标切换冷却机制（防止摆头时提前开火）
    std::chrono::steady_clock::time_point last_switch_time = std::chrono::steady_clock::now();
    const double shoot_cooldown_ms = 0.0; // 目标切换后的开火冷却时间(ms)，可根据云台响应速度调整

    io::KeyboardManager::print_controls();
    const char *show_env = std::getenv("AUTO_AIMER_SHOW");
    const bool show_full_preview = has_display && show_env != nullptr && show_env[0] == '1';
    if (has_display)
    {
        std::cout << "Display detected. Full preview is "
                  << (show_full_preview ? "enabled" : "disabled")
                  << " (set AUTO_AIMER_SHOW=1 to enable)." << std::endl;
    }
    else
    {
        std::cout << "No display detected, running in CLI mode without OpenCV windows." << std::endl;
    }

    io::JudgerData latest_judger_data;
    // 启动键盘输入处理线程（保留多线程特性）
    keyboard_manager.start(running);

    // 帧率统计
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    auto last_fps_time = start_time;
    const auto send_interval = std::chrono::milliseconds(10); // 每10ms发送一次命令
    double fps = 0.0;
    int fps_frame_count = 0;
    DrawMode draw_mode = show_full_preview ? DrawMode::full : DrawMode::off;

    // 自动瞄准相关变量
    std::optional<io::Command> latest_autoaim_cmd;
    AimPoint latest_aim_point{};
    double latest_autoaim_time_ms = 0.0;

    // 上下位机通信线程
    std::mutex cmd_mutex;
    io::Command latest_cmd_to_send;
    std::string latest_cmd_source = "keyboard";
    bool has_latest_cmd = false;

    std::mutex image_mutex;
    std::condition_variable image_cv;
    std::shared_ptr<const ROS2Manager::FramePacket> latest_image_packet;
    uint64_t latest_image_id = 0;
    uint64_t consumed_image_id = 0;
    bool image_thread_done = false;

    // imu 接收线程和命令发送线程
    std::mutex imu_mutex;
    std::thread imu_thread;
    std::thread send_thread;
    if (usb)
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
                        io::JudgerData judger_data;
                        if (usb->receive_all(quat, imu_yaw, imu_pitch, judger_data))
                        {

                            imu_receive_count++;

                            // std::cout << "Receive:" << "x" << quat.x() << " y" << quat.y() << " z" << quat.z() << " w" << quat.w() << " imu_yaw" << imu_yaw << " imu_pitch" << imu_pitch << " game_time: " << judger_data.game_time << " self_hp " << judger_data.self_hp << " self_id: " << judger_data.self_id << std::endl;
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

                            auto_aim.updateImu(quat, imu_yaw, imu_pitch);
                            auto_aim.updateJudgerData(judger_data);

                            {
                                std::lock_guard<std::mutex> lock(imu_mutex);
                                latest_judger_data = judger_data;
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
                    catch (const std::exception &e)
                    {
                        static int ex_count = 0;
                        if (++ex_count % 100 == 0)
                        {
                            std::cerr << "[IMU Thread] exception: " << e.what() << std::endl;
                        }
                    }
                    catch (...)
                    {
                        static int ex_count = 0;
                        if (++ex_count % 100 == 0)
                        {
                            std::cerr << "[IMU Thread] unknown exception" << std::endl;
                        }
                    }
                    
                }
            });
            // io::send thread
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
                        geometry_msgs::msg::Twist cmd_vel;
                        if (ros_node->get_cmd_vel(cmd_vel))
                        {
                            io::base_Command base_cmd = tools::from_cmd_vel(cmd_vel);
                            // 没有敌军时保持转圈
                            if (cmd_source == "keyboard") {
                                base_cmd.w_yaw = 0.03;
                            }
                            std::cout << "valid:" << cmd_to_send.valid << " shoot" << cmd_to_send.shoot << " v_x:" << base_cmd.v_x << "  v_y:" << base_cmd.v_y << "  w_yaw:" << base_cmd.w_yaw << std::endl;

                            usb->send_command(cmd_to_send, base_cmd);
                        }
                        else
                        {
                            io::base_Command base_cmd{0.0, 0.0, 0.0}; 
                            usb->send_command(cmd_to_send, base_cmd);
                            std::cout << "valid:" << cmd_to_send.valid << " shoot" << cmd_to_send.shoot << " yaw:" << cmd_to_send.yaw << "  pitch:" << cmd_to_send.pitch << std::endl;
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

    // 命令优先级选择：自瞄命令 > 键盘变化 > 上次命令
    // 返回 {cmd, source}，source 为 "auto_aim" / "auto_aim+p" / "keyboard"
    auto select_command = [&]() -> std::pair<io::Command, std::string>
    {
        const io::Command keyboard_cmd = keyboard_manager.get_command();
        const bool keyboard_changed = !has_last_keyboard ||
            (keyboard_cmd.valid != last_keyboard_cmd.valid) ||
            (keyboard_cmd.shoot != last_keyboard_cmd.shoot) ||
            (keyboard_cmd.yaw != last_keyboard_cmd.yaw) ||
            (keyboard_cmd.pitch != last_keyboard_cmd.pitch);
        last_keyboard_cmd = keyboard_cmd;
        has_last_keyboard = true;

        if (latest_autoaim_cmd && latest_autoaim_cmd->valid)
        {
            const std::string src = keyboard_cmd.shoot ? "auto_aim+p" : "auto_aim";
            last_cmd_to_send = *latest_autoaim_cmd;
            last_cmd_source = src;
            has_last_cmd = true;
            return {*latest_autoaim_cmd, src};
        }
        if (keyboard_changed || !has_last_cmd)
        {
            last_cmd_to_send = keyboard_cmd;
            last_cmd_source = "keyboard";
            has_last_cmd = true;
            return {keyboard_cmd, "keyboard"};
        }
        return {last_cmd_to_send, last_cmd_source};
    };

    std::thread image_thread([&]() {
        while (running && rclcpp::ok())
        {
            std::shared_ptr<const ROS2Manager::FramePacket> packet;
            if (ros_node->get_frame_packet(packet))
            {
                if (!packet || packet->frame.empty()) continue;
                {
                    std::lock_guard<std::mutex> lock(image_mutex);
                    latest_image_packet = std::move(packet);
                    latest_image_id = latest_image_packet->id;
                }
                image_cv.notify_one();
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        {
            std::lock_guard<std::mutex> lock(image_mutex);
            image_thread_done = true;
        }
        image_cv.notify_all();
    });

    while (running && rclcpp::ok())
    {
        std::shared_ptr<const ROS2Manager::FramePacket> image_packet;

        {
            std::unique_lock<std::mutex> lock(image_mutex);
            image_cv.wait(lock, [&]() { return latest_image_id != consumed_image_id || image_thread_done || !running || !rclcpp::ok(); });
            if ((latest_image_id == consumed_image_id && image_thread_done) || !running || !rclcpp::ok())
            {
                break;
            }
            image_packet = latest_image_packet;
            consumed_image_id = latest_image_id;
        }

        if (!image_packet || image_packet->frame.empty()) continue;
        const cv::Mat &img = image_packet->frame;
        auto image_timestamp = image_packet->timestamp;

        frame_count++;
        fps_frame_count++;
        auto frame_start = std::chrono::steady_clock::now();

        const auto result = auto_aim.processFrame(img, image_timestamp);
        ArmorArray armors = result.armors;
        std::vector<Target> targets = result.targets;
        const double detect_time = result.detect_time_ms;
        const double track_time = result.track_time_ms;
        const double aim_time = result.aim_time_ms;

        // 瞄准阶段（增加延迟补偿）
        io::Command auto_cmd = result.cmd;
        const bool is_switching = result.is_switching;

        // 检测切换完成（从切换状态退出）
        static bool last_is_switching = false;
        if (last_is_switching && !is_switching)
        {
            // 记录切换完成的时间点
            last_switch_time = std::chrono::steady_clock::now();
            // std::cout << "[Shoot Control] Target switch completed, entering cooldown for " << shoot_cooldown_ms << "ms" << std::endl;
        }
        last_is_switching = is_switching;

        // 计算距离上次切换的时间
        auto time_since_switch = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - last_switch_time).count();

        if (!targets.empty())
        {
            auto_cmd = result.cmd;

            // 开火逻辑修改：TODO，白色装甲板不打
            bool can_shoot = false;

            if (is_switching)
            {
                // 正在切换目标，禁止开火
                can_shoot = false;
                // std::cout << "[Shoot Control] Target switching, shoot disabled" << std::endl;
            }
            else if (result.tracker_state == TrackerState::TRACKING)
            {
                // tracking 状态：需要检查冷却时间
                // 只有距离上次切换超过冷却时间才允许开火
                if (time_since_switch >= shoot_cooldown_ms)
                {
                    can_shoot = true;
                }
                else
                {
                    can_shoot = false;
                }
            }
            else
            {
                // detecting, temp_lost, lost 状态不开火
                can_shoot = false;
            }

            auto_cmd.shoot = can_shoot && auto_cmd.valid;
            latest_autoaim_cmd = auto_cmd;
            latest_autoaim_time_ms = aim_time;
            latest_aim_point = result.aim_point;
        }
        else
        {
            latest_autoaim_cmd.reset();
            latest_aim_point.valid = false;
            latest_aim_point.xyza.setZero();
            latest_autoaim_time_ms = 0.0;
            has_last_cmd = false;
        }

        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            bool cmd_valid = latest_autoaim_cmd.has_value() && latest_autoaim_cmd->valid;
            const io::AimerData aim_result_pub = tools::from_vis_dec(cmd_valid, latest_judger_data);
            ros_node->update_aimer_data(aim_result_pub);
        }

        // 准备显示帧（仅在需要可视化时进行绘制）
        const bool should_draw = has_display && draw_mode == DrawMode::full;
        cv::Mat display_frame = img;
        auto draw_start = std::chrono::steady_clock::now();

        if (should_draw)
        {
            const auto &pnp_solver = auto_aim.getPnpSolver();
            drawArmorDetection(display_frame, armors);
            drawTargetInfo(display_frame, targets, result.tracker_state_name(), pnp_solver);
            if (latest_aim_point.valid)
            {
                drawTrajectory(display_frame, latest_aim_point, bullet_speed, config_path, camera_matrix, distort_coeffs, pnp_solver.gimbal2world());
            }
        }

        // 计算FPS
        auto current_time = std::chrono::steady_clock::now();
        auto fps_duration = std::chrono::duration<double>(current_time - last_fps_time).count();

        if (fps_duration > 0.5)
        { // 每0.5秒更新一次FPS
            fps = fps_frame_count / fps_duration;
            fps_frame_count = 0;
            last_fps_time = current_time;
        }

        auto draw_end = std::chrono::steady_clock::now();
        double draw_time_ms = std::chrono::duration<double, std::milli>(draw_end - draw_start).count();
        double total_time_ms = std::chrono::duration<double, std::milli>(draw_end - frame_start).count();

        if (should_draw)
        {
            drawPerformanceInfo(display_frame, fps, detect_time, track_time);
            std::ostringstream perf_ss;
            perf_ss << std::fixed << std::setprecision(2);
            perf_ss << "Aim:" << aim_time << "ms Draw:" << draw_time_ms << "ms Total:" << total_time_ms << "ms";
            tools::draw_text(display_frame, perf_ss.str(), cv::Point(10, 95), cv::Scalar(255, 255, 255), 0.5, 1);
            tools::draw_text(display_frame, std::string("DrawMode: ") + drawModeName(draw_mode), cv::Point(10, 120), cv::Scalar(0, 255, 255), 0.5, 1);
        }

        if (should_draw)
        {
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
        }

        // // 定期记录日志（每秒一次，只记录装甲板信息）
        auto log_duration = std::chrono::duration<double>(current_time - start_time).count();
        if (log_duration >= 1.0)
        {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");

            // 控制台输出系统信息
            std::string console_output = "[" + ss.str() + "] FPS: " + std::to_string(fps).substr(0, 5) + ", Detected: " + std::to_string(armors.size()) + ", Tracking: " + std::to_string(targets.size()) +
                                         ", State: " + std::string(result.tracker_state_name()) + ", Detect: " + std::to_string(detect_time).substr(0, 5) + "ms" + ", Track: " + std::to_string(track_time).substr(0, 5) + "ms";

            // 添加冷却时间信息
            auto time_since_last_switch = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - last_switch_time).count();
            if (time_since_last_switch < shoot_cooldown_ms * 2) // 在冷却期或刚结束时显示
            {
                console_output += ", Cooldown: " + std::to_string(time_since_last_switch).substr(0, 5) + "ms";
            }

            if (latest_autoaim_cmd && latest_autoaim_cmd->valid)
            {
                console_output += ", Aim: " + std::to_string(aim_time).substr(0, 5) + "ms";
                console_output += ", Yaw: " + std::to_string(latest_autoaim_cmd->yaw * 180.0 / CV_PI).substr(0, 5) + "deg";
                console_output += ", Pitch: " + std::to_string(latest_autoaim_cmd->pitch * 180.0 / CV_PI).substr(0, 5) + "deg";
            }
            console_output += "\n";
            std::cout << console_output;

            // 重置统计
            frame_count = 0;
            start_time = current_time;
        }

        // 更新要发送的命令（实际发送在独立线程中完成）
        if (usb)
        {
            auto [cmd_to_send, cmd_source] = select_command();
            std::lock_guard<std::mutex> lock(cmd_mutex);
            latest_cmd_to_send = cmd_to_send;
            latest_cmd_source = cmd_source;
            has_latest_cmd = true;
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
        usb.reset();
    }

    if (imu_thread.joinable())
    {
        imu_thread.join();
    }
    if (send_thread.joinable())
    {
        send_thread.join();
    }
    if (image_thread.joinable())
    {
        image_thread.join();
    }

    keyboard_manager.restore(); // 恢复终端设置
    rclcpp::shutdown();
    spin_thread.join();
    if (has_display)
    {
        cv::destroyAllWindows();
    }

    // std::cout << "\nAuto aimer test finished." << std::endl;
    return 0;

}
