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
#include "../../tasks/aimer.hpp"
#include "../../tasks/detector.hpp"
#include "../../tasks/pnp_solver.hpp"
#include "../../tasks/tracker.hpp"
#include "../../tools/draw.hpp"
#include "../../tools/pharser.hpp"
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

// IMU历史数据结构（轻量级）
struct IMUTimestamp
{
    std::chrono::steady_clock::time_point time;
    Eigen::Quaterniond quat;
    double yaw;
    double pitch;
};

// IMU历史缓冲（轻量级实现）
class IMUHistory
{
  private:
    std::deque<IMUTimestamp> buffer_;
    mutable std::mutex mutex_;
    static constexpr size_t MAX_SIZE = 100; // 缓冲100个数据点（约100ms）

  public:
    // 添加新的IMU数据
    void push(const Eigen::Quaterniond &quat, double yaw, double pitch)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        IMUTimestamp data;
        data.time = std::chrono::steady_clock::now();
        data.quat = quat;
        data.yaw = yaw;
        data.pitch = pitch;
        buffer_.push_back(data);

        // 限制缓冲区大小
        if (buffer_.size() > MAX_SIZE)
        {
            buffer_.pop_front();
        }
    }

    // 查询指定时间最近的IMU数据
    bool query(const std::chrono::steady_clock::time_point &target_time, Eigen::Quaterniond &out_quat, double &out_yaw, double &out_pitch)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) return false;

        // 如果目标时间早于所有数据，返回最早的
        if (target_time <= buffer_.front().time)
        {
            out_quat = buffer_.front().quat;
            out_yaw = buffer_.front().yaw;
            out_pitch = buffer_.front().pitch;
            return true;
        }

        // 如果目标时间晚于所有数据，返回最新的
        if (target_time >= buffer_.back().time)
        {
            out_quat = buffer_.back().quat;
            out_yaw = buffer_.back().yaw;
            out_pitch = buffer_.back().pitch;
            return true;
        }

        // 二分查找最接近的数据
        auto it = std::lower_bound(buffer_.begin(), buffer_.end(), target_time, [](const IMUTimestamp &data, const std::chrono::steady_clock::time_point &t) { return data.time < t; });

        // 比较前后两个点，选择时间差更小的
        if (it != buffer_.begin())
        {
            auto prev = std::prev(it);
            auto dt_prev = std::abs(std::chrono::duration_cast<std::chrono::microseconds>(prev->time - target_time).count());
            auto dt_curr = std::abs(std::chrono::duration_cast<std::chrono::microseconds>(it->time - target_time).count());

            if (dt_prev < dt_curr)
            {
                it = prev;
            }
        }

        out_quat = it->quat;
        out_yaw = it->yaw;
        out_pitch = it->pitch;
        return true;
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size();
    }
};

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

    // 4、初始化检测器
    Detector detector(yolo_model_path);

    // 5、初始化 PnP 解算器
    PnpSolver pnp_solver(config_path);

    // 6、初始化追踪器
    Tracker tracker(config_path, pnp_solver);

    // 7、初始化瞄准器
    Aimer aimer(config_path);

    // 8、加载相机参数（用于弹道绘制）
    auto camera_params = loadCameraParameters(config_path);
    cv::Mat camera_matrix = camera_params.first;
    cv::Mat distort_coeffs = camera_params.second;

    // 9、初始化3D可视化器
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
    auto last_send_time = start_time;
    const auto send_interval = std::chrono::milliseconds(10); // 每10ms发送一次命令
    double fps = 0.0;
    int fps_frame_count = 0;
    DrawMode draw_mode = show_full_preview ? DrawMode::full : DrawMode::off;

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

    std::mutex image_mutex;
    std::condition_variable image_cv;
    std::shared_ptr<const ROS2Manager::FramePacket> latest_image_packet;
    uint64_t latest_image_id = 0;
    uint64_t consumed_image_id = 0;
    bool image_thread_done = false;

    // IMU历史缓冲器（用于时间戳同步）
    IMUHistory imu_history;

    // imu 接收线程和命令发送线程
    std::mutex imu_mutex;
    bool imu_has_data = false;
    Eigen::Quaterniond imu_quat(1.0, 0.0, 0.0, 0.0);
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

                            // 存入历史缓冲（带时间戳）
                            imu_history.push(quat, imu_yaw, imu_pitch);

                            // 同时更新最新值（向后兼容）
                            {
                                std::lock_guard<std::mutex> lock(imu_mutex);
                                imu_quat = quat;
                                latest_imu_yaw = imu_yaw;
                                latest_imu_pitch = imu_pitch;
                                imu_has_data = true;
                                has_latest_imu = true;
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
                    catch (...)
                    {
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
                            if (cmd_source.find("auto_aim") == std::string::npos) {
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

        {
            if (img.empty())
            {
                continue;
            }

            frame_count++;
            fps_frame_count++;
            auto frame_start = std::chrono::steady_clock::now();

            // 使用真实的图像时间戳
            // image_timestamp 是图像曝光时刻的准确时间
            auto estimated_exposure_time = image_timestamp;

            // 计算图像延迟（用于调试）
            auto image_delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(frame_start - image_timestamp).count();

            static int delay_print_count = 0;
            // if (++delay_print_count % 100 == 0)
            // {
            //     std::cout << "[Image Delay] Current delay: " << image_delay_ms << "ms" << std::endl;
            // }

            // step1: 检测阶段
            auto detect_start = std::chrono::steady_clock::now();
            ArmorArray armors = detector.detect(img);
            std::cout << "armorsize detect"<< armors.size()<<std::endl;
            auto detect_end = std::chrono::steady_clock::now();
            double detect_time = std::chrono::duration<double, std::milli>(detect_end - detect_start).count();
            for (const auto &armor : armors)
            {
                std::cout << "Confidence" << armor.confidence << std::endl; 
            }

            // step2: 更新云台到世界坐标系的旋转矩阵（从IMU历史缓冲查询）

            static Eigen::Quaterniond last_quat(1.0, 0.0, 0.0, 0.0);
            static int quat_update_count = 0;
            static int debug_frame_count = 0;
            static auto last_imu_stats_time = std::chrono::steady_clock::now();
            static int imu_updates_in_period = 0;

            if (usb)
            {
                Eigen::Quaterniond synced_quat;
                double synced_yaw, synced_pitch;
   
                // 关键改进：查询图像曝光时刻的IMU数据，而不是最新的
                if (imu_history.query(estimated_exposure_time, synced_quat, synced_yaw, synced_pitch))
                {
                    // 计算四元数变化量（用于调试）
                    // double quat_diff = (synced_quat.coeffs() - last_quat.coeffs()).norm();

                    pnp_solver.set_R_gimbal2world(synced_quat);
                    last_quat = synced_quat;
                    quat_update_count++;
                    debug_frame_count++;
                    imu_updates_in_period++;

                    // 每1秒打印一次IMU更新频率
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_imu_stats_time).count();
                    if (elapsed >= 1000)
                    {
                        double imu_update_hz = imu_updates_in_period * 1000.0 / elapsed;
                        // std::cout << "[IMU Stats] Update freq: " << std::fixed << std::setprecision(1) << imu_update_hz << " Hz"
                        //           << " | Buffer size: " << imu_history.size() << " | Quat change: " << std::scientific << std::setprecision(2) << quat_diff << " | Total frames: " << debug_frame_count << std::endl;

                        last_imu_stats_time = now;
                        imu_updates_in_period = 0;
                    }
                }
                else
                {
                    // 降级方案：如果查询失败，使用最新IMU
                    std::lock_guard<std::mutex> lock(imu_mutex);
                    if (imu_has_data)
                    {
                        pnp_solver.set_R_gimbal2world(imu_quat);
                        last_quat = imu_quat;
                    }
                }
            }
            else
            {
                // 没有下位机时，使用单位四元数（相当于 video_test 的情况）
                static bool once = true;
                if (once)
                {
                    Eigen::Quaterniond identity_quat(1.0, 0.0, 0.0, 0.0);
                    pnp_solver.set_R_gimbal2world(identity_quat);
                    // std::cout << "[IMU] No USB connection, using identity quaternion" << std::endl;
                    once = false;
                }
            }

            // step3: 追踪阶段（tracker 内部会自动进行 PnP 解算）
            // 更新敌人颜色
            {
                std::lock_guard<std::mutex> lock(imu_mutex);
                if (has_latest_imu) {
                    bool self_is_red = tools::get_color_from_self_id(latest_judger_data);
                    std::cout << "is red: " << self_is_red << std::endl;
                    tracker.get_enemy_color(self_is_red);
                }
            }
            
            auto track_start = std::chrono::steady_clock::now();
            std::vector<Target> targets = tracker.track(armors, frame_start);
            // std::cout << "armorsize track"<< armors.size()<<std::endl;

            auto track_end = std::chrono::steady_clock::now();
            double track_time = std::chrono::duration<double, std::milli>(track_end - track_start).count();




            // 调试输出：追踪后的装甲板信息（已经过 PnP 解算）
                // if (!armors.empty())
                // {
                //     std::cout << "\n=== PnP Results (after tracking) | Frame " << frame_count << " ===" << std::endl;
                //     for (size_t i = 0; i < armors.size(); ++i)
                //     {
                //         const auto &armor = armors[i];
                        
                //         if (armor.p_camera.norm() > 0) // 只显示解算成功的装甲板
                //         {
                //             std::cout << " | Armor " << i << "] ID:" << armor.car_num << " | p_world: (" << armor.p_world.x() << ", " << armor.p_world.y() << ", " << armor.p_world.z() << ")"
                //                       << " | yaw: " << armor.ypr_in_world(0) * 180 / CV_PI << "°" << std::endl;
                //         }
                //     }
                //     std::cout << "====================================\n" << std::endl;
                // }


            // step4: 瞄准阶段（增加延迟补偿）
            io::Command auto_cmd;
            double aim_time = 0.0;

            // 检测目标切换状态（使用target的is_switch_成员）
            bool is_switching = false;
            if (!targets.empty())
            {
                is_switching = targets[0].is_switch_;
            }

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
                std::list<Target> target_list(targets.begin(), targets.end());
                auto aim_start = std::chrono::steady_clock::now();

                auto_cmd = aimer.aim(target_list, frame_start);

                // 开火逻辑修改：TODO，白色装甲板不打
                bool can_shoot = false;

                if (is_switching)
                {
                    // 正在切换目标，禁止开火
                    can_shoot = false;
                    // std::cout << "[Shoot Control] Target switching, shoot disabled" << std::endl;
                }
                else if (tracker.state() == "tracking")
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
                        // 可选：打印调试信息
                        static int cooldown_log_count = 0;
                        // if (++cooldown_log_count % 10 == 0)
                        // {
                        //     // std::cout << "[Shoot Control] In cooldown: " << std::fixed << std::setprecision(1) << time_since_switch << "ms / " << shoot_cooldown_ms << "ms" << std::endl;
                        // }
                    }
                }
                else
                {
                    // detecting, temp_lost, lost 状态不开火
                    can_shoot = false;
                }

                auto_cmd.shoot = auto_cmd.valid;

                auto aim_end = std::chrono::steady_clock::now();
                aim_time = std::chrono::duration<double, std::milli>(aim_end - aim_start).count();
                latest_autoaim_cmd = auto_cmd;
                latest_aim_point = aimer.debug_aim_point;
                latest_autoaim_time_ms = aim_time;
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
                drawArmorDetection(display_frame, armors);
                drawTargetInfo(display_frame, targets, tracker.state(), pnp_solver);
                if (latest_aim_point.valid)
                {
                    tools::drawTrajectory(display_frame, latest_aim_point, bullet_speed, config_path, camera_matrix, distort_coeffs, pnp_solver.R_gimbal2world_);
                }
            }

            // 发布3D可视化数据到RViz2
            // 发布相机位置（世界坐标系）
            // visualizer.publishCameraPosition(pnp_solver);
            // std::cout << "[RViz2] Published camera position" << std::endl;

            // 发布装甲板位置（世界坐标系）
            //if (!armors.empty())
            //{
                //visualizer.publishArmorsWorld(armors);
                // std::cout << "[RViz2] Published " << armors.size() << " armors" << std::endl;
            //}

            // 发布目标预测位置
            // if (!targets.empty())
            // {
            //     //visualizer.publishTargets(targets);

            //     // 打印中心位置和装甲板位置
            //     const auto &target = targets[0];
            //     if (target.isinit)
            //     {
            //         Eigen::VectorXd ekf_x = target.ekf_x();
            //         if (ekf_x.size() >= 9)
            //         {
            //             std::cout << "[EKF Center] World: (" << ekf_x[0] << ", " << ekf_x[2] << ", " << ekf_x[4] << ")"
            //                       << " | yaw: " << ekf_x[6] << " rad | r: " << ekf_x[8] << "m" << std::endl;

            //         //     // 打印4个装甲板位置
            //         //     auto xyza_list = target.armor_xyza_list();
            //         //     for (size_t i = 0; i < xyza_list.size(); ++i)
            //         //     {
            //         //         std::cout << "  [Armor " << i << "] World: (" << xyza_list[i][0] << ", " << xyza_list[i][1] << ", " << xyza_list[i][2] << ") | angle: " << xyza_list[i][3] << " rad" << std::endl;
            //         //     }

            //         //     // 验证：计算4个装甲板的几何中心
            //         //     double avg_x = 0, avg_y = 0, avg_z = 0;
            //         //     for (const auto &xyza : xyza_list)
            //         //     {
            //         //         avg_x += xyza[0];
            //         //         avg_y += xyza[1];
            //         //         avg_z += xyza[2];
            //         //     }
            //         //     avg_x /= xyza_list.size();
            //         //     avg_y /= xyza_list.size();
            //         //     avg_z /= xyza_list.size();
            //         //     std::cout << "[Armor Center] Avg: (" << avg_x << ", " << avg_y << ", " << avg_z << ")" << std::endl;
            //         //     std::cout << "[Diff] EKF-Avg: (" << (ekf_x[0] - avg_x) << ", " << (ekf_x[2] - avg_y) << ", " << (ekf_x[4] - avg_z) << ")" << std::endl;
            //          }
            //     }
            // }
            // else
            // {
            //     std::cout << "[RViz2] No targets to publish" << std::endl;
            // }

            // //发布瞄准点（如果有效）
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

            //     std::string log_entry = "[" + ss.str() + "]\n";

                // 记录每个检测到的装甲板实际位置
                for (const auto &armor : armors)
                {
                    if (armor.p_camera.norm() > 0) // 只有 tracker 解算过的 armor 才有 p_camera
                    {
                        // 使用 ypr_in_world 而不是 armor.yaw（armor.yaw 已被注释掉）
                        double yaw_deg = armor.ypr_in_world(0) * 180.0 / CV_PI;
                        double pitch_deg = armor.ypr_in_world(1) * 180.0 / CV_PI;

                        // log_entry += "  Detected Armor ID:" + std::to_string(armor.detect_id) + " Num:" + std::to_string(armor.car_num) + " Pos:(" + std::to_string(armor.p_camera.x()).substr(0, 6) + "," +
                        //              std::to_string(armor.p_camera.y()).substr(0, 6) + "," + std::to_string(armor.p_camera.z()).substr(0, 6) + ")" + " Yaw:" + std::to_string(yaw_deg).substr(0, 6) + "deg" +
                        //              " Pitch:" + std::to_string(pitch_deg).substr(0, 6) + "deg\n";
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
                            Eigen::Vector3d center(ekf_x[0], ekf_x[2], ekf_x[4]);
                            Eigen::Vector3d velocity(ekf_x[1], ekf_x[3], ekf_x[5]);
                            double yaw_rad = ekf_x[6];
                            double yaw_v = ekf_x[7];

                            // log_entry += "  Target[" + std::to_string(i) + "] CarNum:" + std::to_string(target.car_num) + " Center:(" + std::to_string(center.x()).substr(0, 6) + "," +
                            //              std::to_string(center.y()).substr(0, 6) + "," + std::to_string(center.z()).substr(0, 6) + ")" + " Vel:(" + std::to_string(velocity.x()).substr(0, 5) + "," +
                            //              std::to_string(velocity.y()).substr(0, 5) + "," + std::to_string(velocity.z()).substr(0, 5) + ")" + " Yaw:" + std::to_string(yaw_rad * 180 / CV_PI).substr(0, 6) + "deg" +
                            //              " YawV:" + std::to_string(yaw_v * 180 / CV_PI).substr(0, 6) + "deg/s\n";
                        }

                        // 获取预测的所有装甲板位置
                        auto xyza_list = target.armor_xyza_list();
                        for (size_t j = 0; j < xyza_list.size(); ++j)
                        {
                            const auto &xyza = xyza_list[j];
                            // log_entry += "    Armor[" + std::to_string(j) + "] Pos:(" + std::to_string(xyza(0)).substr(0, 6) + "," + std::to_string(xyza(1)).substr(0, 6) + "," + std::to_string(xyza(2)).substr(0, 6) +
                            //              ") Yaw:" + std::to_string(xyza(3) * 180 / CV_PI).substr(0, 6) + "deg\n";
                        }
                    }
                }

                // 只在有装甲板信息时才写入日志
                // if (log_entry.length() > ss.str().length() + 3) // 检查是否有装甲板信息（除了时间戳和换行符）
                // {
                //     if (log_file.is_open())
                //     {
                //         log_file << log_entry;
                //         log_file.flush();
                //     }
                // }

                //控制台输出仍然显示系统信息（用于调试）
                std::string console_output = "[" + ss.str() + "] FPS: " + std::to_string(fps).substr(0, 5) + ", Detected: " + std::to_string(armors.size()) + ", Tracking: " + std::to_string(targets.size()) +
                                             ", State: " + tracker.state() + ", Detect: " + std::to_string(detect_time).substr(0, 5) + "ms" + ", Track: " + std::to_string(track_time).substr(0, 5) + "ms";

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
                    cmd_to_send = *latest_autoaim_cmd;
                    // 注意：shoot 状态已经在 aim 阶段根据 tracker 状态设置好了
                    // 不再强制设置为 1，保留根据状态机判断的结果
                    // cmd_to_send.shoot = 1;
                    // cmd_to_send.shoot = keyboard_cmd.shoot;
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
            // if (has_display)
            // {
            //     if (should_draw)
            //     {
            //         cv::imshow("Auto Aimer Test", display_frame);
            //         cv::pollKey();
            //     }
            // }
            // cv::imshow("Auto Aimer Test", display_frame);
            // cv::pollKey();
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
