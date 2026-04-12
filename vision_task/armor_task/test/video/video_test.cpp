#include "../include/armor.hpp"
#include "../tools/draw.hpp"
#include "../tools/pharser.hpp"
#include "aimer.hpp"
#include "detector.hpp"
#include "pnp_solver.hpp"
#include "tracker.hpp"
#include <atomic>
#include <Eigen/Dense>
#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <optional>
#include <sstream>
#include <thread>

using namespace armor_task;

namespace
{
enum class DrawMode
{
    Off,
    Lite,
    Full,
};

struct StageStats
{
    double fps = 0.0;
    double detect_ms = 0.0;
    double track_ms = 0.0;
    double aim_ms = 0.0;
    double draw_ms = 0.0;
    double total_ms = 0.0;
};

struct VideoFramePacket
{
    cv::Mat frame;
    int frame_index = 0;
    bool eof = false;
};

DrawMode next_draw_mode(DrawMode mode)
{
    if (mode == DrawMode::Off) return DrawMode::Lite;
    if (mode == DrawMode::Lite) return DrawMode::Full;
    return DrawMode::Off;
}

const char *draw_mode_name(DrawMode mode)
{
    if (mode == DrawMode::Off) return "OFF";
    if (mode == DrawMode::Lite) return "LITE";
    return "FULL";
}

void draw_stage_stats(cv::Mat &img, const StageStats &stats, DrawMode mode, int frame_index)
{
    int y = 25;
    auto put = [&](const std::string &text, const cv::Scalar &color, double scale = 0.55, int thickness = 1)
    {
        cv::putText(img, text, cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness);
        y += 22;
    };

    std::ostringstream fps_ss;
    fps_ss << std::fixed << std::setprecision(1) << "FPS: " << stats.fps << "  Draw: " << draw_mode_name(mode);
    put(fps_ss.str(), cv::Scalar(0, 255, 0), 0.6, 2);

    std::ostringstream stage_ss;
    stage_ss << std::fixed << std::setprecision(2) << "detect " << stats.detect_ms << "  track " << stats.track_ms << "  aim " << stats.aim_ms;
    put(stage_ss.str(), cv::Scalar(255, 255, 255));

    std::ostringstream total_ss;
    total_ss << std::fixed << std::setprecision(2) << "draw " << stats.draw_ms << "  total " << stats.total_ms << "  frame " << frame_index;
    put(total_ss.str(), cv::Scalar(255, 255, 255));
}
} // namespace

int main(int argc, char *argv[])
{
    try
    {
        std::string test_config_path = "../config/video_test.yaml";
        if (argc > 1) test_config_path = argv[1];

        const TestConfig test_config = load_video_test_config(test_config_path);
        const std::string model_path = test_config.yolo_model_path;
        const std::string config_path = test_config.config_path;
        const std::string input_video_path = test_config.video_path;
        const double bullet_speed = test_config.bullet_speed;

        auto camera_params = loadCameraParameters(config_path);
        const cv::Mat &camera_matrix = camera_params.first;
        const cv::Mat &distort_coeffs = camera_params.second;

        Detector detector(model_path);
        PnpSolver pnp_solver(config_path);
        pnp_solver.set_R_gimbal2world(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
        Tracker tracker(config_path, pnp_solver);
        Aimer aimer(config_path);

        cv::VideoCapture cap(input_video_path);
        if (!cap.isOpened())
        {
            std::cerr << "Error: Cannot open video file: " << input_video_path << std::endl;
            return -1;
        }

        std::cout << "Video source : " << input_video_path << std::endl;
        std::cout << "YOLO model   : " << model_path << std::endl;
        std::cout << "Config path  : " << config_path << std::endl;
        std::cout << "Bullet speed : " << bullet_speed << " m/s" << std::endl;
        std::cout << "Controls     : q quit, v draw mode" << std::endl;

        std::atomic<bool> running{true};
        std::mutex frame_mutex;
        std::condition_variable frame_cv;
        std::shared_ptr<VideoFramePacket> latest_packet;
        int reader_frame_index = 0;
        uint64_t latest_packet_id = 0;
        uint64_t consumed_packet_id = 0;
        bool reader_done = false;

        std::thread reader_thread([&]() {
            while (running)
            {
                cv::Mat frame;
                if (!cap.read(frame))
                {
                    auto packet = std::make_shared<VideoFramePacket>();
                    packet->eof = true;
                    {
                        std::lock_guard<std::mutex> lock(frame_mutex);
                        latest_packet = packet;
                        latest_packet_id = static_cast<uint64_t>(++reader_frame_index);
                        reader_done = true;
                    }
                    frame_cv.notify_all();
                    running = false;
                    break;
                }

                auto packet = std::make_shared<VideoFramePacket>();
                packet->frame = std::move(frame);
                packet->frame_index = ++reader_frame_index;
                {
                    std::lock_guard<std::mutex> lock(frame_mutex);
                    latest_packet = packet;
                    latest_packet_id = static_cast<uint64_t>(packet->frame_index);
                }
                frame_cv.notify_one();
            }

            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                reader_done = true;
            }
            frame_cv.notify_all();
        });

        DrawMode draw_mode = DrawMode::Full;
        int frame_count = 0;
        int fps_frame_count = 0;
        auto start_time = std::chrono::steady_clock::now();
        auto last_fps_time = start_time;
        double fps = 0.0;

        while (running)
        {
            std::shared_ptr<VideoFramePacket> packet;
            {
                std::unique_lock<std::mutex> lock(frame_mutex);
                frame_cv.wait(lock, [&]() { return latest_packet_id != consumed_packet_id || reader_done || !running; });
                if (!running || (latest_packet_id == consumed_packet_id && reader_done))
                {
                    break;
                }
                packet = latest_packet;
                consumed_packet_id = latest_packet_id;
            }

            if (!packet) continue;
            if (packet->eof) break;

            cv::Mat &display_frame = packet->frame;
            const auto frame_time = std::chrono::steady_clock::now();
            const auto total_begin = frame_time;
            frame_count++;
            fps_frame_count++;

            StageStats stats;
            stats.fps = fps;
            ArmorArray detected_armors;
            std::vector<Target> targets;
            std::optional<io::Command> latest_autoaim_cmd;
            AimPoint latest_aim_point{};

            auto detect_begin = std::chrono::steady_clock::now();
            detected_armors = detector.detect(display_frame);
            auto detect_end = std::chrono::steady_clock::now();
            stats.detect_ms = std::chrono::duration<double, std::milli>(detect_end - detect_begin).count();

            pnp_solver.set_R_gimbal2world(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
            auto track_begin = std::chrono::steady_clock::now();
            targets = tracker.track(detected_armors, frame_time);
            auto track_end = std::chrono::steady_clock::now();
            stats.track_ms = std::chrono::duration<double, std::milli>(track_end - track_begin).count();

            if (!targets.empty())
            {
                auto aim_begin = std::chrono::steady_clock::now();
                std::list<Target> target_list(targets.begin(), targets.end());
                latest_autoaim_cmd = aimer.aim(target_list, frame_time);
                latest_aim_point = aimer.debug_aim_point;
                stats.aim_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - aim_begin).count();
            }

            auto draw_begin = std::chrono::steady_clock::now();
            if (draw_mode != DrawMode::Off)
            {
                drawArmorDetection(display_frame, detected_armors);
                if (draw_mode == DrawMode::Full)
                {
                    drawTargetInfo(display_frame, targets, tracker.state(), pnp_solver);
                    if (latest_aim_point.valid)
                    {
                        drawTrajectory(display_frame, latest_aim_point, bullet_speed, config_path, camera_matrix, distort_coeffs, pnp_solver.R_gimbal2world_);
                    }
                }
            }

            const auto now = std::chrono::steady_clock::now();
            const double fps_elapsed = std::chrono::duration<double>(now - last_fps_time).count();
            if (fps_elapsed >= 0.5)
            {
                fps = fps_frame_count / fps_elapsed;
                fps_frame_count = 0;
                last_fps_time = now;
            }
            stats.fps = fps;
            stats.draw_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - draw_begin).count();
            stats.total_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_begin).count();

            if (draw_mode != DrawMode::Off)
            {
                draw_stage_stats(display_frame, stats, draw_mode, packet->frame_index);

                if (latest_autoaim_cmd)
                {
                    std::ostringstream ss;
                    ss << std::fixed << std::setprecision(2) << "Yaw " << latest_autoaim_cmd->yaw * 180.0 / CV_PI << " deg  Pitch " << latest_autoaim_cmd->pitch * 180.0 / CV_PI << " deg";
                    cv::putText(display_frame, ss.str(), cv::Point(10, display_frame.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                }
                else
                {
                    cv::putText(display_frame, "No target", cv::Point(10, display_frame.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                }

                cv::imshow("Auto Aim Test - Target Visualization", display_frame);
            }

            const int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27)
            {
                running = false;
                break;
            }
            if (key == 'v')
            {
                draw_mode = next_draw_mode(draw_mode);
                std::cout << "Draw mode: " << draw_mode_name(draw_mode) << std::endl;
            }
        }

        running = false;
        if (reader_thread.joinable()) reader_thread.join();
        cap.release();
        cv::destroyAllWindows();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
