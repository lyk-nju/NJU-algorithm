#include "tasks/aimer.hpp"
#include "tasks/detector.hpp"
#include "tasks/pnp_solver.hpp"
#include "tasks/shooter.hpp"
#include "tasks/tracker.hpp"
#include "tools/draw.hpp"
#include "tools/math_tools.hpp"
#include "tools/pharser.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

using namespace armor_task;

int main(int argc, char *argv[])
{
    try
    {
        std::string test_config_path = "../config/video_test.yaml";
        bool force_onnx = false;

        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "--onnx")
            {
                force_onnx = true;
                continue;
            }
            if (arg == "--config" && i + 1 < argc)
            {
                test_config_path = argv[++i];
                continue;
            }
            if (!arg.empty() && arg[0] != '-')
            {
                test_config_path = arg; // backward-compatible: first positional arg is config path
                continue;
            }
            std::cerr << "Unknown arg: " << arg << std::endl;
            std::cerr << "Usage: video_test [config_path] [--config <path>] [--onnx]" << std::endl;
            return -1;
        }

        const TestConfig test_config = load_video_test_config(test_config_path);
        std::string model_path = test_config.yolo_model_path;

        const auto ends_with = [](const std::string &s, const std::string &suffix) { return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0; };

        if (force_onnx)
        {
            if (!ends_with(model_path, ".onnx"))
            {
                std::cerr << "Warning: --onnx specified but config model is not .onnx, fallback to ../models/best.onnx" << std::endl;
                model_path = "../models/best.onnx";
            }
        }
        else
        {
            if (!ends_with(model_path, ".engine"))
            {
                std::cerr << "Warning: default mode uses TensorRT engine, fallback to ../models/best.engine" << std::endl;
                model_path = "../models/best.engine";
            }
        }
        const std::string config_path = test_config.config_path;
        const std::string input_video_path = test_config.video_path;
        const double bullet_speed = test_config.bullet_speed;
        const int playback_fps = test_config.playback_fps;

        Detector detector(model_path);
        PnpSolver pnp_solver(config_path);
        Tracker tracker(config_path, pnp_solver);
        Aimer aimer(config_path);
        Shooter shooter(config_path);
        const auto camera_params = loadCameraParameters(config_path);
        const cv::Mat camera_matrix = camera_params.first;
        const cv::Mat distort_coeffs = camera_params.second;

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
        std::cout << "Playback FPS : " << playback_fps << std::endl;

        int frame_index = 0;
        int fps_frame_count = 0;
        auto last_fps_time = std::chrono::steady_clock::now();
        double fps = 0.0;

        // Video test has no real cboard, use a fixed enemy color assumption.
        const bool enemy_is_red = true;

        cv::Mat frame;
        while (cap.read(frame))
        {
            const auto loop_start = std::chrono::steady_clock::now();
            cv::Mat display_frame = frame.clone();
            ++frame_index;
            ++fps_frame_count;
            const auto frame_time = std::chrono::steady_clock::now();

            const auto detect_begin = std::chrono::steady_clock::now();
            ArmorArray detected_armors = detector.detect(frame);
            const auto detect_end = std::chrono::steady_clock::now();
            const double detect_ms = std::chrono::duration<double, std::milli>(detect_end - detect_begin).count();

            pnp_solver.set_R_gimbal2world(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

            const auto track_begin = std::chrono::steady_clock::now();
            std::vector<Target> targets = tracker.track(detected_armors, frame_time, enemy_is_red);
            const auto track_end = std::chrono::steady_clock::now();
            const double track_ms = std::chrono::duration<double, std::milli>(track_end - track_begin).count();

            const auto aim_begin = std::chrono::steady_clock::now();
            io::Vision2Cboard cmd2cboard{
                io::gimbal_command{false, false, 0.0f, 0.0f},
                io::base_command{0.0f, 0.0f, 0.0f}};
            if (!targets.empty())
            {
                std::list<Target> target_list(targets.begin(), targets.end());
                cmd2cboard = aimer.aim(target_list, frame_time, bullet_speed);
            }
            const double aim_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - aim_begin).count();

            // video_test默认可以开火
            cmd2cboard.gimbal_cmd_.shoot = true;

            // 1) detector: detected armors
            for (const auto &armor : detected_armors)
            {
                tools::draw_box(display_frame, armor, cv::Scalar(0, 255, 0), 2);
                tools::draw_point(display_frame, cv::Point(cvRound(armor.center.x), cvRound(armor.center.y)), cv::Scalar(0, 255, 0), 3);
            }

            // 2) tracker/ekf: predicted armors
            if (!targets.empty())
            {
                tools::draw_box(display_frame, targets.front(), pnp_solver, cv::Scalar(255, 0, 0), 2);
            }

            // 3) aim point and trajectory
            const AimPoint &aim_point = aimer.debug_aim_point;
            if (aim_point.valid)
            {
                tools::draw_point(display_frame, aim_point, pnp_solver, cv::Scalar(0, 0, 255), 4, false);

                tools::drawTrajectory(display_frame, aim_point, bullet_speed, config_path, camera_matrix, distort_coeffs, pnp_solver.R_gimbal2world_);
            }

            const auto now = std::chrono::steady_clock::now();
            const double fps_elapsed = std::chrono::duration<double>(now - last_fps_time).count();
            if (fps_elapsed >= 0.5)
            {
                fps = static_cast<double>(fps_frame_count) / fps_elapsed;
                fps_frame_count = 0;
                last_fps_time = now;
            }

            {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(1) << "FPS: " << fps;
                tools::draw_text(display_frame, oss.str(), cv::Point(20, 30), cv::Scalar(0, 255, 0), 0.7, 2);
            }
            tools::draw_text(display_frame, "Tracker: " + tracker.state(), cv::Point(20, 60), cv::Scalar(255, 255, 255), 0.7, 2);

            cv::imshow("video_test", display_frame);
            int wait_ms = 1;
            if (playback_fps > 0)
            {
                const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - loop_start).count();
                const int frame_period_ms = static_cast<int>(1000.0 / playback_fps);
                wait_ms = std::max(1, frame_period_ms - static_cast<int>(elapsed_ms));
            }
            const int key = cv::waitKey(wait_ms);
            if (key == 27 || key == 'q' || key == 'Q')
            {
                break;
            }

            std::cout << std::fixed << std::setprecision(2) << "frame=" << frame_index << " fps=" << fps << " detect_ms=" << detect_ms << " track_ms=" << track_ms << " aim_ms=" << aim_ms
                      << " tracker_state=" << tracker.state() << " targets=" << targets.size()
                      << " valid=" << cmd2cboard.gimbal_cmd_.valid << " shoot=" << cmd2cboard.gimbal_cmd_.shoot
                      << " yaw=" << cmd2cboard.gimbal_cmd_.yaw << " pitch=" << cmd2cboard.gimbal_cmd_.pitch << std::endl;
        }

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
