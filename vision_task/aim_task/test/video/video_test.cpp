#include "tasks/aim_pipeline.hpp"
#include "tasks/pipeline_types.hpp"
#include "tools/draw.hpp"
#include "tools/math_tools.hpp"
#include "tools/parser.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <iomanip>
#include <iostream>
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

        // 一套算法入口，与 auto_aim_test 共享接线
        AimPipeline pipeline(model_path, config_path);

        // 只有调试/可视化才需要中间量，生产路径不会产生这份拷贝
        DebugSnapshot dbg;

        // 从 Pipeline 内部的 PnpSolver 导出一份 CameraInfo，用于：
        //   (a) FrameBundle.camera 指向它；
        //   (b) drawTrajectory 仍然需要 camera_matrix / distort_coeffs 作为参数。
        // 统一从这一个来源，避免二次读 yaml 引入不一致。
        const io::CameraInfo camera_info = pipeline.pnp_solver().as_camera_info(0, "video_test");

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

        // video_test 没有真实下位机，假定敌方恒为红色
        const GameState game_state{/*enemy_is_red=*/true, bullet_speed};

        cv::Mat frame;
        while (cap.read(frame))
        {
            const auto loop_start = std::chrono::steady_clock::now();
            cv::Mat display_frame = frame.clone();
            ++frame_index;
            ++fps_frame_count;
            const auto frame_time = std::chrono::steady_clock::now();

            FrameBundle bundle;
            bundle.frame.image = frame;
            bundle.frame.timestamp = frame_time;
            bundle.camera = &camera_info;
            bundle.gimbal_quat = Eigen::Quaterniond::Identity(); // video_test 无 IMU

            const auto step_begin = std::chrono::steady_clock::now();
            io::Vision2Cboard decision = pipeline.step(bundle, game_state, dbg);
            const auto step_end = std::chrono::steady_clock::now();
            const double step_ms = std::chrono::duration<double, std::milli>(step_end - step_begin).count();

            io::Vision2Cboard cmd2cboard = decision.cmd;
            // video_test 里永远允许开火（原先就是这么写的），
            // 覆盖掉 Shooter 的判断结果以便在离线视频上观察瞄准效果
            cmd2cboard.gimbal_cmd_.shoot = true;

            // 1) detector: detected armors
            for (const auto &armor : dbg.detected_armors)
            {
                tools::draw_box(display_frame, armor, cv::Scalar(0, 255, 0), 2);
                tools::draw_point(display_frame, cv::Point(cvRound(armor.center.x), cvRound(armor.center.y)), cv::Scalar(0, 255, 0), 3);
            }

            // 2) tracker/ekf: predicted armors
            if (!dbg.targets.empty())
            {
                tools::draw_box(display_frame, dbg.targets.front(), pipeline.pnp_solver(), cv::Scalar(255, 0, 0), 2);
            }

            // 3) aim point and trajectory
            const AimPoint &aim_point = dbg.debug_aim_point;
            if (aim_point.valid)
            {
                tools::draw_point(display_frame, aim_point, pipeline.pnp_solver(), cv::Scalar(0, 0, 255), 4, false);
                tools::drawTrajectory(display_frame, aim_point, bullet_speed, config_path, camera_info.camera_matrix, camera_info.distort_coeffs, pipeline.pnp_solver().R_gimbal2world_);
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
            tools::draw_text(display_frame, "Tracker: " + pipeline.tracker().state(), cv::Point(20, 60), cv::Scalar(255, 255, 255), 0.7, 2);

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

            std::cout << std::fixed << std::setprecision(2) << "frame=" << frame_index << " fps=" << fps << " step_ms=" << step_ms
                      << " tracker_state=" << pipeline.tracker().state() << " targets=" << dbg.targets.size()
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
