#include "aimer.hpp"
#include "detector.hpp"
#include "pnp_solver.hpp"
#include "shooter.hpp"
#include "tracker.hpp"
#include "draw.hpp"
#include "math_tools.hpp"
#include "pharser.hpp"

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
  try {
    std::string test_config_path = "../config/video_test.yaml";
    if (argc > 1) {
      test_config_path = argv[1];
    }

    const TestConfig test_config = load_video_test_config(test_config_path);
    const std::string model_path = test_config.yolo_model_path;
    const std::string config_path = test_config.config_path;
    const std::string input_video_path = test_config.video_path;
    const double bullet_speed = test_config.bullet_speed;

    Detector detector(model_path);
    PnpSolver pnp_solver(config_path);
    Tracker tracker(config_path, pnp_solver);
    Aimer aimer(config_path);
    Shooter shooter(config_path);
    const auto camera_params = loadCameraParameters(config_path);
    const cv::Mat camera_matrix = camera_params.first;
    const cv::Mat distort_coeffs = camera_params.second;

    cv::VideoCapture cap(input_video_path);
    if (!cap.isOpened()) {
      std::cerr << "Error: Cannot open video file: " << input_video_path << std::endl;
      return -1;
    }

    std::cout << "Video source : " << input_video_path << std::endl;
    std::cout << "YOLO model   : " << model_path << std::endl;
    std::cout << "Config path  : " << config_path << std::endl;
    std::cout << "Bullet speed : " << bullet_speed << " m/s" << std::endl;

    int frame_index = 0;
    int fps_frame_count = 0;
    auto last_fps_time = std::chrono::steady_clock::now();
    double fps = 0.0;

    // Video test has no real cboard, use a fixed enemy color assumption.
    const bool enemy_is_red = true;

    cv::Mat frame;
    while (cap.read(frame)) {
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
      io::Vision2Cboard cmd2cboard{false, false, 0.0, 0.0, {0.0f, 0.0f, 0.0f}};
      if (!targets.empty()) {
        std::list<Target> target_list(targets.begin(), targets.end());
        cmd2cboard = aimer.aim(target_list, frame_time, bullet_speed);
      }
      const double aim_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - aim_begin).count();

      Eigen::Vector3d ypr = tools::eulers(pnp_solver.R_gimbal2world_, 2, 1, 0);
      cmd2cboard.shoot = shooter.shoot(cmd2cboard, aimer, targets, ypr);

      // 1) detector: detected armors
      for (const auto &armor : detected_armors) {
        tools::draw_box(display_frame, armor.box, cv::Scalar(0, 255, 0), 2);
        tools::draw_point(display_frame, cv::Point(cvRound(armor.center.x), cvRound(armor.center.y)), cv::Scalar(0, 255, 0), 3);
      }

      // 2) tracker/ekf: predicted armors
      if (!targets.empty()) {
        const Target &target = targets.front();
        const bool is_large = (target.car_num == 1);
        const auto xyza_list = target.armor_xyza_list();
        for (const auto &xyza : xyza_list) {
          const auto corners = pnp_solver.reproject_armor(xyza.head<3>(), xyza[3], is_large);
          if (corners.size() < 4) {
            continue;
          }
          cv::Rect ekf_box = cv::boundingRect(corners);
          tools::draw_box(display_frame, ekf_box, cv::Scalar(255, 0, 0), 2);
        }
      }

      // 3) aim point and trajectory
      const AimPoint &aim_point = aimer.debug_aim_point;
      if (aim_point.valid) {
        const auto aim_corners = pnp_solver.reproject_armor(aim_point.xyza.head<3>(), aim_point.xyza[3], false);
        if (!aim_corners.empty()) {
          cv::Point2f sum(0.0F, 0.0F);
          for (const auto &pt : aim_corners) {
            sum += pt;
          }
          const cv::Point2f center = sum * (1.0F / static_cast<float>(aim_corners.size()));
          tools::draw_point(display_frame, cv::Point(cvRound(center.x), cvRound(center.y)), cv::Scalar(0, 0, 255), 4);
        }

        drawTrajectory(display_frame, aim_point, bullet_speed, config_path, camera_matrix, distort_coeffs, pnp_solver.R_gimbal2world_);
      }

      const auto now = std::chrono::steady_clock::now();
      const double fps_elapsed = std::chrono::duration<double>(now - last_fps_time).count();
      if (fps_elapsed >= 0.5) {
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
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q' || key == 'Q') {
        break;
      }

      std::cout << std::fixed << std::setprecision(2)
                << "frame=" << frame_index
                << " fps=" << fps
                << " detect_ms=" << detect_ms
                << " track_ms=" << track_ms
                << " aim_ms=" << aim_ms
                << " tracker_state=" << tracker.state()
                << " targets=" << targets.size()
                << " valid=" << cmd2cboard.valid
                << " shoot=" << cmd2cboard.shoot
                << " yaw=" << cmd2cboard.yaw
                << " pitch=" << cmd2cboard.pitch
                << std::endl;
    }

    cap.release();
    cv::destroyAllWindows();
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}

