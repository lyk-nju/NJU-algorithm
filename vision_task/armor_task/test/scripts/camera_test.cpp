#include "io/driver/camera/camera_base.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

namespace
{
std::atomic<bool> g_quit{false};

void handle_signal(int)
{
  g_quit.store(true);
}
}  // namespace

int main(int argc, char * argv[])
{
  const std::string config_path = (argc > 1) ? argv[1] : "config/camera1.yaml";
  const bool display = (argc <= 2) ? true : (std::string(argv[2]) != "0");

  try {
    io::Camera camera(config_path);
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    auto last_print = std::chrono::steady_clock::now();
    std::size_t frame_count = 0;

    while (!g_quit.load()) {
      camera.read(img, timestamp);
      if (img.empty()) {
        if (display) {
          const int key = cv::waitKey(1);
          if (key == 'q' || key == 'Q' || key == 27) {
            break;
          }
        }
        continue;
      }

      ++frame_count;

      const auto now = std::chrono::steady_clock::now();
      const auto elapsed = std::chrono::duration<double>(now - last_print).count();
      if (elapsed >= 1.0) {
        const double fps = static_cast<double>(frame_count) / elapsed;
        std::cout << "[camera_test] fps: " << fps << std::endl;
        frame_count = 0;
        last_print = now;
      }

      if (!display) {
        continue;
      }

      cv::imshow("camera_test", img);
      const int key = cv::waitKey(1);
      if (key == 'q' || key == 'Q' || key == 27) {
        break;
      }
    }
    if (display) {
      cv::destroyAllWindows();
    }
  } catch (const std::exception & e) {
    std::cerr << "[camera_test] exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
