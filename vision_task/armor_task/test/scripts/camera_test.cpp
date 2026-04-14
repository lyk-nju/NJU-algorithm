#include "io/camera/camera_base.hpp"

#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

int main(int argc, char * argv[])
{
  const std::string config_path = (argc > 1) ? argv[1] : "config/camera1.yaml";
  const bool display = (argc <= 2) ? true : (std::string(argv[2]) != "0");

  try {
    io::Camera camera(config_path);

    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    auto last_print = std::chrono::steady_clock::now();
    std::size_t frame_count = 0;

    while (true) {
      camera.read(img, timestamp);
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
      if (key == 'q' || key == 27) {
        break;
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "[camera_test] exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
