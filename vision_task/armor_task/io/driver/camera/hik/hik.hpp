#pragma once

#include "../camera_base.hpp"
#include "../../../algorithm/thread_safe_queue.hpp"
#include "include/MvCameraControl.h"

#include <atomic>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <thread>

namespace io
{
class Hik : public CameraBase
{
public:
  explicit Hik(const std::string & config_path);
  ~Hik() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  float exposure_time_ = 1000.0F;
  float gain_ = 16.9F;
  int adc_bit_depth_ = 8;
  bool auto_white_banlance_ = false;
  bool auto_exposure_time_ = false;
  float acquisition_frame_rate_ = 250.0F;
  bool publish_image_raw_ = true;
  int vid_ = -1;
  int pid_ = -1;

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_{false};
  void * handle_ = nullptr;
  std::thread capture_thread_;
  std::atomic<bool> capture_quit_{false};
  std::atomic<bool> capturing_{false};
  tools::ThreadSafeQueue<CameraData> queue_{1};

  void load_config(const std::string & config_path);
  void capture_start();
  void capture_stop();
  void apply_camera_params();

  bool set_float_value(const char * name, float value);
  bool set_enum_value(const char * name, unsigned int value);
  bool get_enum_value(const char * name, MVCC_ENUMVALUE & value) const;
  void set_adc_bit_depth();
  void reset_usb() const;
  void publish_frame(const cv::Mat & frame);

  bool owns_ros_context_ = false;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};
}  // namespace io
