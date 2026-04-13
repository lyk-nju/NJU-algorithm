#include "hik.hpp"

#include <cstdint>
#include <iostream>
#include <libusb-1.0/libusb.h>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace io
{
Hik::Hik(const std::string & config_path)
{
  load_config(config_path);
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
    owns_ros_context_ = true;
  }
  ros_node_ = rclcpp::Node::make_shared("hik_camera_publisher");
  image_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS().keep_last(1));

  if (libusb_init(nullptr) != 0) {
    std::cerr << "[Hik] libusb init failed." << std::endl;
  }

  daemon_thread_ = std::thread([this] {
    capture_start();

    while (!daemon_quit_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      if (capturing_.load()) {
        continue;
      }

      capture_stop();
      reset_usb();
      capture_start();
    }

    capture_stop();
  });
}

Hik::~Hik()
{
  daemon_quit_.store(true);
  if (daemon_thread_.joinable()) {
    daemon_thread_.join();
  }
  libusb_exit(nullptr);
  if (owns_ros_context_ && rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void Hik::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data{};
  queue_.pop(data);
  img = data.img;
  timestamp = data.timestamp;
}

void Hik::load_config(const std::string & config_path)
{
  const YAML::Node yaml = YAML::LoadFile(config_path);

  if (yaml["exposure_time"]) {
    exposure_time_ = yaml["exposure_time"].as<float>();
  }
  if (yaml["gain"]) {
    gain_ = yaml["gain"].as<float>();
  }
  if (yaml["adc_bit_depth"]) {
    adc_bit_depth_ = yaml["adc_bit_depth"].as<int>();
  }
  if (yaml["pixel_format"]) {
    pixel_format_ = yaml["pixel_format"].as<std::string>();
  }
  if (yaml["auto_white_banlance"]) {
    auto_white_banlance_ = yaml["auto_white_banlance"].as<bool>();
  } else if (yaml["auto_white_balance"]) {
    auto_white_banlance_ = yaml["auto_white_balance"].as<bool>();
  }
  if (yaml["auto_exposure_time"]) {
    auto_exposure_time_ = yaml["auto_exposure_time"].as<bool>();
  }
  if (yaml["acquisition_frame_rate"]) {
    acquisition_frame_rate_ = yaml["acquisition_frame_rate"].as<float>();
  }
  if (yaml["publish_image_raw"]) {
    publish_image_raw_ = yaml["publish_image_raw"].as<bool>();
  }
  if (yaml["vid"] && !yaml["vid"].IsNull()) {
    try {
      if (yaml["vid"].IsScalar()) {
        const std::string value = yaml["vid"].as<std::string>();
        if (!value.empty()) {
          vid_ = std::stoi(value, nullptr, 0);
        }
      } else {
        vid_ = yaml["vid"].as<int>();
      }
    } catch (const std::exception &) {
      std::cerr << "[Hik] Invalid vid in yaml, fallback to first detected camera." << std::endl;
      vid_ = -1;
    }
  }
  if (yaml["pid"] && !yaml["pid"].IsNull()) {
    try {
      if (yaml["pid"].IsScalar()) {
        const std::string value = yaml["pid"].as<std::string>();
        if (!value.empty()) {
          pid_ = std::stoi(value, nullptr, 0);
        }
      } else {
        pid_ = yaml["pid"].as<int>();
      }
    } catch (const std::exception &) {
      std::cerr << "[Hik] Invalid pid in yaml, fallback to first detected camera." << std::endl;
      pid_ = -1;
    }
  }
}

void Hik::capture_start()
{
  if (capturing_.load()) {
    return;
  }
  capturing_.store(false);
  capture_quit_.store(false);

  MV_CC_DEVICE_INFO_LIST device_list{};
  int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    std::cerr << "[Hik] MV_CC_EnumDevices failed: " << std::hex << ret << std::dec << std::endl;
    return;
  }
  if (device_list.nDeviceNum == 0) {
    std::cerr << "[Hik] No camera found." << std::endl;
    return;
  }

  MV_CC_DEVICE_INFO * selected = device_list.pDeviceInfo[0];

  ret = MV_CC_CreateHandle(&handle_, selected);
  if (ret != MV_OK) {
    std::cerr << "[Hik] MV_CC_CreateHandle failed: " << std::hex << ret << std::dec << std::endl;
    handle_ = nullptr;
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    std::cerr << "[Hik] MV_CC_OpenDevice failed: " << std::hex << ret << std::dec << std::endl;
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return;
  }

  apply_camera_params();

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    std::cerr << "[Hik] MV_CC_StartGrabbing failed: " << std::hex << ret << std::dec << std::endl;
    capture_stop();
    return;
  }

  capture_thread_ = std::thread([this] {
    capturing_.store(true);
    while (!capture_quit_.load()) {
      MV_FRAME_OUT raw{};
      int grab_ret = MV_CC_GetImageBuffer(handle_, &raw, 100);
      if (grab_ret != MV_OK) {
        std::cerr << "[Hik] MV_CC_GetImageBuffer failed: " << std::hex << grab_ret << std::dec << std::endl;
        break;
      }

      const int width = static_cast<int>(raw.stFrameInfo.nWidth);
      const int height = static_cast<int>(raw.stFrameInfo.nHeight);
      cv::Mat frame_bgr;

      if (raw.stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed) {
        frame_bgr = cv::Mat(height, width, CV_8UC3, raw.pBufAddr).clone();
      } else if (
        raw.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8 ||
        raw.stFrameInfo.enPixelType == PixelType_Gvsp_BayerBG8 ||
        raw.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR8 ||
        raw.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB8)
      {
        const cv::Mat src(height, width, CV_8UC1, raw.pBufAddr);
        int cv_code = cv::COLOR_BayerRG2BGR;
        switch (raw.stFrameInfo.enPixelType) {
          case PixelType_Gvsp_BayerRG8:
            cv_code = cv::COLOR_BayerRG2BGR;
            break;
          case PixelType_Gvsp_BayerBG8:
            cv_code = cv::COLOR_BayerBG2BGR;
            break;
          case PixelType_Gvsp_BayerGR8:
            cv_code = cv::COLOR_BayerGR2BGR;
            break;
          case PixelType_Gvsp_BayerGB8:
            cv_code = cv::COLOR_BayerGB2BGR;
            break;
          default:
            break;
        }
        cv::cvtColor(src, frame_bgr, cv_code);
      } else if (raw.stFrameInfo.enPixelType == PixelType_Gvsp_Mono8) {
        const cv::Mat src(height, width, CV_8UC1, raw.pBufAddr);
        cv::cvtColor(src, frame_bgr, cv::COLOR_GRAY2BGR);
      } else {
        MV_CC_PIXEL_CONVERT_PARAM convert_param{};
        std::vector<unsigned char> dst(static_cast<size_t>(width) * static_cast<size_t>(height) * 3U);
        convert_param.nWidth = raw.stFrameInfo.nWidth;
        convert_param.nHeight = raw.stFrameInfo.nHeight;
        convert_param.pSrcData = raw.pBufAddr;
        convert_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
        convert_param.enSrcPixelType = raw.stFrameInfo.enPixelType;
        convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        convert_param.pDstBuffer = dst.data();
        convert_param.nDstBufferSize = static_cast<unsigned int>(dst.size());
        const int cvt_ret = MV_CC_ConvertPixelType(handle_, &convert_param);
        if (cvt_ret == MV_OK) {
          frame_bgr = cv::Mat(height, width, CV_8UC3, dst.data()).clone();
        }
      }

      const int free_ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (free_ret != MV_OK) {
        std::cerr << "[Hik] MV_CC_FreeImageBuffer failed: " << std::hex << free_ret << std::dec << std::endl;
        break;
      }

      if (!frame_bgr.empty()) {
        queue_.push(CameraData{frame_bgr, std::chrono::steady_clock::now()});
        publish_frame(frame_bgr);
      }
    }
    capturing_.store(false);
  });
}

void Hik::capture_stop()
{
  capture_quit_.store(true);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  capturing_.store(false);

  if (handle_ != nullptr) {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }
}

void Hik::apply_camera_params()
{
  const unsigned int white_auto =
    auto_white_banlance_ ? MV_BALANCEWHITE_AUTO_CONTINUOUS : MV_BALANCEWHITE_AUTO_OFF;
  set_enum_value("BalanceWhiteAuto", white_auto);

  const unsigned int exposure_auto =
    auto_exposure_time_ ? MV_EXPOSURE_AUTO_MODE_CONTINUOUS : MV_EXPOSURE_AUTO_MODE_OFF;
  set_enum_value("ExposureAuto", exposure_auto);

  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);

  if (!auto_exposure_time_) {
    set_float_value("ExposureTime", exposure_time_);
  }
  set_float_value("Gain", gain_);
  set_float_value("AcquisitionFrameRate", acquisition_frame_rate_);

  set_enum_value("PixelFormat", PixelType_Gvsp_BayerRG8);

  if (adc_bit_depth_ == 8) {
    set_int_value("ADCBitDepth", 8U);
  } else if (adc_bit_depth_ == 10) {
    set_int_value("ADCBitDepth", 10U);
  } else if (adc_bit_depth_ == 12) {
    set_int_value("ADCBitDepth", 12U);
  }
}

bool Hik::set_float_value(const char * name, float value)
{
  const int ret = MV_CC_SetFloatValue(handle_, name, value);
  if (ret != MV_OK) {
    std::cerr << "[Hik] MV_CC_SetFloatValue(" << name << ") failed: " << std::hex << ret << std::dec << std::endl;
    return false;
  }
  return true;
}

bool Hik::set_enum_value(const char * name, unsigned int value)
{
  const int ret = MV_CC_SetEnumValue(handle_, name, value);
  if (ret != MV_OK) {
    std::cerr << "[Hik] MV_CC_SetEnumValue(" << name << ") failed: " << std::hex << ret << std::dec << std::endl;
    return false;
  }
  return true;
}

bool Hik::set_int_value(const char * name, unsigned int value)
{
  const int ret = MV_CC_SetIntValue(handle_, name, value);
  if (ret != MV_OK) {
    std::cerr << "[Hik] MV_CC_SetIntValue(" << name << ") failed: " << std::hex << ret << std::dec << std::endl;
    return false;
  }
  return true;
}

void Hik::reset_usb() const
{
  if (vid_ < 0 || pid_ < 0) {
    return;
  }

  libusb_device_handle * usb_handle =
    libusb_open_device_with_vid_pid(nullptr, static_cast<uint16_t>(vid_), static_cast<uint16_t>(pid_));
  if (usb_handle == nullptr) {
    std::cerr << "[Hik] Unable to open usb for reset." << std::endl;
    return;
  }

  if (libusb_reset_device(usb_handle) != 0) {
    std::cerr << "[Hik] Unable to reset usb." << std::endl;
  } else {
    std::cerr << "[Hik] Reset usb successfully." << std::endl;
  }
  libusb_close(usb_handle);
}

void Hik::publish_frame(const cv::Mat & frame)
{
  if (!publish_image_raw_ || !image_pub_ || frame.empty()) {
    return;
  }
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  msg->header.stamp = ros_node_->now();
  msg->header.frame_id = "hik_camera";
  image_pub_->publish(*msg);
}

}  // namespace io
