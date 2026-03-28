#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera
{
namespace
{
bool enumSupports(const MVCC_ENUMVALUE &enum_value, unsigned int value)
{
  for (unsigned int i = 0; i < enum_value.nSupportedNum; ++i) {
    if (enum_value.nSupportValue[i] == value) return true;
  }
  return false;
}

}  // namespace

class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    MV_CC_DEVICE_INFO_LIST device_list;
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);

    MV_CC_OpenDevice(camera_handle_);
    MV_CC_SetTriggerMode(camera_handle_, 0);
    configureAdcBitDepth();
    configureSdkStream();
    applyBinning();

    // Get camera infomation
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.resize(img_info_.nHeightValue * img_info_.nWidthValue * 3);

    MVCC_ENUMVALUE pixel_format_value;
    std::memset(&pixel_format_value, 0, sizeof(pixel_format_value));
    nRet = MV_CC_GetPixelFormat(camera_handle_, &pixel_format_value);
    if (nRet == MV_OK) {
      RCLCPP_INFO(
        this->get_logger(), "Current camera pixel format: 0x%x, supported count: %u",
        pixel_format_value.nCurValue, pixel_format_value.nSupportedNum);

      if (enumSupports(pixel_format_value, PixelType_Gvsp_BGR8_Packed)) {
        nRet = MV_CC_SetPixelFormat(camera_handle_, PixelType_Gvsp_BGR8_Packed);
        if (nRet == MV_OK) {
          native_bgr_output_ = true;
          RCLCPP_INFO(this->get_logger(), "Using native BGR8 camera output.");
        }
      } else if (enumSupports(pixel_format_value, PixelType_Gvsp_RGB8_Packed)) {
        nRet = MV_CC_SetPixelFormat(camera_handle_, PixelType_Gvsp_RGB8_Packed);
        if (nRet == MV_OK) {
          native_rgb_output_ = true;
          RCLCPP_INFO(this->get_logger(), "Using native RGB8 camera output.");
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Camera does not support native BGR8/RGB8 output, keeping SDK conversion path.");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to query pixel format, nRet: [%x]", nRet);
    }

    // Init convert param
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    convert_param_.pDstBuffer = image_msg_.data.data();
    convert_param_.nDstBufferSize = image_msg_.data.size();

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    publish_camera_info_ = this->declare_parameter("publish_camera_info", true);
    enable_timing_log_ = this->declare_parameter("enable_timing_log", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    if (publish_camera_info_) {
      camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
    } else {
      image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    }

    declareParameters();

    MV_CC_StartGrabbing(camera_handle_);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
      camera_info_msg_.binning_x = binning_horizontal_;
      camera_info_msg_.binning_y = binning_vertical_;
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = native_rgb_output_ ? "rgb8" : "bgr8";
      image_msg_.height = img_info_.nHeightValue;
      image_msg_.width = img_info_.nWidthValue;
      image_msg_.step = img_info_.nWidthValue * 3;
      int stat_count = 0;
      double total_get_ms = 0.0;
      double total_convert_ms = 0.0;
      double total_publish_ms = 0.0;
      auto stat_start = std::chrono::steady_clock::now();

      while (rclcpp::ok()) {
        auto get_start = std::chrono::steady_clock::now();
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 500);
        auto get_end = std::chrono::steady_clock::now();
        if (MV_OK == nRet) {
          auto convert_start = std::chrono::steady_clock::now();
          if (native_bgr_output_ || native_rgb_output_) {
            const size_t image_size = static_cast<size_t>(image_msg_.step) * image_msg_.height;
            std::memcpy(image_msg_.data.data(), out_frame.pBufAddr, image_size);
          } else {
            convert_param_.pSrcData = out_frame.pBufAddr;
            convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
            convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
            MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
          }
          auto convert_end = std::chrono::steady_clock::now();

          image_msg_.header.stamp = this->now();

          auto publish_start = std::chrono::steady_clock::now();
          if (publish_camera_info_) {
            camera_info_msg_.header = image_msg_.header;
            camera_pub_.publish(image_msg_, camera_info_msg_);
      
          } else {
            image_pub_->publish(image_msg_);
      
          }
          auto publish_end = std::chrono::steady_clock::now();

          if (enable_timing_log_) {
            total_get_ms += std::chrono::duration<double, std::milli>(get_end - get_start).count();
            total_convert_ms += std::chrono::duration<double, std::milli>(convert_end - convert_start).count();
            total_publish_ms += std::chrono::duration<double, std::milli>(publish_end - publish_start).count();
            ++stat_count;

            const auto elapsed = std::chrono::duration<double>(publish_end - stat_start).count();
            if (elapsed >= 1.0) {
              const double fps = stat_count / elapsed;
              RCLCPP_INFO(
                this->get_logger(),
                "capture fps=%.1f get=%.3fms convert=%.3fms publish=%.3fms mode=%s",
                fps,
                total_get_ms / stat_count,
                total_convert_ms / stat_count,
                total_publish_ms / stat_count,
                publish_camera_info_ ? "image+info" : "image-only");
              stat_count = 0;
              total_get_ms = 0.0;
              total_convert_ms = 0.0;
              total_publish_ms = 0.0;
              stat_start = publish_end;
            }
          }

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_conut_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          MV_CC_StopGrabbing(camera_handle_);
          MV_CC_StartGrabbing(camera_handle_);
          fail_conut_++;
        }

        if (fail_conut_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed!");
          rclcpp::shutdown();
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  void applyBinning()
  {
    binning_horizontal_ = this->declare_parameter("binning_horizontal", 2);
    binning_vertical_ = this->declare_parameter("binning_vertical", 2);

    binning_horizontal_ = setBinningAxis("BinningHorizontal", binning_horizontal_);
    binning_vertical_ = setBinningAxis("BinningVertical", binning_vertical_);

    RCLCPP_INFO(
      this->get_logger(), "Binning applied: horizontal=%d vertical=%d",
      binning_horizontal_, binning_vertical_);
  }

  void configureSdkStream()
  {
    int sdk_image_node_num = this->declare_parameter("sdk_image_node_num", 3);
    sdk_image_node_num = std::max(1, std::min(30, sdk_image_node_num));
    nRet = MV_CC_SetImageNodeNum(camera_handle_, static_cast<unsigned int>(sdk_image_node_num));
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "SDK image node num: %d", sdk_image_node_num);
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to set SDK image node num, nRet: [%x]", nRet);
    }

    const double acquisition_frame_rate = this->declare_parameter("acquisition_frame_rate", 0.0);
    if (acquisition_frame_rate > 0.0) {
      nRet = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
      if (nRet != MV_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to enable AcquisitionFrameRate, nRet: [%x]", nRet);
        return;
      }
      nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", acquisition_frame_rate);
      if (nRet == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Acquisition frame rate limit: %.2f fps", acquisition_frame_rate);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to set AcquisitionFrameRate, nRet: [%x]", nRet);
      }
    }
  }

  void configureAdcBitDepth()
  {
    static constexpr const char * kAdcKeys[] = {"ADCBitDepth", "ADCBitsDepth"};
    static constexpr const char * kAdc8BitValues[] = {"ADCBitDepth_8", "ADCBitsDepth_8"};

    for (const char * key : kAdcKeys) {
      MVCC_ENUMVALUE enum_value;
      std::memset(&enum_value, 0, sizeof(enum_value));
      nRet = MV_CC_GetEnumValue(camera_handle_, key, &enum_value);
      if (nRet != MV_OK) {
        continue;
      }

      for (const char * value : kAdc8BitValues) {
        nRet = MV_CC_SetEnumValueByString(camera_handle_, key, value);
        if (nRet == MV_OK) {
          RCLCPP_INFO(this->get_logger(), "%s configured to 8-bit ADC.", key);
          return;
        }
      }

      RCLCPP_WARN(this->get_logger(), "Failed to set %s to 8-bit ADC, nRet: [%x]", key, nRet);
      return;
    }

    RCLCPP_WARN(this->get_logger(), "Camera does not expose an ADC bit-depth enum node.");
  }

  int setBinningAxis(const char * key, int requested)
  {
    MVCC_INTVALUE int_value;
    std::memset(&int_value, 0, sizeof(int_value));
    nRet = MV_CC_GetIntValue(camera_handle_, key, &int_value);
    if (nRet != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "%s is not supported or unavailable, nRet: [%x].", key, nRet);
      return 1;
    }

    int target = std::max(static_cast<int>(int_value.nMin), requested);
    target = std::min(static_cast<int>(int_value.nMax), target);
    const unsigned int inc = int_value.nInc == 0 ? 1 : int_value.nInc;
    const int min_value = static_cast<int>(int_value.nMin);
    target = min_value + ((target - min_value) / static_cast<int>(inc)) * static_cast<int>(inc);

    nRet = MV_CC_SetIntValue(camera_handle_, key, static_cast<unsigned int>(target));
    if (nRet != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to set %s to %d, nRet: [%x]. Keep current=%u",
        key, target, nRet, int_value.nCurValue);
      return static_cast<int>(int_value.nCurValue);
    }

    MVCC_INTVALUE applied_value;
    std::memset(&applied_value, 0, sizeof(applied_value));
    nRet = MV_CC_GetIntValue(camera_handle_, key, &applied_value);
    if (nRet == MV_OK) {
      return static_cast<int>(applied_value.nCurValue);
    }
    return target;
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Exposure auto
    param_desc.description = "Auto exposure mode: 0=Off, 1=Once, 2=Continuous";
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 2;
    int exposure_auto = this->declare_parameter("exposure_auto", 0, param_desc);
    nRet = MV_CC_SetExposureAutoMode(camera_handle_, exposure_auto);
    if (nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set exposure auto mode, nRet: [%x]", nRet);
    } else {
      RCLCPP_INFO(this->get_logger(), "Exposure auto mode: %d", exposure_auto);
    }

    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

    // Balance White Auto
    param_desc.description = "Auto white balance mode: 0=Off, 1=Continuous, 2=Once";
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 2;
    param_desc.integer_range[0].step = 1;
    int balance_white_auto = this->declare_parameter("balance_white_auto", 0, param_desc);
    MV_CC_SetBalanceWhiteAuto(camera_handle_, balance_white_auto);
    RCLCPP_INFO(this->get_logger(), "Balance white auto mode: %d", balance_white_auto);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "exposure_auto") {
        int status = MV_CC_SetExposureAutoMode(camera_handle_, param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure auto mode, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "acquisition_frame_rate") {
        const double frame_rate = param.as_double();
        int status = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", frame_rate > 0.0);
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set AcquisitionFrameRateEnable, status = " + std::to_string(status);
          continue;
        }
        if (frame_rate > 0.0) {
          status = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate);
          if (MV_OK != status) {
            result.successful = false;
            result.reason = "Failed to set acquisition frame rate, status = " + std::to_string(status);
          }
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "balance_white_auto") {
        int status = MV_CC_SetBalanceWhiteAuto(camera_handle_, param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set balance white auto, status = " + std::to_string(status);
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  sensor_msgs::msg::Image image_msg_;

  image_transport::CameraPublisher camera_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  int nRet = MV_OK;
  void * camera_handle_;
  MV_IMAGE_BASIC_INFO img_info_;

  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  bool publish_camera_info_ = true;
  bool enable_timing_log_ = true;
  bool native_bgr_output_ = false;
  bool native_rgb_output_ = false;
  int binning_horizontal_ = 1;
  int binning_vertical_ = 1;

  int fail_conut_ = 0;
  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
