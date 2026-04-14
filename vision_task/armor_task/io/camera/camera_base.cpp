#include "camera_base.hpp"

#include <stdexcept>

#include "hik/hik.hpp"
#include <yaml-cpp/yaml.h>

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto camera_name = yaml["camera_name"].as<std::string>();

  if (camera_name == "hik") {
    camera_ = std::make_unique<Hik>(config_path);
  }

  else if (camera_name == "usbcamera") {
    throw std::runtime_error("usbcamera is not implemented in current camera factory.");
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
