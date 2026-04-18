#pragma once

#include "../../interface/i_frame_source.hpp"
#include "ros2_manager.hpp"

#include <cstdint>
#include <memory>

namespace io
{

/**
 * @brief ROS2 话题图像源，从 ROS2Manager 订阅的 /image_raw 缓存中取帧。
 *
 * 时间戳由 ROS2Manager 在图像回调中将 ROS header.stamp (system_clock)
 * 换算为 steady_clock 得到，后续用 Cboard::q(t) 做 slerp 对齐。
 *
 * 直接实现 io::IFrameSource；当前没有物理相机概念，因此不继承 ICamera。
 */
class Ros2ImageSource : public IFrameSource
{
  public:
    explicit Ros2ImageSource(std::shared_ptr<ROS2Manager> node);

    bool read(Frame &out) override;

  private:
    std::shared_ptr<ROS2Manager> node_;
    int camera_id_ = 0;
    uint64_t frame_counter_ = 0;
};

} // namespace io
