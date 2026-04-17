#pragma once

#include "../../interface/i_frame_source.hpp"
#include "camera_base.hpp"

#include <cstdint>
#include <string>

namespace io
{

/**
 * @brief USB 直连相机（走 io::Camera -> Hik）作为图像源。
 *
 * 时间戳由相机采集线程在抓帧回调处打上 steady_clock::now()，
 * 用于后续 Cboard::q(t) 做 slerp 对齐。
 *
 * 直接实现 io::IFrameSource；多相机场景下可以改造为 ICamera（携带 CameraInfo / camera_id）。
 */
class DirectImageSource : public IFrameSource
{
  public:
    explicit DirectImageSource(const std::string &config_path);

    bool read(Frame &out) override;

  private:
    Camera camera_;
    int camera_id_ = 0;
    uint64_t frame_counter_ = 0;
};

} // namespace io
