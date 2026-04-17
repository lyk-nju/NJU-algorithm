#pragma once

#include "../structs/camera_info.hpp"
#include "i_frame_source.hpp"

namespace io
{

/**
 * 物理相机抽象。继承 IFrameSource，额外要求能报告自己的 ID / 内外参。
 *
 * 预留给多相机场景：上层可以通过 info() 拿到每一路的 CameraInfo，用于：
 *   - PnpSolver  选择对应内参做解算
 *   - 坐标变换   从 camera 坐标系到 gimbal 坐标系
 *
 * 注：ROS2 话题来源的图像走 IFrameSource 就够了，不需要实现 ICamera。
 */
class ICamera : public IFrameSource
{
  public:
    virtual int camera_id() const = 0;
    virtual const CameraInfo &info() const = 0;
};

} // namespace io
