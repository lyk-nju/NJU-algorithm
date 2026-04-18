#pragma once

#include "../structs/frame.hpp"

#include <chrono>
#include <opencv2/core/mat.hpp>

namespace io
{

/**
 * 图像输入源的统一抽象。
 *
 * 不区分"物理相机"还是"ROS2 话题"，只规定：
 *   - 每次 read() 返回一个完整的 Frame（含 image/timestamp/id/camera_id）
 *   - timestamp 必须在 steady_clock 域，且尽量接近曝光时刻
 *
 * 上层（AutoAimSystem / FrameScheduler）只和这个接口打交道。
 * 多相机场景下，ICameraRig 会聚合多个 IFrameSource 并按策略派发。
 */
class IFrameSource
{
  public:
    virtual ~IFrameSource() = default;

    /// 取下一帧；无新帧时返回 false（调用方可自行等待/重试）
    virtual bool read(Frame &out) = 0;

    // 兼容旧接口：不关心 id/camera_id 的调用者可以直接拿 image+timestamp。
    // 默认实现走新接口一遍，driver 端不需要重写。
    virtual bool read(cv::Mat &image, std::chrono::steady_clock::time_point &timestamp)
    {
        Frame f;
        if (!read(f)) return false;
        image = std::move(f.image);
        timestamp = f.timestamp;
        return true;
    }
};

} // namespace io
