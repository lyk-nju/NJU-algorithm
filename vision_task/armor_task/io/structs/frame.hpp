#pragma once

#include <chrono>
#include <cstdint>
#include <opencv2/core/mat.hpp>

namespace io
{

// 统一的图像帧结构：所有 IFrameSource / ICamera::read 的返回值。
// 多相机场景下用 camera_id 区分来源；单相机时默认为 0。
struct Frame
{
    cv::Mat image;
    std::chrono::steady_clock::time_point timestamp;
    uint64_t id = 0;       // 帧序号，便于去重 / 调试
    int camera_id = 0;     // 全系统唯一的相机 ID（0..N-1）
};

} // namespace io
