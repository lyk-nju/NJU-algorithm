#pragma once

#include "gimbal.hpp"
#include "judger.hpp"

namespace io
{

// 上位机发送给下位机的数据帧
struct Vision2Cboard
{
    gimbal_command gimbal_cmd_;
    base_command base_cmd_;
};

// 下位机发送给上位机的数据帧
struct Cboard2Vision
{
    PlayerMode mode_ = PlayerMode::MANUAL;
    GimbalData gimbal_data_;
    JudgerData judge_;
};

} // namespace io
