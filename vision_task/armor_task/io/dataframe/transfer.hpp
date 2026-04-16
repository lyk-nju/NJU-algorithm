#pragma once
#include "struct.hpp"

#include <geometry_msgs/msg/twist.hpp>

// 这里仅做“不同接口之间的数据转换”，不依赖 usb/ros2 的具体实现类（例如 Cboard/ROS2Manager）。
namespace io::transfer
{

// ---------- ROS2 数据 -> dataframe ----------
inline base_command from_cmd_vel(const geometry_msgs::msg::Twist &twist)
{
    base_command cmd{};
    cmd.v_x = 0.5f * static_cast<float>(twist.linear.x);
    cmd.v_y = 0.5f * static_cast<float>(twist.linear.y);
    cmd.w_yaw = 0.0f * static_cast<float>(twist.angular.z);
    return cmd;
}

// ---------- dataframe -> 决策/上层数据 ----------
inline AimerData from_vis_dec(bool cmd_valid, const JudgerData &judger_data)
{
    AimerData out{};
    out.cmd_valid = cmd_valid;
    out.game_time = judger_data.game_time;
    out.self_hp = judger_data.self_hp;
    return out;
}

// ---------- dataframe -> 自身颜色 ----------
// 返回值：true 表示“自身为红方”，false 表示“自身为蓝方”。
inline bool self_is_red_from_id(const io::JudgerData &judger_data)
{
    if(judger_data.self_id <= 11)
    {
        return true; // 红色
    }
    return false; // 蓝色
}


} // namespace io::transfer