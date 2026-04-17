#pragma once

#include "../structs/judger.hpp"
#include "../structs/protocol.hpp"

#include <geometry_msgs/msg/twist.hpp>

// ROS2 消息 <-> io::structs 之间的纯转换函数。
// 把 ROS2 依赖隔离到算法层的这一个文件里，structs/ 层保持零依赖。
namespace io::transfer
{

// ROS2 消息 -> 内部结构体
inline base_command from_cmd_vel(const geometry_msgs::msg::Twist &twist)
{
    base_command cmd{};
    cmd.v_x = 0.5f * static_cast<float>(twist.linear.x);
    cmd.v_y = 0.5f * static_cast<float>(twist.linear.y);
    cmd.w_yaw = 0.0f * static_cast<float>(twist.angular.z);
    return cmd;
}

// 内部结构体 -> 决策/上层数据
inline AimerData from_vis_dec(bool cmd_valid, const JudgerData &judger_data)
{
    AimerData out{};
    out.cmd_valid = cmd_valid;
    out.game_time = judger_data.game_time;
    out.self_hp = judger_data.self_hp;
    return out;
}

// 由 judger 上报的 self_id 推算自身颜色（true = 红方）
inline bool self_is_red_from_id(const io::JudgerData &judger_data)
{
    return judger_data.self_id <= 11;
}

} // namespace io::transfer
