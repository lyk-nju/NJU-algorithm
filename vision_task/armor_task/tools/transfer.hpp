#pragma once

#include "../io/serial_manager.hpp"
#include <geometry_msgs/msg/twist.hpp>

namespace tools
{
    io::base_Command from_cmd_vel(const geometry_msgs::msg::Twist &twist);
    io::AimerData from_vis_dec(bool vallid, io::JudgerData &judger_data);
    bool get_color_from_self_id(io::JudgerData judger_data);
} // namespace tools

