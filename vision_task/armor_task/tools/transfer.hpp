#pragma once

#include "../io/serial_manager.hpp"
#include <geometry_msgs/msg/twist.hpp>

namespace tools
{

// 将 /cmd_vel (geometry_msgs::msg::Twist) 转成用于串口下发的 base_Command
    io::base_Command from_cmd_vel(const geometry_msgs::msg::Twist &twist);

} // namespace tools

