#include "transfer.hpp"

namespace tools
{

io::base_Command from_cmd_vel(const geometry_msgs::msg::Twist &twist)
{
    io::base_Command cmd;
    cmd.values.clear();
    cmd.values.reserve(6);

    // 固定顺序编码 /cmd_vel：linear(x,y,z), angular(x,y,z)
    cmd.values.push_back(twist.linear.x);
    cmd.values.push_back(twist.linear.y);
    cmd.values.push_back(twist.linear.z);
    cmd.values.push_back(twist.angular.x);
    cmd.values.push_back(twist.angular.y);
    cmd.values.push_back(twist.angular.z);

    return cmd;
}

} // namespace tools

