#include "transfer.hpp"

namespace tools
{

io::base_Command from_cmd_vel(const geometry_msgs::msg::Twist &twist)
{
    io::base_Command cmd;

    // v_x, v_y
    cmd.v_x = twist.linear.x;
    cmd.v_y = twist.linear.y;
    cmd.w_yaw = 0.1 * twist.angular.z;

    return cmd;
}

} // namespace tools


