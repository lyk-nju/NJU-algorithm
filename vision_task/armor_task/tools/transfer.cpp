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


io::AimerData from_vis_dec(bool valid, io::JudgerData &judger_data)
{
    io::AimerData aim_result;
    
    aim_result.cmd_valid = valid;
    aim_result.game_time = judger_data.game_time;
    aim_result.self_hp = judger_data.self_hp;

    return aim_result;
}

bool get_color_from_self_id(io::JudgerData judger_data)
{
    if(judger_data.self_id <= 11)
    {
        return true; // 红色
    }
    return false; // 蓝色
} // namespace tools

}

