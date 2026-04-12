#pragma once
#include"../usb/cboard.hpp"
#include <geometry_msgs/msg/twist.hpp>

//原transfer部分暂时移动至此处

namespace io{

    struct AimerData
{
    bool cmd_valid = false;
    int game_time = 0; // 比赛时间（如剩余秒数，由串口协议约定）
    int self_hp = 0; 
};

base_Command from_cmd_vel(const geometry_msgs::msg::Twist &twist)
{
    io::base_Command cmd;

    // v_x, v_y
    cmd.v_x = 0.5 * twist.linear.x;
    cmd.v_y = 0.5 * twist.linear.y;
    cmd.w_yaw = 0.0 * twist.angular.z;

    return cmd;
}


AimerData from_vis_dec(bool valid, JudgerData &judger_data)
{
    AimerData aim_result;
    
    aim_result.cmd_valid = valid;
    aim_result.game_time = judger_data.game_time;
    aim_result.self_hp = judger_data.self_hp;

    return aim_result;
}
};