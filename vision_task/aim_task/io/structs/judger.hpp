#pragma once

namespace io
{

// 游戏模式（上位机决策用）
enum class PlayerMode
{
    MANUAL,     // 手瞄
    ARMOR,      // 装甲板
    LARGE_BUFF, // 大能量机关
    SMALL_BUFF  // 小能量机关
};

// 裁判系统数据
struct JudgerData
{
    int game_time = 0;       // 比赛剩余时间（秒）
    int self_hp = 0;         // 自身血量
    int self_id = 0;         // 本机机器人 ID（红方 1-11，蓝方 12+）
    double bullet_speed = 0; // 当前子弹速度 m/s
};

// 决策模块向 ROS2 发布的数据
struct AimerData
{
    bool cmd_valid = false;
    int game_time = 0;
    int self_hp = 0;
};

} // namespace io
