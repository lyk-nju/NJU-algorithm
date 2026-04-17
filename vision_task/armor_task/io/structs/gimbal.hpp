#pragma once

namespace io
{

// 云台控制命令（上位机 → 下位机）
struct gimbal_command
{
    bool valid = false;
    bool shoot = false;
    float yaw = 0.0f;
    float pitch = 0.0f;
};

// 底盘控制命令（上位机 → 下位机）
struct base_command
{
    float v_x = 0.0f;
    float v_y = 0.0f;
    float w_yaw = 0.0f;
};

// 云台状态数据（下位机 → 上位机）
struct GimbalData
{
    // IMU 四元数
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    float yaw = 0.0f;
    float pitch = 0.0f;
};

} // namespace io
