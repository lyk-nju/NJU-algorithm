# pragma once
namespace io{

// ------------------------- 上位机控制指令 -------------------------

//云台控制命令
struct gimbal_command
{
    bool valid;
    bool shoot;
    float yaw = 0.0f;
    float pitch = 0.0f;
};

//底盘控制命令
struct base_command
{
    float v_x = 0.0f;  
    float v_y = 0.0f;
    float w_yaw = 0.0f;
};

// ------------------------- 下位机反馈数据 -------------------------

//云台状态数据
struct GimbalData
{
    double w;
    double x;
    double y;
    double z;
    float yaw = 0.0f;
    float pitch = 0.0f;
};

//裁判系统数据
struct JudgerData
{
    int game_time = 0; // 比赛时间（如剩余秒数，由串口协议约定）
    int self_hp = 0;   // 自身血量
    int self_id = 0; 
    double bullet_speed = 0; //子弹速度，新增
};

enum class PlayerMode
{
    MANUAL, //手瞄
    ARMOR,  //装甲板
    LARGE_BUFF, //大能量机关
    SMALL_BUFF //小能量机关
};

// ------------------------- 上下位机通信数据帧 -------------------------

//下位机发送给上位机的数据帧
struct Vision2Cboard
{
    gimbal_command gimbal_cmd_;
    base_command base_cmd_;
};

//上位机发送给下位机的数据帧
struct Cboard2Vision
{
    PlayerMode mode_;
    GimbalData gimbal_data_;
    JudgerData judge_;
};

// ------------------------- 决策模块数据 -------------------------

//决策模块数据
struct AimerData
{
    bool cmd_valid = false;
    int game_time = 0; // 比赛时间（如剩余秒数，由串口协议约定）
    int self_hp = 0; 
};


}; //namespace io
