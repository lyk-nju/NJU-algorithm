# pragma once
namespace io{
//定义上下位机通信数据帧格式

//速度与角速度信息，主要是哨兵用
struct base_Command
{
    float v_x = 0.0f;  
    float v_y = 0.0f;
    float w_yaw = 0.0f;
};

//裁判系统数据
struct JudgerData
{
    int game_time = 0; // 比赛时间（如剩余秒数，由串口协议约定）
    int self_hp = 0;   // 自身血量
    int self_id = 0; 
    double bullet_speed = 0; //子弹速度，新增
};

struct Vision2Cboard{
    bool valid;
    bool shoot;
    double yaw;
    double pitch;
    base_Command base_cmd_;
};

enum class PlayerMode{
    MANUAL, //手瞄
    ARMOR,  //装甲板
    LARGE_BUFF, //大能量机关
    SMALL_BUFF //小能量机关
};



struct Cboard2Vision
{
    PlayerMode mode_;
    double w;
    double x;
    double y;
    double z;
    double yaw;
    double pitch;
    JudgerData judge_;
};
}; //namespace io
