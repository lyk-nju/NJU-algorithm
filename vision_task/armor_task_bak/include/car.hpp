#ifndef CAR_HPP
#define CAR_HPP

#include "armor.hpp"

struct Robot
{
    Armor armor; // 机器人所包含的Armor类
    float omega; // 机器人旋转的角速度
    float vx;    // 机器人在水平方向的线速度
    float ax;    // 机器人在水平方向的线速度
    float vz;    // 机器人在垂直方向的线速度
    float az;    // 机器人在垂直方向的线速度

    // 构造函数
    Robot() = default;
    Robot(const Armor &a) : armor(a), omega(0.0f), vx(0.0f), ax(0.0f), vz(0.0f), az(0.0f) {}
    Robot(const Armor &a, float w = 0.0f, float v_x = 0.0f, float v_z = 0.0f) : armor(a), omega(w), vx(v_x), ax(0.0f), vz(v_z), az(0.0f) {}
};

#endif // CAR_HPP
