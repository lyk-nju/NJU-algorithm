#include "codec.hpp"

#include <cstdio>
#include <iomanip>
#include <sstream>

namespace io::codec
{

PlayerMode decode_mode(int mode_value)
{
    switch (mode_value)
    {
    case 1:
        return PlayerMode::ARMOR;
    case 2:
        return PlayerMode::LARGE_BUFF;
    case 3:
        return PlayerMode::SMALL_BUFF;
    case 0:
    default:
        return PlayerMode::MANUAL;
    }
}

std::string encode(const Vision2Cboard &data)
{
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setprecision(6)
       << (data.gimbal_cmd_.valid ? 1 : 0) << ","
       << (data.gimbal_cmd_.shoot ? 1 : 0) << ","
       << data.gimbal_cmd_.yaw << ","
       << data.gimbal_cmd_.pitch << ","
       << data.base_cmd_.v_x << ","
       << data.base_cmd_.v_y << ","
       << data.base_cmd_.w_yaw << "\n";
    return ss.str();
}

bool decode(const std::string &line, Cboard2Vision &out)
{
    int mode = 0;
    double w = 0.0, x = 0.0, y = 0.0, z = 0.0, yaw = 0.0, pitch = 0.0, bullet_speed = 0.0;
    int game_time = 0, self_hp = 0, self_id = 0;
    const int count = std::sscanf(
        line.c_str(),
        "%d,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%lf",
        &mode, &w, &x, &y, &z, &yaw, &pitch,
        &game_time, &self_hp, &self_id, &bullet_speed);

    if (count < 11) return false;

    out.mode_ = decode_mode(mode);
    out.gimbal_data_ = GimbalData{w, x, y, z, static_cast<float>(yaw), static_cast<float>(pitch)};
    out.judge_ = JudgerData{game_time, self_hp, self_id, bullet_speed};
    return true;
}

} // namespace io::codec
