#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "../io/serial_manager.hpp"
#include "target.hpp"

// aimer内部只管yaw角的计算，pitch角的计算交给trajectory模块

namespace armor_task
{
using namespace io;

struct AimPoint
{
    bool valid;
    Eigen::Vector4d xyza; // a represent yaw
};

class Aimer
{
  public:
    AimPoint debug_aim_point;
    explicit Aimer(const std::string &config_path);
    io::Vision2Cboard aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, bool to_now = true);
    io::Vision2Cboard aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed, bool to_now = true);

    double bullet_speed_;
  private:
    double yaw_offset_;
    // std::optional<double> left_yaw_offset_, right_yaw_offset_;
    double pitch_offset_;
    double comming_angle_;
    double leaving_angle_;
    double lock_id_ = -1;
    double high_speed_delay_time_;
    double low_speed_delay_time_;
    double decision_speed_;
    

    AimPoint choose_aim_point(const Target &target);
};
} // namespace armor_task

#endif // AUTO_AIM__AIMER_HPP
