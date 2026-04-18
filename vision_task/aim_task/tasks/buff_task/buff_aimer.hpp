#pragma once

#include "../armor_task/aimer.hpp"
#include "buff_target.hpp"

namespace buff_task
{
struct BuffAimPoint : public armor_task::AimPoint
{
    BuffAimPoint() = default;
    BuffAimPoint(const armor_task::AimPoint &aim_point)
        : armor_task::AimPoint(aim_point)
    {
    }
};

class BuffAimer
{
  public:
    BuffAimPoint debug_aim_point;

    explicit BuffAimer(const std::string &config_path)
        : aimer_(config_path)
    {
    }

    io::Vision2Cboard aim(std::list<BuffTarget> targets, std::chrono::steady_clock::time_point timestamp, bool to_now = true)
    {
        std::list<armor_task::Target> inner_targets;
        for (const auto &target : targets)
        {
            inner_targets.emplace_back(static_cast<const armor_task::Target &>(target));
        }
        io::Vision2Cboard cmd = aimer_.aim(inner_targets, timestamp, to_now);
        debug_aim_point = BuffAimPoint(aimer_.debug_aim_point);
        return cmd;
    }

    io::Vision2Cboard aim(std::list<BuffTarget> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed, bool to_now = true)
    {
        std::list<armor_task::Target> inner_targets;
        for (const auto &target : targets)
        {
            inner_targets.emplace_back(static_cast<const armor_task::Target &>(target));
        }
        io::Vision2Cboard cmd = aimer_.aim(inner_targets, timestamp, bullet_speed, to_now);
        debug_aim_point = BuffAimPoint(aimer_.debug_aim_point);
        return cmd;
    }

    double bullet_speed() const { return aimer_.bullet_speed_; }
    void set_bullet_speed(double bullet_speed) { aimer_.bullet_speed_ = bullet_speed; }

  private:
    armor_task::Aimer aimer_;
};
}
