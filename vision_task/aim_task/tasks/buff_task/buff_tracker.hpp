#pragma once

#include "../armor_task/tracker.hpp"
#include "buff_pnp_solver.hpp"
#include "buff_target.hpp"

namespace buff_task
{
class BuffTracker
{
  public:
    BuffTracker(const std::string &config_path, BuffPnpSolver &solver)
        : tracker_(config_path, solver.inner())
    {
    }

    BuffTracker() = default;

    std::string state() const { return tracker_.state(); }

    std::vector<BuffTarget> track(
        std::vector<armor_task::Armor> &armors, std::chrono::steady_clock::time_point t, bool enemy_is_red)
    {
        std::vector<armor_task::Target> targets = tracker_.track(armors, t, enemy_is_red);
        std::vector<BuffTarget> buff_targets;
        buff_targets.reserve(targets.size());
        for (const auto &target : targets)
        {
            buff_targets.emplace_back(target);
        }
        return buff_targets;
    }

  private:
    armor_task::Tracker tracker_;
};
}
