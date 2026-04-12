#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "pnp_solver.hpp"
#include "target.hpp"

namespace armor_task
{
class Tracker
{
  public:
    Tracker(const std::string &config_path, PnpSolver &solver);
    Tracker() = default;

    std::string state() const;

    std::vector<Target> track(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t,bool enemy_is_red);

    bool get_enemy_color(bool iam_red);


  private:
    PnpSolver &solver_;
    Color enemy_color_;
    bool init_ = false;
    bool last_self_is_red_;
    int min_detect_count_;
    int max_temp_lost_count_;
    int detect_count_;
    int normal_temp_lost_count_;
    int temp_lost_count_;
    std::string state_, pre_state_;
    Target target_;
    std::chrono::steady_clock::time_point last_timestamp_;

    void state_machine(bool found);

    bool set_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t);

    bool update_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t);
};

} // namespace armor_task

#endif // AUTO_AIM__TRACKER_HPP
