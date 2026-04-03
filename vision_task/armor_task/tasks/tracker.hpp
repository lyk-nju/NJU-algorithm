#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>
#include <yaml-cpp/yaml.h>

#include "armor.hpp"
#include "pnp_solver.hpp"
#include "target.hpp"

namespace armor_task
{
enum class TrackerState
{
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
    SWITCHING,
};

inline const char *trackerStateName(TrackerState s)
{
    switch (s)
    {
    case TrackerState::LOST: return "lost";
    case TrackerState::DETECTING: return "detecting";
    case TrackerState::TRACKING: return "tracking";
    case TrackerState::TEMP_LOST: return "temp_lost";
    case TrackerState::SWITCHING: return "switching";
    }
    return "unknown";
}

class Tracker
{
  public:
    Tracker(const std::string &config_path, PnpSolver &solver);
    Tracker(const YAML::Node &config, PnpSolver &solver);
    Tracker() = default;

    std::string state() const;
    TrackerState state_enum() const { return state_; }

    std::vector<Target> track(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t);

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
    TrackerState state_, pre_state_;
    Target target_;
    std::chrono::steady_clock::time_point last_timestamp_;

    // EKF 配置（A-09）
    double ekf_process_noise_v1_ = 100.0;
    double ekf_process_noise_v1y_ = 100.0;
    double ekf_process_noise_v2_ = 400.0;
    Eigen::VectorXd ekf_p0_diag_{Eigen::VectorXd::Ones(11)};

    void state_machine(bool found);

    bool set_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t);

    bool update_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t);
};

} // namespace armor_task

#endif // AUTO_AIM__TRACKER_HPP
