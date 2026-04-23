#include "tracker.hpp"

#include "../../tools/math_tools.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <iostream>
#include <limits>

namespace armor_task
{
Tracker::Tracker(const std::string &config_path, PnpSolver &solver)
    : solver_{solver},
      detect_count_(0),
      temp_lost_count_(0),
      state_{"lost"},
      pre_state_{"lost"},
      last_timestamp_(std::chrono::steady_clock::now())
{
    auto yaml = YAML::LoadFile(config_path);
    min_detect_count_ = yaml["min_detect_count"].as<int>();
    max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();

    // 普通兵种的 temp_lost 阈值（从配置读入）
    normal_temp_lost_count_ = max_temp_lost_count_;
}

std::string Tracker::state() const { return state_; }

std::vector<Target> Tracker::track(
    std::vector<Armor> &armors, std::chrono::steady_clock::time_point t, bool enemy_is_red)
{
    enemy_color_ = enemy_is_red ? Color::red : Color::blue;

    last_timestamp_ = t;

    // 过滤非敌方颜色的装甲
    auto it = std::remove_if(armors.begin(), armors.end(),
                             [&](const Armor &a) { return a.color != enemy_color_; });
    armors.erase(it, armors.end());

    // 按靠近图像中心的优先级排序（近中心的目标优先被选为 track 目标
    const cv::Point2f img_center(1440.0f / 2.0f, 1080.0f / 2.0f);
    std::sort(armors.begin(), armors.end(),
              [img_center](const Armor &a, const Armor &b)
              {
                  const float d1 = (a.center.x - img_center.x) * (a.center.x - img_center.x) +
                                   (a.center.y - img_center.y) * (a.center.y - img_center.y);
                  const float d2 = (b.center.x - img_center.x) * (b.center.x - img_center.x) +
                                   (b.center.y - img_center.y) * (b.center.y - img_center.y);
                  return d1 < d2;
              });

    bool found = false;
    if (state_ == "lost")
    {
        found = set_target(armors, t);
    }
    else
    {
        found = update_target(armors, t);
    }

    pre_state_ = state_;
    state_machine(found);

    if (state_ == "lost") return {};
    return {target_};
}

void Tracker::state_machine(bool found)
{
    if (state_ == "lost")
    {
        if (!found) return;

        state_ = "detecting";
        detect_count_ = 1;
    }

    else if (state_ == "detecting")
    {
        if (found)
        {
            detect_count_++;
            if (detect_count_ >= min_detect_count_) state_ = "tracking";
        }
        else
        {
            detect_count_ = 0;
            state_ = "lost";
        }
    }

    else if (state_ == "tracking")
    {
        if (found) return;

        temp_lost_count_ = 1;
        state_ = "temp_lost";
    }

    else if (state_ == "switching")
    {
        if (found)
        {
            state_ = "detecting";
        }
        else
        {
            temp_lost_count_++;
            if (temp_lost_count_ > 200) state_ = "lost";
        }
    }

    else if (state_ == "temp_lost")
    {
        if (found)
        {
            state_ = "tracking";
            temp_lost_count_ = 0;
        }
        else
        {
            temp_lost_count_++;
            max_temp_lost_count_ = normal_temp_lost_count_;
            if (temp_lost_count_ > max_temp_lost_count_)
            {
                state_ = "lost";
            }
        }
    }
}

bool Tracker::set_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t)
{
    if (armors.empty()) return false;

    // 选取排序后最靠近图像中心的装甲板作为初始目标
    auto &armor = armors.front();
    solver_.solve_pnp(armor);

    // 初始协方差对角线：位??/ 速度 / yaw / 角速度 / r / dl / dh
    Eigen::VectorXd P0_dig(11);
    P0_dig << 10, 64, 10, 64, 10, 64, 0.4, 100, 0.1, 0.1, 0.1;
    target_ = Target(armor, t, 4, P0_dig);
    return true;
}

bool Tracker::update_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t)
{
    target_.predict(t);

    const auto predicted_armors = target_.armor_xyza_list();
    Armor *best_armor = nullptr;
    double best_score = std::numeric_limits<double>::max();

    for (auto &armor : armors)
    {
        if (armor.car_num != target_.car_num) continue;
        if (!solver_.solve_pnp(armor)) continue;

        double min_position_error = std::numeric_limits<double>::max();
        double min_angle_error = std::numeric_limits<double>::max();

        for (const auto &xyza : predicted_armors)
        {
            const Eigen::Vector3d predicted_xyz = xyza.head<3>();
            const Eigen::Vector3d predicted_ypd = tools::xyz2ypd(predicted_xyz);
            const double position_error = (armor.p_world - predicted_xyz).norm();
            const double yaw_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3]));
            const double centerline_error =
                std::abs(tools::limit_rad(armor.ypd_in_world[0] - predicted_ypd[0]));
            const double angle_error = yaw_error + centerline_error;

            min_position_error = std::min(min_position_error, position_error);
            min_angle_error = std::min(min_angle_error, angle_error);
        }

        const double score = min_position_error + 0.35 * min_angle_error;
        if (score < best_score)
        {
            best_score = score;
            best_armor = &armor;
        }
    }

    if (best_armor == nullptr) return false;

    target_.update(*best_armor);
    return true;
}

} // namespace armor_task
