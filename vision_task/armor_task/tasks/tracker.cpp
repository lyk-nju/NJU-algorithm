#include "tracker.hpp"
#include <yaml-cpp/yaml.h>

#include <algorithm> // 如果尚未包含
#include <iostream>
#include <numeric>
#include <tuple>

#include "../tools/math_tools.hpp"

namespace armor_task
{
Tracker::Tracker(const std::string &config_path, PnpSolver &solver)
    : solver_{solver},
      detect_count_(0),
      temp_lost_count_(0),
      state_(TrackerState::LOST),
      pre_state_(TrackerState::LOST),
      last_timestamp_(std::chrono::steady_clock::now()),
      last_self_is_red_(true)
{
    auto yaml = YAML::LoadFile(config_path);
    enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
    min_detect_count_ = yaml["min_detect_count"].as<int>();
    max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();

    // 初始化 temp_lost 阈值（使用配置文件中的值）
    normal_temp_lost_count_ = max_temp_lost_count_;

    ekf_p0_diag_.resize(11);
    ekf_p0_diag_ << 10, 64, 10, 64, 10, 64, 0.4, 100, 0.1, 0.1, 0.1;
    if (yaml["ekf"])
    {
        const auto ekf = yaml["ekf"];
        if (ekf["process_noise_v1"]) ekf_process_noise_v1_ = ekf["process_noise_v1"].as<double>();
        if (ekf["process_noise_v1y"]) ekf_process_noise_v1y_ = ekf["process_noise_v1y"].as<double>();
        if (ekf["process_noise_v2"]) ekf_process_noise_v2_ = ekf["process_noise_v2"].as<double>();
        if (ekf["p0_diag"] && ekf["p0_diag"].IsSequence() && ekf["p0_diag"].size() == 11)
        {
            for (size_t i = 0; i < 11; ++i)
            {
                ekf_p0_diag_[static_cast<int>(i)] = ekf["p0_diag"][i].as<double>();
            }
        }
    }

}

Tracker::Tracker(const YAML::Node &yaml, PnpSolver &solver)
    : solver_{solver},
      detect_count_(0),
      temp_lost_count_(0),
      state_(TrackerState::LOST),
      pre_state_(TrackerState::LOST),
      last_timestamp_(std::chrono::steady_clock::now()),
      last_self_is_red_(true)
{
    enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
    min_detect_count_ = yaml["min_detect_count"].as<int>();
    max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
    normal_temp_lost_count_ = max_temp_lost_count_;

    ekf_p0_diag_.resize(11);
    ekf_p0_diag_ << 10, 64, 10, 64, 10, 64, 0.4, 100, 0.1, 0.1, 0.1;
    if (yaml["ekf"])
    {
        const auto ekf = yaml["ekf"];
        if (ekf["process_noise_v1"]) ekf_process_noise_v1_ = ekf["process_noise_v1"].as<double>();
        if (ekf["process_noise_v1y"]) ekf_process_noise_v1y_ = ekf["process_noise_v1y"].as<double>();
        if (ekf["process_noise_v2"]) ekf_process_noise_v2_ = ekf["process_noise_v2"].as<double>();
        if (ekf["p0_diag"] && ekf["p0_diag"].IsSequence() && ekf["p0_diag"].size() == 11)
        {
            for (size_t i = 0; i < 11; ++i)
            {
                ekf_p0_diag_[static_cast<int>(i)] = ekf["p0_diag"][i].as<double>();
            }
        }
    }
}

std::string Tracker::state() const { return trackerStateName(state_); }

bool Tracker::get_enemy_color(bool iam_red)
{
    if(!init_)
    {
        init_ = true;
        enemy_color_ = iam_red ? Color::blue : Color::red;
        last_self_is_red_ = iam_red;
        return true;
    }
    if (iam_red != last_self_is_red_ )
    {
        enemy_color_ = iam_red ? Color::blue : Color::red;
        last_self_is_red_ = iam_red;
        return true;
    }
    return false;
}

std::vector<Target> Tracker::track(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t)
{
    auto dt = tools::delta_time(t, last_timestamp_);
    last_timestamp_ = t;

    // 过滤掉非我方装甲板
    auto it = std::remove_if(armors.begin(), armors.end(), [&](const Armor &a) { return a.color != enemy_color_; });
    armors.erase(it, armors.end());

    // 优先选择靠近图像中心的装甲板
    const cv::Point2f img_center(1440.0f / 2.0f, 1080.0f / 2.0f);
    std::sort(armors.begin(),
              armors.end(),
              [img_center](const Armor &a, const Armor &b)
              {
                  float d1=(a.center.x-img_center.x)*(a.center.x-img_center.x)+(a.center.y-img_center.y)*(a.center.y-img_center.y);
                  float d2=(b.center.x-img_center.x)*(b.center.x-img_center.x)+(b.center.y-img_center.y)*(b.center.y-img_center.y);
                  return d1 < d2;
              });

    bool found;
    if (state_ == TrackerState::LOST)
    {
        found = set_target(armors, t);
    }

    else
    {
        found = update_target(armors, t);
    }
    pre_state_ = state_;
    state_machine(found);

    if (state_ == TrackerState::LOST) return {};

    std::vector<Target> targets = {target_};
    return targets;
}

void Tracker::state_machine(bool found)
{
    if (state_ == TrackerState::LOST)
    {
        if (!found) return;

        state_ = TrackerState::DETECTING;
        detect_count_ = 1;
    }

    else if (state_ == TrackerState::DETECTING)
    {
        if (found)
        {
            detect_count_++;
            if (detect_count_ >= min_detect_count_) state_ = TrackerState::TRACKING;
        }
        else
        {
            detect_count_ = 0;
            state_ = TrackerState::LOST;
        }
    }

    else if (state_ == TrackerState::TRACKING)
    {
        if (found) return;

        temp_lost_count_ = 1;
        state_ = TrackerState::TEMP_LOST;
    }

    else if (state_ == TrackerState::SWITCHING)
    {
        if (found)
        {
            state_ = TrackerState::DETECTING;
        }
        else
        {
            temp_lost_count_++;
            if (temp_lost_count_ > 200) state_ = TrackerState::LOST;
        }
    }

    else if (state_ == TrackerState::TEMP_LOST)
    {
        if (found)
        {
            // std::cout << "[Tracker] Recovering from temp_lost to tracking" << std::endl;
            state_ = TrackerState::TRACKING;
            temp_lost_count_ = 0; // 重置计数器
        }
        else
        {
            temp_lost_count_++;
            // if (target_.name == ArmorName::outpost)
            //   //前哨站的temp_lost_count需要设置的大一些
            //   max_temp_lost_count_ = outpost_max_temp_lost_count_;
            // else
            max_temp_lost_count_ = normal_temp_lost_count_;

            // 添加调试输出
            if (temp_lost_count_ % 10 == 0)
            { // 每10帧打印一次，避免输出过多
              // std::cout << "[Tracker] temp_lost_count: " << temp_lost_count_ << " / " << max_temp_lost_count_ << std::endl;
            }

            if (temp_lost_count_ > max_temp_lost_count_)
            {
                // std::cout << "[Tracker] temp_lost timeout, switching to lost state" << std::endl;
                state_ = TrackerState::LOST;
            }
        }
    }
}

bool Tracker::set_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t)
{
    if (armors.empty()) return false;

    // for (auto & armor : armors) {
    //   solver_.solvePnP(armor);
    //   armor.grade = armor.p_camera.norm();
    // }

    // std::sort(armors.begin(), armors.end(), [](const Armor & a, const Armor & b) {
    //   return a.grade < b.grade;
    // });

    auto &armor = armors.front();
    solver_.solvePnP(armor);

    //   // 根据兵种优化初始化参数
    //   auto is_balance = (armor.type == ArmorType::big) &&
    //                     (armor.name == ArmorName::three || armor.name == ArmorName::four ||
    //                      armor.name == ArmorName::five);

    //   if (is_balance) {
    //     Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    //     target_ = Target(armor, t, 0.2, 2, P0_dig);
    //   }

    //   else if (armor.name == ArmorName::outpost) {
    //     Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    //     target_ = Target(armor, t, 0.2765, 3, P0_dig);
    //   }

    //   else if (armor.name == ArmorName::base) {
    //     Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    //     target_ = Target(armor, t, 0.3205, 3, P0_dig);
    //   }

    //   else {
    target_ = Target(armor, t, 4, ekf_p0_diag_, ekf_process_noise_v1_, ekf_process_noise_v1y_, ekf_process_noise_v2_);
    //   }

    return true;
}

bool Tracker::update_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t)
{
    target_.predict(t);

    const auto predicted_armors = target_.armor_xyza_list();
    Armor *best_armor = nullptr;
    double best_score = std::numeric_limits<double>::max();

    // 性能优化（P-06）：只对前几个候选做 PnP
    int checked = 0;
    for (auto &armor : armors)
    {
        if (armor.car_num != target_.car_num) continue;
        if (++checked > 3) break;

        if (!solver_.solvePnP(armor)) continue;

        double min_position_error = std::numeric_limits<double>::max();
        double min_angle_error = std::numeric_limits<double>::max();

        for (const auto &xyza : predicted_armors)
        {
            const Eigen::Vector3d predicted_xyz = xyza.head<3>();
            const Eigen::Vector3d predicted_ypd = tools::xyz2ypd(predicted_xyz);
            const double position_error = (armor.p_world - predicted_xyz).norm();
            const double yaw_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3]));
            const double centerline_error = std::abs(tools::limit_rad(armor.ypd_in_world[0] - predicted_ypd[0]));
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
}
