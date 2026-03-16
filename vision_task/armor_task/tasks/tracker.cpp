#include "tracker.hpp"
#include <yaml-cpp/yaml.h>

#include <algorithm> // 如果尚未包含
#include <iostream>
#include <numeric>
#include <tuple>

#include "../tools/math_tools.hpp"

namespace armor_task
{
Tracker::Tracker(const std::string &config_path, PnpSolver &solver) : solver_{solver}, detect_count_(0), temp_lost_count_(0), state_{"lost"}, pre_state_{"lost"}, last_timestamp_(std::chrono::steady_clock::now())
{
    auto yaml = YAML::LoadFile(config_path);
    enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
    min_detect_count_ = yaml["min_detect_count"].as<int>();
    max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();

    // 初始化 temp_lost 阈值（使用配置文件中的值）
    normal_temp_lost_count_ = max_temp_lost_count_;

    // std::cout << "[Tracker] Initialized with temp_lost threshold: " << normal_temp_lost_count_ << std::endl;
}

std::string Tracker::state() const { return state_; }

std::vector<Target> Tracker::track(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t)
{
    auto dt = tools::delta_time(t, last_timestamp_);
    last_timestamp_ = t;

    // if (state_ != "lost" && dt > 0.1) {
    //   std::cout<<"[Tracker] Large dt: {:.3f}s"<< dt<<std::endl;
    //   state_ = "lost";
    // }

    // 过滤掉非我方装甲板
    auto it = std::remove_if(armors.begin(), armors.end(), [&](const Armor &a) { return a.color != enemy_color_; });
    armors.erase(it, armors.end());

    // 过滤前哨站顶部装甲板
    // armors.remove_if([this](const auto_aim::Armor & a) {
    //   return a.name == ArmorName::outpost &&
    //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
    //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
    // });

    // if (armors.size() > 3) {
    //  优先选择靠近图像中心的装甲板
    std::sort(armors.begin(),
              armors.end(),
              [](const Armor &a, const Armor &b)
              {
                  cv::Point2f img_center(1440 / 2, 1080 / 2); // TODO
                  auto distance_1 = cv::norm(a.center - img_center);
                  auto distance_2 = cv::norm(b.center - img_center);
                  return distance_1 < distance_2;
              });

    // 保留排序后的前三个
    // auto it = armors.begin();
    // std::advance(it, 3);
    // armors.erase(it, armors.end());
    //}

    // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
    //   armors.sort(
    //     [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

    bool found;
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

    // if (state_ != pre_state_)
    // {
    //     std::cout << "[Tracker Internal] State change: " << pre_state_ << " -> " << state_ << std::endl;

    //     // 特别标记switching状态的出现和结束
    //     if (state_ == "switching")
    //     {
    //         std::cout << "[Tracker Internal] !!! SWITCHING state ENTERED !!!" << std::endl;
    //     }
    //     if (pre_state_ == "switching")
    //     {
    //         std::cout << "[Tracker Internal] !!! SWITCHING state EXITED !!!" << std::endl;
    //     }
    // }

    // std::cout << "PRESTATE:" << pre_state_ << std::endl;
    // std::cout << "STATE:" << state_ << std::endl;

    // // 发散检测
    // if (state_ != "lost" && target_.diverged()) {
    //   //tools::logger()->debug("[Tracker] Target diverged!");
    //   state_ = "lost";
    //   return {};
    // }

    // //收敛效果检测：
    // if (
    //   std::accumulate(
    //     target_.ekf().nis_failures.begin(), target_.ekf().nis_failures.end(), 0) >=
    //   (0.4 * target_.ekf().window_size)) {
    //   std::cout << "[Target] Bad Converge Found!" << std::endl;
    //   state_ = "lost";
    //   return {};
    // }

    if (state_ == "lost") return {};

    std::vector<Target> targets = {target_};
    return targets;
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
            // std::cout << "[Tracker] Recovering from temp_lost to tracking" << std::endl;
            state_ = "tracking";
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
                state_ = "lost";
            }
        }
    }
}

bool Tracker::set_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t)
{
    if (armors.empty()) return false;

    // for (auto & armor : armors) {
    //   solver_._solve_pnp(armor);
    //   armor.grade = armor.p_camera.norm();
    // }

    // std::sort(armors.begin(), armors.end(), [](const Armor & a, const Armor & b) {
    //   return a.grade < b.grade;
    // });

    auto &armor = armors.front();
    solver_._solve_pnp(armor);

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
    Eigen::VectorXd P0_dig(11);

    // 自瞄调试参数
    P0_dig << 10, 64, 10, 64, 10, 64, 0.4, 100, 0.1, 0.1, 0.1;
    target_ = Target(armor, t, 4, P0_dig);
    //   }

    return true;
}

bool Tracker::update_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t)
{
    target_.predict(t);

    int found_count = 0;
    // double min_x = 1e10;  // 画面最左侧

    // 调试输出：当前追踪的目标信息
    // std::cout << "[Tracker] Updating target, looking for CarNum=" << target_.car_num << " | Available armors: " << armors.size() << std::endl;

    for (const auto &armor : armors)
    {
        // std::cout << "  Armor CarNum=" << armor.car_num << " (match: " << (armor.car_num == target_.car_num ? "YES" : "NO") << ")" << std::endl;

        if (armor.car_num != target_.car_num) continue;
        found_count++;
        // min_x = armor.center.x < min_x ? armor.center.x : min_x;
    }

    // std::cout << "[Tracker] Found " << found_count << " matching armors" << std::endl;

    if (found_count == 0) return false;

    for (auto &armor : armors)
    {
        if (armor.car_num != target_.car_num
            //  || armor.center.x != min_x
        )
            continue;

        solver_._solve_pnp(armor);

        target_.update(armor);
    }

    return true;
}

} // namespace armor_task