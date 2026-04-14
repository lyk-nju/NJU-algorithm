#include "tracker.hpp"
#include <yaml-cpp/yaml.h>

#include <algorithm> // 濡傛灉灏氭湭鍖呭惈
#include <iostream>
#include <numeric>
#include <tuple>

#include "../tools/math_tools.hpp"

namespace armor_task
{
Tracker::Tracker(const std::string &config_path, PnpSolver &solver) : solver_{solver}, detect_count_(0), temp_lost_count_(0), state_{"lost"}, pre_state_{"lost"}, last_timestamp_(std::chrono::steady_clock::now()), last_self_is_red_(true)
{
    auto yaml = YAML::LoadFile(config_path);
    min_detect_count_ = yaml["min_detect_count"].as<int>();
    max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();

    // 鍒濆鍖?temp_lost 闃堝€硷紙浣跨敤閰嶇疆鏂囦欢涓殑鍊硷級
    normal_temp_lost_count_ = max_temp_lost_count_;

}

std::string Tracker::state() const { return state_; }

bool Tracker::get_enemy_color(bool iam_red)
{
    if(!init_)
    {
        init_ = true;
        enemy_color_ = iam_red ? Color::blue : Color::red;
        std::cout << "enemy_color:" << enemy_color_ << std::endl;
        last_self_is_red_ = iam_red;
        return true;
    }
    if (iam_red != last_self_is_red_ )
    {
        enemy_color_ = iam_red ? Color::blue : Color::red;
        std::cout << "enemy_color:" << enemy_color_ << std::endl;
        last_self_is_red_ = iam_red;
        return true;
    }
    return false;
}

std::vector<Target> Tracker::track(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t,bool enemy_is_red)
{
    enemy_color_ = enemy_is_red? Color::red: Color::blue;

    auto dt = tools::delta_time(t, last_timestamp_);
    last_timestamp_ = t;

    // if (state_ != "lost" && dt > 0.1) {
    //   std::cout<<"[Tracker] Large dt: {:.3f}s"<< dt<<std::endl;
    //   state_ = "lost";
    // }

    // 杩囨护鎺夐潪鎴戞柟瑁呯敳鏉?
    // for (const auto &armor : armors)
    // {
    //     std::cout << "Armor1 "  << " color: " << armor.color << " center: (" << armor.center.x << ", " << armor.center.y << ")" << std::endl;
    // }
    auto it = std::remove_if(armors.begin(), armors.end(), [&](const Armor &a) { return a.color != enemy_color_; });
    armors.erase(it, armors.end());

    // for (const auto &armor : armors)
    // {
    //     std::cout << "Armor2 "  << " color: " << armor.color << " center: (" << armor.center.x << ", " << armor.center.y << ")" << std::endl;
    // }

    //std::cout<<"total num of amrmors"<<armors.size()<<std::endl;
    // 杩囨护鍓嶅摠绔欓《閮ㄨ鐢叉澘
    // armors.remove_if([this](const auto_aim::Armor & a) {
    //   return a.name == ArmorName::outpost &&
    //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
    //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
    // });

    // if (armors.size() > 3) {
    //  浼樺厛閫夋嫨闈犺繎鍥惧儚涓績鐨勮鐢叉澘
    const cv::Point2f img_center(1440.0f / 2.0f, 1080.0f / 2.0f);
    std::sort(armors.begin(),
              armors.end(),
              [img_center](const Armor &a, const Armor &b)
              {
                  float d1=(a.center.x-img_center.x)*(a.center.x-img_center.x)+(a.center.y-img_center.y)*(a.center.y-img_center.y);
                  float d2=(b.center.x-img_center.x)*(b.center.x-img_center.x)+(b.center.y-img_center.y)*(b.center.y-img_center.y);
                  return d1 < d2;
              });

    // 淇濈暀鎺掑簭鍚庣殑鍓嶄笁涓?
    // auto it = armors.begin();
    // std::advance(it, 3);
    // armors.erase(it, armors.end());
    //}

    // 鎸変紭鍏堢骇鎺掑簭锛屼紭鍏堢骇鏈€楂樺湪棣栦綅(浼樺厛绾ц秺楂樻暟瀛楄秺灏忥紝1鐨勪紭鍏堢骇鏈€楂?
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

    //     // 鐗瑰埆鏍囪switching鐘舵€佺殑鍑虹幇鍜岀粨鏉?
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

    // // 鍙戞暎妫€娴?
    // if (state_ != "lost" && target_.diverged()) {
    //   //tools::logger()->debug("[Tracker] Target diverged!");
    //   state_ = "lost";
    //   return {};
    // }

    // //鏀舵暃鏁堟灉妫€娴嬶細
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
            temp_lost_count_ = 0; // 閲嶇疆璁℃暟鍣?
        }
        else
        {
            temp_lost_count_++;
            // if (target_.name == ArmorName::outpost)
            //   //鍓嶅摠绔欑殑temp_lost_count闇€瑕佽缃殑澶т竴浜?
            //   max_temp_lost_count_ = outpost_max_temp_lost_count_;
            // else
            max_temp_lost_count_ = normal_temp_lost_count_;

            // 娣诲姞璋冭瘯杈撳嚭
            if (temp_lost_count_ % 10 == 0)
            { // 姣?0甯ф墦鍗颁竴娆★紝閬垮厤杈撳嚭杩囧
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

    //   // 鏍规嵁鍏电浼樺寲鍒濆鍖栧弬鏁?
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

    // 鑷瀯璋冭瘯鍙傛暟
    P0_dig << 10, 64, 10, 64, 10, 64, 0.4, 100, 0.1, 0.1, 0.1;
    target_ = Target(armor, t, 4, P0_dig);
    //   }

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

        if (!solver_._solve_pnp(armor)) continue;

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
// bool Tracker::update_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t)
// {
//     // 1. 棰勬祴蹇呴』鍋?
//     target_.predict(t);

//     if (armors.empty()) return false;

//     // 2. 瀵绘壘鏈€浣冲尮閰嶈鐢叉澘
//     Armor* best_armor = nullptr;
//     float min_dist_sq = 1e10; // 浣跨敤璺濈骞虫柟
//     const cv::Point2f img_center(1440.0f / 2, 1080.0f / 2);

//     for (auto &armor : armors)
//     {
//         // 涓ユ牸鍖归厤 ID
//         if (armor.car_num != target_.car_num) continue;

//         // 绠€鍗曠殑璺濈绛涢€夛紙濡傛灉宸茬粡鏄帓搴忚繃鐨勶紝绗竴涓尮閰嶇殑灏辨槸鏈€濂界殑锛屽彲浠ョ洿鎺?break锛?
//         // 杩欓噷鍋囪 armors 宸茬粡鍦?track 鍑芥暟閲?sort 杩囦簡
//         best_armor = &armor;
//         break; 
        
//         // 濡傛灉涓婇潰娌℃湁 sort锛屽垯闇€瑕佸湪杩欓噷閬嶅巻鎵炬渶杩戠殑锛?
//         /*
//         float d = distance_sq(armor.center, img_center);
//         if (d < min_dist_sq) {
//             min_dist_sq = d;
//             best_armor = &armor;
//         }
//         */
//     }

//     if (!best_armor) return false;

//     // 3. 鍏抽敭浼樺寲锛氬彧瀵广€愬敮涓€銆戠殑鏈€浣宠鐢叉澘鍋?PnP 鍜?Update
//     // 閬垮厤浜嗗鎵€鏈夎鐢叉澘鍋?PnP 鐨勫法澶у紑閿€
//     solver_._solve_pnp(*best_armor);
//     target_.update(*best_armor);

//     return true;
// }
// } // namespace armor_task

} // namespace armor_task

