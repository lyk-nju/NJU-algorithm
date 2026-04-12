#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "../tools/math_tools.hpp"
#include "trajectory_normal.hpp"

namespace armor_task
{

Aimer::Aimer(const std::string &config_path)
{
    auto yaml = YAML::LoadFile(config_path);
    yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;       // 角度转弧度
    pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;   // 角度转弧度
    comming_angle_ = yaml["comming_angle"].as<double>() / 57.3; // 角度转弧度
    leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3; // 角度转弧度
    high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
    low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
    decision_speed_ = yaml["decision_speed_"].as<double>();
    bullet_speed_ = yaml["bullet_speed"].as<double>();
}

// 重载版本：写死子弹速度，从配置文件读取
io::Vision2Cboard Aimer::aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, bool to_now)
{
    return aim(std::move(targets), timestamp, bullet_speed_, to_now);
}

// 重载版本：使用下位机传入的子弹速度
io::Vision2Cboard Aimer::aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed, bool to_now)
{
    if (targets.empty()) return {false, false, 0, 0};
    auto target = targets.front();

    // 按角速度选择延时参数
    const double delay_time = target.ekf_x()[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

    // 根据时间补偿预测目标位置
    auto future = timestamp;
    if (to_now)
    {
        const double dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
        future += std::chrono::microseconds(static_cast<int>(dt * 1e6));
        target.predict(future);
    }
    else
    {
        const double dt = 0.005 + delay_time; // 检测和触发延时
        future += std::chrono::microseconds(static_cast<int>(dt * 1e6));
        target.predict(future);
    }

    auto aim_point0 = choose_aim_point(target);
    debug_aim_point = aim_point0;
    if (!aim_point0.valid)
    {
        return {false, false, 0, 0};
    }

    const Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
    const double d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
    Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
    if (trajectory0.unsolvable)
    {
        debug_aim_point.valid = false;
        return {false, false, 0, 0};
    }

    // 迭代飞行时间直到收敛
    double prev_fly_time = trajectory0.fly_time;
    Trajectory current_traj = trajectory0;
    std::vector<Target> iteration_target(10, target);

    for (int iter = 0; iter < 10; ++iter)
    {
        const auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
        iteration_target[iter].predict(predict_time);

        auto aim_point = choose_aim_point(iteration_target[iter]);
        debug_aim_point = aim_point;
        if (!aim_point.valid)
        {
            return {false, false, 0, 0};
        }

        const Eigen::Vector3d xyz = aim_point.xyza.head(3);
        const double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
        current_traj = Trajectory(bullet_speed, d, xyz.z());

        if (current_traj.unsolvable)
        {
            debug_aim_point.valid = false;
            return {false, false, 0, 0};
        }

        if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) break;
        prev_fly_time = current_traj.fly_time;
    }

    const Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
    const double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
    const double pitch = -(current_traj.pitch + pitch_offset_); // 世界坐标系下向上为负 pitch
    return {true, false, static_cast<float>(yaw), static_cast<float>(pitch)};
}

AimPoint Aimer::choose_aim_point(const Target &target)
{
    const Eigen::VectorXd ekf_x = target.ekf_x();
    const std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
    const size_t armor_num = armor_xyza_list.size();

    // 未发生跳变时，直接使用当前装甲板
    if (!target.jumped) return {true, armor_xyza_list[0]};

    const auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

    std::vector<double> delta_angle_list;
    for (size_t i = 0; i < armor_num; i++)
    {
        const auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
        delta_angle_list.emplace_back(delta_angle);
    }

    // 非小陀螺模式
    if (std::abs(target.ekf_x()[8]) <= 2)
    {
        std::vector<int> id_list;
        for (size_t i = 0; i < armor_num; i++)
        {
            if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
            id_list.push_back(static_cast<int>(i));
        }

        if (id_list.empty())
        {
            return {false, armor_xyza_list[0]};
        }

        // 两块装甲板都可射时，锁定其中一块避免来回切换
        if (id_list.size() > 1)
        {
            const int id0 = id_list[0], id1 = id_list[1];
            if (lock_id_ != id0 && lock_id_ != id1)
            {
                lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;
            }
            return {true, armor_xyza_list[lock_id_]};
        }

        lock_id_ = -1;
        return {true, armor_xyza_list[id_list[0]]};
    }

    const double coming_angle = comming_angle_;
    const double leaving_angle = leaving_angle_;

    // 小陀螺模式：优先选择正在进入可打击区域的装甲板
    for (size_t i = 0; i < armor_num; i++)
    {
        if (std::abs(delta_angle_list[i]) > coming_angle) continue;
        if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
        if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
    }

    return {false, armor_xyza_list[0]};
}

} // namespace armor_task
