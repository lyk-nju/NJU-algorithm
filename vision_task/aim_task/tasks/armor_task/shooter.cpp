#include "shooter.hpp"

#include <cmath>
#include <yaml-cpp/yaml.h>

namespace armor_task
{
Shooter::Shooter(const std::string & config_path)
  : last_command_{io::gimbal_command{false, false, 0.0f, 0.0f}, io::base_command{}}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ =
    yaml["first_tolerance"] ? (yaml["first_tolerance"].as<double>() / 57.3) : (2.0 / 57.3);
  second_tolerance_ =
    yaml["second_tolerance"] ? (yaml["second_tolerance"].as<double>() / 57.3) : (3.0 / 57.3);
  judge_distance_ = yaml["judge_distance"] ? yaml["judge_distance"].as<double>() : 3.0;
}

bool Shooter::shoot(
  const io::Vision2Cboard & command, const Aimer & aimer,
  const std::vector<Target> & targets, const Eigen::Vector3d & gimbal_ypr)
{
  if (!command.gimbal_cmd_.valid || targets.empty() || !aimer.debug_aim_point.valid) {
    has_last_command_ = false;
    return false;
  }

  const auto target_x = targets.front().ekf_x()[0];
  const auto target_y = targets.front().ekf_x()[2];
  const auto tolerance = std::hypot(target_x, target_y) > judge_distance_
                           ? second_tolerance_
                           : first_tolerance_;

  if (!has_last_command_) {
    last_command_ = command;
    has_last_command_ = true;
    return false;
  }

  const bool command_stable =
    std::abs(last_command_.gimbal_cmd_.yaw - command.gimbal_cmd_.yaw) < tolerance * 2.0;
  const bool gimbal_close = std::abs(gimbal_ypr[0] - command.gimbal_cmd_.yaw) < tolerance;

  last_command_ = command;
  return command_stable && gimbal_close;
}

}  // namespace armor_task
