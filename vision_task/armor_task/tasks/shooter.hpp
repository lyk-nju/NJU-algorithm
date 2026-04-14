#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include "../io/dataframe/base_cmd.hpp"
#include <Eigen/Dense>
#include <list>
#include <string>
#include <vector>

#include "aimer.hpp"
#include "target.hpp"

namespace armor_task
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  bool shoot(
    const io::Vision2Cboard & command, const Aimer & aimer,
    const std::vector<Target> & targets, const Eigen::Vector3d & gimbal_ypr);

private:
  io::Vision2Cboard last_command_;
  bool has_last_command_ = false;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
};
}  // namespace armor_task

#endif  // AUTO_AIM__SHOOTER_HPP
