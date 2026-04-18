#pragma once

#include "../armor_task/ekf.hpp"

namespace buff_task
{
class BuffEkf : public armor_task::Ekf
{
  public:
    BuffEkf() = default;
    BuffEkf(const armor_task::Ekf &ekf)
        : armor_task::Ekf(ekf)
    {
    }

    using armor_task::Ekf::Ekf;
};
}
