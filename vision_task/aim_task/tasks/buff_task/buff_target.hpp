#pragma once

#include "../armor_task/target.hpp"

namespace buff_task
{
class BuffTarget : public armor_task::Target
{
  public:
    BuffTarget() = default;
    BuffTarget(const armor_task::Target &target)
        : armor_task::Target(target)
    {
    }

    using armor_task::Target::Target;
};
}
