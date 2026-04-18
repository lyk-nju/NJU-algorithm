#pragma once

#include "buff_aimer.hpp"
#include "buff_detector.hpp"
#include "buff_pnp_solver.hpp"
#include "buff_target.hpp"
#include "buff_tracker.hpp"
#include "../pipeline_types.hpp"

#include <list>
#include <string>

namespace buff_task
{

class BuffPipeline
{
  public:
    BuffPipeline(const std::string &yolo_model_path, const std::string &algo_config_path);

    io::Vision2Cboard step(const armor_task::FrameBundle &bundle, const armor_task::GameState &gs);

  private:
    BuffDetector detector_;
    BuffPnpSolver pnp_solver_;
    BuffTracker tracker_;
    BuffAimer aimer_;
};

} // namespace buff_task
