#include "buff_pipeline.hpp"

namespace buff_task
{

BuffPipeline::BuffPipeline(const std::string &yolo_model_path, const std::string &algo_config_path)
    : detector_(yolo_model_path),
      pnp_solver_(algo_config_path),
      tracker_(algo_config_path, pnp_solver_),
      aimer_(algo_config_path)
{
}

io::Vision2Cboard BuffPipeline::step(const armor_task::FrameBundle &bundle, const armor_task::GameState &gs)
{
    pnp_solver_.set_camera(*bundle.camera);
    pnp_solver_.set_R_gimbal2world(bundle.gimbal_quat);

    armor_task::ArmorArray armors = detector_.detect(bundle.frame.image);
    std::vector<BuffTarget> targets = tracker_.track(armors, bundle.frame.timestamp, gs.enemy_is_red);

    io::Vision2Cboard cmd{};
    if (targets.empty())
    {
        return cmd;
    }

    std::list<BuffTarget> target_list(targets.begin(), targets.end());
    cmd = aimer_.aim(target_list, bundle.frame.timestamp, gs.bullet_speed);
    cmd.gimbal_cmd_.shoot = false;
    return cmd;
}

} // namespace buff_task
