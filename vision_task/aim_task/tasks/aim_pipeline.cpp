#include "aim_pipeline.hpp"

#include "../tools/math_tools.hpp"

#include <list>
#include <utility>

namespace armor_task
{

AimPipeline::AimPipeline(const std::string &yolo_model_path, const std::string &algo_config_path)
    : detector_(yolo_model_path),
      pnp_solver_(algo_config_path),
      tracker_(algo_config_path, pnp_solver_),
      aimer_(algo_config_path),
      shooter_(algo_config_path)
{
}

AimDecision AimPipeline::step(const FrameBundle &bundle, const GameState &gs)
{
    return step_impl(bundle, gs, nullptr);
}

AimDecision AimPipeline::step(const FrameBundle &bundle, const GameState &gs, DebugSnapshot &debug)
{
    return step_impl(bundle, gs, &debug);
}

AimDecision AimPipeline::step_impl(const FrameBundle &b, const GameState &gs, DebugSnapshot *debug)
{
    // (1) 注入当前相机参数 + 云台姿态。
    //     - bundle.camera 必须非空（契约由上层 AutoAimSystem / FrameScheduler 保证）；
    //     - 单相机场景下 bundle.camera 每帧指向同一份数据，set_camera 与构造期赋值
    //       的数值完全一致，不引入新的浮点差异；
    //     - 多相机场景下 set_camera 会随 frame.camera_id 切换，对应 CameraInfo 必须
    //       和 frame.camera_id 匹配。
    pnp_solver_.set_camera(*b.camera);
    pnp_solver_.set_R_gimbal2world(b.gimbal_quat);

    // (2) 检测
    ArmorArray armors = detector_.detect(b.frame.image);

    // (3) 追踪（Tracker 内部会自行调用 solve_pnp）
    std::vector<Target> targets = tracker_.track(armors, b.frame.timestamp, gs.enemy_is_red);

    AimDecision decision{};

    if (targets.empty())
    {
        if (debug)
        {
            debug->detected_armors = std::move(armors);
            debug->targets.clear();
            debug->debug_aim_point = {};
        }
        return decision;
    }

    // (4) 瞄准。保持原有 std::list 传参语义（Aimer::aim 只用 front()，但改接口
    //     会动算法边界，违反 Step A 硬约束，留给 Step B）。
    std::list<Target> target_list(targets.begin(), targets.end());
    decision.cmd = aimer_.aim(target_list, b.frame.timestamp, gs.bullet_speed);

    // (5) 开火判断
    const Eigen::Vector3d gimbal_ypr = tools::eulers(pnp_solver_.R_gimbal2world_, 2, 1, 0);
    decision.cmd.gimbal_cmd_.shoot = shooter_.shoot(decision.cmd, aimer_, targets, gimbal_ypr);
    decision.valid_target = decision.cmd.gimbal_cmd_.valid;

    if (debug)
    {
        debug->detected_armors = std::move(armors);
        debug->targets = std::move(targets);
        debug->debug_aim_point = aimer_.debug_aim_point;
    }
    return decision;
}

} // namespace armor_task
