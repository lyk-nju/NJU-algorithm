#pragma once

// 自瞄视觉流水线：detect -> solve_pnp(in tracker) -> track -> aim -> shoot。
//
// 设计约束（Step A 金线）：
//   - 本类不访问任何 I/O：无 cboard、无 image_source、无 rclcpp、无 cv_bridge；
//   - 本类不起线程；
//   - 本类持有算法组件的跨帧状态（Tracker 的 target_/state_machine 等），
//     但不持有任何跨帧业务状态（enemy_is_red / bullet_speed / judger）——
//     那些由上层按帧通过 GameState 注入。
//
// 用一句话：同一帧 input，任何时刻调用都得到同一 output，除非它内部的
// 算法状态（tracker EKF 等）因为上一帧的调用而改变。

#include "aimer.hpp"
#include "detector.hpp"
#include "pipeline_types.hpp"
#include "pnp_solver.hpp"
#include "shooter.hpp"
#include "tracker.hpp"

#include <string>

namespace armor_task
{

class AimPipeline
{
  public:
    AimPipeline(const std::string &yolo_model_path, const std::string &algo_config_path);

    /// 生产路径：不填 DebugSnapshot，生产路径每帧 0 拷贝调试数据。
    AimDecision step(const FrameBundle &bundle, const GameState &gs);

    /// 调试路径：显式要求一份 DebugSnapshot，由调用方提供 buffer（便于复用）。
    AimDecision step(const FrameBundle &bundle, const GameState &gs, DebugSnapshot &debug);

    bool usingTensorRt() const { return detector_.usingTensorRt(); }
    const char *backendName() const { return detector_.backendName(); }

    /// 可视化/调试用。绝大多数算法外部访问需求都落在只读 getter 上。
    const Detector &detector() const { return detector_; }
    const PnpSolver &pnp_solver() const { return pnp_solver_; }
    const Aimer &aimer() const { return aimer_; }
    const Tracker &tracker() const { return tracker_; }

  private:
    /// 统一实现入口：debug 为 nullptr 时不做任何调试字段填充。
    AimDecision step_impl(const FrameBundle &bundle, const GameState &gs, DebugSnapshot *debug);

    // 成员声明顺序即初始化顺序：pnp_solver_ 必须早于 tracker_，
    // 因为 Tracker 构造时要拿 PnpSolver& 引用。
    Detector detector_;
    PnpSolver pnp_solver_;
    Tracker tracker_;
    Aimer aimer_;
    Shooter shooter_;
};

} // namespace armor_task
