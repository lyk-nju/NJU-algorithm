#pragma once

#include "../io/dataframe/struct.hpp"
#include "../io/dataframe/transfer.hpp"
#include "../io/camera/camera_base.hpp"
#include "../io/ros2/ros2_manager.hpp"
#include "../io/usb/cboard.hpp"
#include "aimer.hpp"
#include "detector.hpp"
#include "imu_history.hpp"
#include "pnp_solver.hpp"
#include "shooter.hpp"
#include "target.hpp"
#include "tracker.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace armor_task
{

/// processFrame 的返回结果（供调用方用于显示、串口发送等）
struct ProcessResult
{
    io::Vision2Cboard cmd;
};

/**
 * @brief 自瞄算法统一入口（纯算法，无 ROS/串口/键盘依赖）
 *
 * 封装 Detector、PnpSolver、Tracker、Aimer 及 IMU 同步逻辑，
 * 供 auto_aimer（独立部署）和 nju_autoaim_node（TUP 集成）共用。
 */
class AutoAimSystem
{
  public:
    AutoAimSystem(const std::string &yolo_model_path, const std::string &config_path, double bullet_speed);
    ~AutoAimSystem();

    /// 启动整套系统（创建并管理 ROS2Manager/Cboard/后台线程）
    void start();
    /// 阻塞运行主循环（取图->处理->发布->发送），直到 rclcpp::ok()==false
    void run();
    /// 停止后台线程（析构也会调用）
    void stop();
    /// 是否使用直连相机输入（camera.read），默认 false（使用 ROS2 /image_raw）
    void enableDirectCameraInput(bool enable = true) { use_direct_camera_input_ = enable; }

    /// 根据下位机发送的数据更新
    void updateData(const Eigen::Quaterniond &quat, const io::JudgerData &judger_data);

    /// 处理一帧图像，返回瞄准控制量与中间结果
    ProcessResult processFrame(const cv::Mat &img, std::chrono::steady_clock::time_point image_timestamp);

    bool usingTensorRt() const { return detector_.usingTensorRt(); }
    const char *backendName() const { return detector_.backendName(); }

    /// 获取 PnP 解算器（用于弹道绘制等，如 drawTrajectory 需要 R_gimbal2world_）
    const PnpSolver &getPnpSolver() const { return pnp_solver_; }

  private:
    std::string yolo_model_path_;
    std::string config_path_;

    Detector detector_;
    PnpSolver pnp_solver_;
    Tracker tracker_;
    Aimer aimer_;
    Shooter shooter_;
    IMUHistory imu_history_;
    std::atomic<double> bullet_speed_;
    Eigen::Quaterniond last_quat_; /// 最近一次成功查询的 IMU 四元数，用于 query 失败时的降级
    std::atomic<bool> enemy_is_red_{true}; /// true=敌方红色，false=敌方蓝色

    // I/O（系统负责）
    std::shared_ptr<ROS2Manager> ros_node_;

    std::unique_ptr<io::Camera> camera_owner_;
    std::unique_ptr<io::Cboard> cboard_owner_;
    bool use_direct_camera_input_ = false;
};

} // namespace armor_task
