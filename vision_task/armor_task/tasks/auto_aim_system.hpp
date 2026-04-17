#pragma once

#include "../io/algorithm/ros2_transfer.hpp"
#include "../io/driver/cboard/cboard.hpp"
#include "../io/driver/camera/direct_image_source.hpp"
#include "../io/driver/ros2/ros2_image_source.hpp"
#include "../io/driver/ros2/ros2_manager.hpp"
#include "../io/interface/i_frame_source.hpp"
#include "../io/structs/structs.hpp"
#include "aim_pipeline.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace armor_task
{

/// 图像输入模式
enum class ImageSourceType
{
    ROS2_TOPIC,    ///< 订阅 /image_raw 话题
    DIRECT_CAMERA, ///< USB 直连相机（io::Camera）
};

/**
 * @brief 自瞄应用外壳（Step A 阶段的"薄壳"）。
 *
 * 持有 ROS2Manager / IFrameSource / Cboard 三类 I/O 资源和跨帧业务状态，
 * 把纯算法流水线委托给 AimPipeline 执行。
 *
 * 图像与 IMU 的时间对齐：
 *   - IFrameSource 负责提供“曝光时刻”的 steady_clock 时间戳；
 *   - Cboard 内部维护带时间戳的四元数环形缓冲，
 *     cboard_->gimbal_quat_at(t) 返回 t 时刻的 slerp 插值姿态；
 *   - run() 每帧以图像时间戳向 Cboard 查询对齐后的 quat，转成 FrameBundle
 *     后交给 AimPipeline::step。
 *
 * Step B 计划：把 run() 的取图/时间对齐抽成 FrameScheduler；
 *              本类改名为 AutoAimApp 并只做生命周期装配。
 */
class AutoAimSystem
{
  public:
    AutoAimSystem(
        const std::string &yolo_model_path,
        const std::string &config_path,
        double bullet_speed,
        ImageSourceType source_type);
    ~AutoAimSystem();

    /// 启动整套系统（创建并管理 ROS2Manager / IFrameSource / Cboard 及其后台线程）
    void start();
    /// 阻塞运行主循环（取图 -> 处理 -> 发布 -> 发送），直到 rclcpp::ok()==false
    void run();
    /// 停止后台线程（析构也会调用）
    void stop();

    /// 根据下位机发送的裁判系统数据更新颜色/弹速等跨帧状态
    void updateData(const io::JudgerData &judger_data);

    /**
     * @brief 处理一帧图像（Step A 薄壳：委托给 pipeline_->step）。
     * @param img 图像（BGR）。
     * @param quat 对齐到 image_timestamp 的云台四元数（由 cboard_->gimbal_quat_at 查询得到）。
     * @param image_timestamp 图像时间戳（steady_clock 域）。
     */
    io::Vision2Cboard processFrame(
        const cv::Mat &img,
        const Eigen::Quaterniond &quat,
        std::chrono::steady_clock::time_point image_timestamp);

    // 外部（可视化/日志）曾经依赖的 getter，转发到 pipeline_。
    bool usingTensorRt() const { return pipeline_.usingTensorRt(); }
    const char *backendName() const { return pipeline_.backendName(); }
    const PnpSolver &getPnpSolver() const { return pipeline_.pnp_solver(); }

  private:
    // 配置
    std::string yolo_model_path_;
    std::string config_path_;
    ImageSourceType source_type_;

    // 算法流水线（无 I/O，无线程）
    AimPipeline pipeline_;

    // 当前相机参数（单相机场景：构造后从 pipeline_.pnp_solver() 拷贝一份，
    // 生命周期 = AutoAimSystem，供每帧 FrameBundle 引用）。
    // Step C 后改由 FrameScheduler 持有 camera_id -> CameraInfo 的表。
    io::CameraInfo camera_info_;

    // 跨帧业务状态（单线程访问：由 run() 主循环更新和使用）
    double bullet_speed_;
    bool enemy_is_red_ = true;

    // I/O
    std::shared_ptr<ROS2Manager> ros_node_;
    std::unique_ptr<io::IFrameSource> image_source_;
    std::unique_ptr<io::Cboard> cboard_owner_;
};

} // namespace armor_task
