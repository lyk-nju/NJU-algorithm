#include "auto_aim_system.hpp"

#include "../io/driver/camera/direct_image_source.hpp"
#include "../io/driver/ros2/ros2_image_source.hpp"

#include <thread>

namespace armor_task
{
namespace
{
constexpr float kIdleYawRate = 0.03f;

// 组装发送给下位机的数据帧：填充 base_cmd（来自 /cmd_vel），失锁时让云台缓慢自旋
io::Vision2Cboard composeSendCommand(const io::Vision2Cboard &cmd, ROS2Manager &ros_node)
{
    io::Vision2Cboard to_send = cmd;
    geometry_msgs::msg::Twist twist;
    if (ros_node.get_cmd_vel(twist))
    {
        to_send.base_cmd_ = io::transfer::from_cmd_vel(twist);
    }
    else
    {
        to_send.base_cmd_ = io::base_command{};
    }

    if (!to_send.gimbal_cmd_.valid)
    {
        to_send.base_cmd_.w_yaw = kIdleYawRate;
    }
    return to_send;
}
} // namespace

AutoAimSystem::AutoAimSystem(
    const std::string &yolo_model_path,
    const std::string &config_path,
    double bullet_speed,
    ImageSourceType source_type)
    : yolo_model_path_(yolo_model_path),
      config_path_(config_path),
      source_type_(source_type),
      pipeline_(yolo_model_path, config_path),
      // 从 Pipeline 内部的 PnpSolver 把 yaml 读入的那组参数 copy 出来，
      // 确保 FrameBundle.camera 指向的值和 PnpSolver 构造期的值逐字节一致——
      // 这是 Step A"零数值差异"的关键锚点。
      camera_info_(pipeline_.pnp_solver().as_camera_info(/*camera_id=*/0, "default")),
      bullet_speed_(bullet_speed)
{
}

AutoAimSystem::~AutoAimSystem()
{
    stop();
}

void AutoAimSystem::start()
{
    stop();

    const bool need_image_sub = (source_type_ == ImageSourceType::ROS2_TOPIC);
    ros_node_ = std::make_shared<ROS2Manager>(need_image_sub);
    ros_node_->startSpin();

    if (source_type_ == ImageSourceType::DIRECT_CAMERA)
    {
        image_source_ = std::make_unique<io::DirectImageSource>(config_path_);
    }
    else
    {
        image_source_ = std::make_unique<io::Ros2ImageSource>(ros_node_);
    }

    cboard_owner_ = std::make_unique<io::Cboard>(config_path_);
}

void AutoAimSystem::run()
{
    if (!ros_node_ || !cboard_owner_ || !image_source_)
    {
        start();
    }

    while (rclcpp::ok())
    {
        cv::Mat image;
        std::chrono::steady_clock::time_point image_timestamp;
        if (!image_source_->read(image, image_timestamp))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // IMU 与图像时间对齐：用图像时间戳向 Cboard 查询 slerp 插值后的云台姿态
        const Eigen::Quaterniond quat = cboard_owner_->gimbal_quat_at(image_timestamp);
        const io::JudgerData jd = cboard_owner_->judger();

        updateData(jd);

        const io::Vision2Cboard cmd = processFrame(image, quat, image_timestamp);

        ros_node_->update_aimer_data(io::transfer::from_vis_dec(cmd.gimbal_cmd_.valid, jd));
        cboard_owner_->send(composeSendCommand(cmd, *ros_node_));
    }
}

void AutoAimSystem::stop()
{
    if (ros_node_)
    {
        ros_node_->stopSpin();
        ros_node_.reset();
    }
    image_source_.reset();
    cboard_owner_.reset();
}

void AutoAimSystem::updateData(const io::JudgerData &judger_data)
{
    // self_id == 0 表示下位机尚未把真实机器人 ID 上报上来，
    // 此时沿用上一次的敌方颜色（默认 enemy_is_red_ = true），
    // 避免上电瞬间把所有真正的敌方装甲板过滤掉。
    if (judger_data.self_id != 0)
    {
        enemy_is_red_ = !io::transfer::self_is_red_from_id(judger_data);
    }
    if (judger_data.bullet_speed > 0.1)
    {
        bullet_speed_ = judger_data.bullet_speed;
    }
}

io::Vision2Cboard AutoAimSystem::processFrame(
    const cv::Mat &img,
    const Eigen::Quaterniond &quat,
    std::chrono::steady_clock::time_point image_timestamp)
{
    FrameBundle bundle;
    bundle.frame.image = img;
    bundle.frame.timestamp = image_timestamp;
    // frame.id / frame.camera_id 保持默认 0（单相机场景）
    bundle.camera = &camera_info_;
    bundle.gimbal_quat = quat;

    const GameState gs{enemy_is_red_, bullet_speed_};
    return pipeline_.step(bundle, gs).cmd;
}

} // namespace armor_task
