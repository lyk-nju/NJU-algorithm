#include "auto_aim_system.hpp"
#include "../tools/math_tools.hpp"

namespace armor_task
{
namespace
{
constexpr float kIdleYawRate = 0.03f;

// 获取图像时间匹配的IMU四元数
Eigen::Quaterniond getQuat(
    IMUHistory &imu_history,
    const std::chrono::steady_clock::time_point &image_timestamp,
    Eigen::Quaterniond &last_quat)
{
    Eigen::Quaterniond synced_quat;
    double synced_yaw = 0.0;
    double synced_pitch = 0.0;
    if (imu_history.query(image_timestamp, synced_quat, synced_yaw, synced_pitch))
    {
        last_quat = synced_quat;
        return synced_quat;
    }
    if (imu_history.size() > 0)
    {
        return last_quat;
    }
    return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
}

// 组装发送给下位机的数据帧
io::Vision2Cboard composeSendCommand(const ProcessResult &result, ROS2Manager &ros_node)
{
    io::Vision2Cboard to_send = result.cmd;
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

AutoAimSystem::AutoAimSystem(const std::string &yolo_model_path, const std::string &config_path, double bullet_speed)
    : yolo_model_path_(yolo_model_path),
      config_path_(config_path),
      detector_(yolo_model_path),
      pnp_solver_(config_path),
      tracker_(config_path, pnp_solver_),
      aimer_(config_path),
      shooter_(config_path),
      bullet_speed_(bullet_speed),
      last_quat_(1.0, 0.0, 0.0, 0.0)
{
}

AutoAimSystem::~AutoAimSystem()
{
    stop();
}

void AutoAimSystem::start()
{
    stop();

    ros_node_ = std::make_shared<ROS2Manager>();
    ros_node_->startSpin();

    if (use_direct_camera_input_)
    {
        camera_owner_ = std::make_unique<io::Camera>(config_path_);
    }
    cboard_owner_ = std::make_unique<io::Cboard>(config_path_);
}

void AutoAimSystem::run()
{
    if (!ros_node_ || !cboard_owner_)
    {
        start();
    }

    while (rclcpp::ok())
    {
        cv::Mat image;
        std::chrono::steady_clock::time_point image_timestamp;
        if (use_direct_camera_input_)
        {
            camera_owner_->read(image, image_timestamp);
            if (image.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
        }
        else
        {
            std::shared_ptr<const ROS2Manager::FramePacket> packet;
            if (!ros_node_->get_frame_packet(packet) || !packet || packet->frame.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            image = packet->frame;
            image_timestamp = packet->timestamp;
        }

        const Eigen::Quaterniond quat = cboard_owner_->q(image_timestamp);
        const io::JudgerData jd = cboard_owner_->judger();

        updateData(quat, jd);

        ProcessResult result = processFrame(image, image_timestamp);

        const bool cmd_valid = result.cmd.gimbal_cmd_.valid;
        ros_node_->update_aimer_data(io::transfer::from_vis_dec(cmd_valid, jd));

        cboard_owner_->send(composeSendCommand(result, *ros_node_));
    }
}

void AutoAimSystem::stop()
{
    if (ros_node_)
    {
        ros_node_->stopSpin();
        ros_node_.reset();
    }
    camera_owner_.reset();
    cboard_owner_.reset();
}

void AutoAimSystem::updateData(const Eigen::Quaterniond &quat, const io::JudgerData &judger_data)
{
    imu_history_.push(quat, 0.0, 0.0);
    const bool self_is_red = io::transfer::self_is_red_from_id(judger_data);
    enemy_is_red_.store(!self_is_red, std::memory_order_relaxed);
    if (judger_data.bullet_speed > 0.1)
    {
        bullet_speed_.store(judger_data.bullet_speed, std::memory_order_relaxed);
    }
}

ProcessResult AutoAimSystem::processFrame(const cv::Mat &img, std::chrono::steady_clock::time_point image_timestamp)
{
    ProcessResult result{};

    const Eigen::Quaterniond curr_quat = getQuat(imu_history_, image_timestamp, last_quat_);
    pnp_solver_.set_R_gimbal2world(curr_quat);
    ArmorArray armors = detector_.detect(img);

    std::vector<Target> targets = tracker_.track(armors, image_timestamp, enemy_is_red_.load(std::memory_order_relaxed));

    if (targets.empty())
    {
        result.cmd.gimbal_cmd_.valid = false;
        result.cmd.gimbal_cmd_.shoot = false;
        result.cmd.gimbal_cmd_.yaw = 0.0f;
        result.cmd.gimbal_cmd_.pitch = 0.0f;
        return result;
    }

    const auto track_end = std::chrono::steady_clock::now();
    std::list<Target> target_list(targets.begin(), targets.end());
    result.cmd = aimer_.aim(target_list, track_end, bullet_speed_.load(std::memory_order_relaxed));

    const Eigen::Vector3d gimbal_ypr = tools::eulers(pnp_solver_.R_gimbal2world_, 2, 1, 0);
    result.cmd.gimbal_cmd_.shoot = shooter_.shoot(result.cmd, aimer_, targets, gimbal_ypr);

    return result;
}

} // namespace armor_task
