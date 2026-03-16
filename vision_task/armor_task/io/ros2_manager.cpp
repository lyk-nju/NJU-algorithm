#include "ros2_manager.hpp"
#include "aimer.hpp"
#include "detector.hpp"
#include "pnp_solver.hpp"
#include "tracker.hpp"

ROS2Manager::ROS2Manager() : Node("ros2_manager"), new_frame_ready_(false)
{
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10, std::bind(&ROS2Manager::image_callback, this, std::placeholders::_1));

    autoaim_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/auto_aim/result", 10);

    // 从导航模块订阅 /cmd_vel（线速度 + 角速度）
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&ROS2Manager::cmd_vel_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ROS2Manager initialized.");
}

void ROS2Manager::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // 转换ROS时间戳为steady_clock时间点
        // ROS使用的是系统时间（system_clock），我们需要转换为steady_clock
        auto ros_time = rclcpp::Time(msg->header.stamp);
        auto now_system = std::chrono::system_clock::now();
        auto now_steady = std::chrono::steady_clock::now();

        // 计算ROS时间与当前系统时间的差值
        auto ros_time_sec = ros_time.seconds();
        auto now_system_sec = std::chrono::duration_cast<std::chrono::duration<double>>(now_system.time_since_epoch()).count();
        auto time_diff = std::chrono::duration<double>(ros_time_sec - now_system_sec);

        // 将时间差应用到steady_clock
        auto frame_timestamp = now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(time_diff);

        std::lock_guard<std::mutex> lock(img_mutex_);
        current_frame_ = frame.clone();
        current_frame_timestamp_ = frame_timestamp;
        new_frame_ready_ = true;
        // RCLCPP_INFO(this->get_logger(), "Image Received.");
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    }
}

void ROS2Manager::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    latest_cmd_vel_ = *msg;
    has_latest_cmd_vel_ = true;
    // 暂时仅缓存，不在本节点内直接使用
}

bool ROS2Manager::get_cmd_vel(geometry_msgs::msg::Twist &twist)
{
    if (!has_latest_cmd_vel_) return false;

    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    twist = latest_cmd_vel_;
    return true;
}

bool ROS2Manager::get_img(cv::Mat &img)
{
    if (!new_frame_ready_) return false;

    std::lock_guard<std::mutex> lock(img_mutex_);
    img = current_frame_.clone();
    new_frame_ready_ = false;
    return true;
}

bool ROS2Manager::get_img_with_timestamp(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp)
{
    if (!new_frame_ready_) return false;

    std::lock_guard<std::mutex> lock(img_mutex_);
    img = current_frame_.clone();
    timestamp = current_frame_timestamp_;
    new_frame_ready_ = false;
    return true;
}

void ROS2Manager::publish_vision_result(const io::Command &cmd, const io::JudgerData &judger_data)
{
    if (!autoaim_pub_)
    {
        return;
    }

    std_msgs::msg::Float32MultiArray msg;
    // 自瞄 3 项 + 裁判 2 项（game_time, self_hp）
    msg.data.reserve(5);
    msg.data.push_back(cmd.valid ? 1.0f : 0.0f);
    msg.data.push_back(cmd.yaw);
    msg.data.push_back(cmd.pitch);
    msg.data.push_back(static_cast<float>(judger_data.game_time));
    msg.data.push_back(static_cast<float>(judger_data.self_hp));
    autoaim_pub_->publish(msg);
}
