#include "ros2_manager.hpp"
#include <filesystem>

ROS2Manager::ROS2Manager() : Node("ros2_manager")
{
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ROS2Manager::image_callback, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::QoS(10),
        std::bind(&ROS2Manager::cmd_vel_callback, this, std::placeholders::_1));

    // armor_pub_ = this->create_publisher<armor_interfaces::msg::ArmorArray>("/armor/prediction", 10);

    RCLCPP_INFO(this->get_logger(), "ROS2Manager initialized.");

    autoaim_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/autoaim_data", 10);
    autoaim_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ROS2Manager::timer_callback, this));
    if (!std::filesystem::exists(save_path_)) 
    {
        std::filesystem::create_directories(save_path_);
        RCLCPP_INFO(this->get_logger(), "Created directory: %s", save_path_.c_str());
    }
}

void ROS2Manager::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        const cv::Mat &frame = cv_ptr->image;

        if (save_count_ < max_save_count_) 
        {
            std::string file_name = save_path_ + "img_" + std::to_string(save_count_) + ".jpg";
            if (cv::imwrite(file_name, frame)) {
                RCLCPP_INFO(this->get_logger(), "Saved image: %s", file_name.c_str());
                save_count_++;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save image: %s", file_name.c_str());
            }
        }

        // 将 ROS 时间戳映射到 steady_clock（用于与 IMUHistory 对齐）。
        // 注意：ROS 时间可能是 system time / ROS time / sim time，无法保证可映射。
        // 这里使用“当前 system_clock 与 msg stamp 的差值”估算，并对异常差值做降级。
        const auto now_system = std::chrono::system_clock::now();
        const auto now_steady = std::chrono::steady_clock::now();
        const auto ros_time = rclcpp::Time(msg->header.stamp);

        const double ros_time_sec = ros_time.seconds();
        const double now_system_sec =
            std::chrono::duration_cast<std::chrono::duration<double>>(now_system.time_since_epoch()).count();
        const double diff_sec = ros_time_sec - now_system_sec;

        std::chrono::steady_clock::time_point frame_timestamp = now_steady;
        if (std::abs(diff_sec) <= 1.0)
        {
            const auto diff = std::chrono::duration<double>(diff_sec);
            frame_timestamp = now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(diff);
        }

        auto packet = std::make_shared<FramePacket>();
        packet->image_ref = cv_ptr;
        packet->frame = frame;
        packet->timestamp = frame_timestamp;

        std::lock_guard<std::mutex> lock(img_mutex_);
        packet->id = ++current_frame_id_;
        latest_frame_packet_ = std::move(packet);
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
}

bool ROS2Manager::get_img(cv::Mat &img)
{
    std::lock_guard<std::mutex> lock(img_mutex_);
    if (!latest_frame_packet_ || consumed_frame_id_ == latest_frame_packet_->id) return false;

    img = latest_frame_packet_->frame.clone();
    consumed_frame_id_ = latest_frame_packet_->id;
    return true;
}

bool ROS2Manager::get_img_with_timestamp(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp)
{
    std::lock_guard<std::mutex> lock(img_mutex_);
    if (!latest_frame_packet_ || consumed_frame_id_ == latest_frame_packet_->id) return false;

    img = latest_frame_packet_->frame.clone();
    timestamp = latest_frame_packet_->timestamp;
    consumed_frame_id_ = latest_frame_packet_->id;
    return true;
}

bool ROS2Manager::get_frame_packet(std::shared_ptr<const ROS2Manager::FramePacket> &packet)
{
    std::lock_guard<std::mutex> lock(img_mutex_);
    if (!latest_frame_packet_ || consumed_frame_id_ == latest_frame_packet_->id) return false;

    packet = latest_frame_packet_;
    consumed_frame_id_ = latest_frame_packet_->id;
    return true;
}

bool ROS2Manager::get_cmd_vel(geometry_msgs::msg::Twist &twist)
{
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    if (!has_latest_cmd_vel_) return false;

    twist = latest_cmd_vel_;
    return true;
}

void ROS2Manager::timer_callback()
{
    std::lock_guard<std::mutex> lock(aimer_data_mutex_);
    if (!has_aimer_data_ || !autoaim_pub_)
    {
        return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(3);
    msg.data.push_back(latest_aimer_data_.cmd_valid ? 1.0f : 0.0f);
    msg.data.push_back(static_cast<float>(latest_aimer_data_.game_time));
    msg.data.push_back(static_cast<float>(latest_aimer_data_.self_hp));
    autoaim_pub_->publish(msg);
}

void ROS2Manager::update_aimer_data(const io::AimerData &data)
{
    std::lock_guard<std::mutex> lock(aimer_data_mutex_);
    latest_aimer_data_ = data;
    has_aimer_data_ = true;
}
