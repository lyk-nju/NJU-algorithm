#include "ros2_manager.hpp"

ROS2Manager::ROS2Manager(bool subscribe_image) : Node("ros2_manager")
{
    if (subscribe_image)
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&ROS2Manager::image_callback, this, std::placeholders::_1));
    }

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::QoS(10),
        std::bind(&ROS2Manager::cmd_vel_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "ROS2Manager initialized (subscribe_image=%s).",
        subscribe_image ? "true" : "false");

    autoaim_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/autoaim_data", 10);
    autoaim_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ROS2Manager::timer_callback, this));
}

ROS2Manager::~ROS2Manager()
{
    stopSpin();
}

void ROS2Manager::startSpin()
{
    if (spinning_.exchange(true))
    {
        return;
    }

    spin_thread_ = std::thread([this]()
                               { rclcpp::spin(this->get_node_base_interface()); });
}

void ROS2Manager::stopSpin()
{
    if (!spinning_.exchange(false))
    {
        return;
    }

    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }
}

void ROS2Manager::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        const cv::Mat &frame = cv_ptr->image;

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
