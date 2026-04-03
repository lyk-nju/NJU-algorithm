#pragma once

#include "armor.hpp"
#include <cv_bridge/cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "serial_manager.hpp"

class ROS2Manager : public rclcpp::Node
{
  public:
    struct FramePacket
    {
        cv::Mat frame;
        cv_bridge::CvImageConstPtr image_ref;
        std::chrono::steady_clock::time_point timestamp;
        uint64_t id = 0;
    };

    ROS2Manager();

    // 从图像订阅中获取当前图像帧（深拷贝）和时间戳
    bool get_img(cv::Mat &img);
    bool get_img_with_timestamp(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp);
    bool get_frame_packet(std::shared_ptr<const FramePacket> &packet);
    bool get_cmd_vel(geometry_msgs::msg::Twist &twist);
    void update_aimer_data(const io::AimerData &data);
    std::mutex &get_aimer_mutex() { return aimer_data_mutex_; }


  private:
    int save_count_ = 0; // 记录已保存的图像数量
    const int max_save_count_ = 10; // 最大保存数量
    const std::string save_path_ = "/home/nvidia/NJU-algorithm/vision_task/hiki_ros2/image/";
    // 图像订阅回调
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timer_callback();

    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr autoaim_pub_;
    rclcpp::TimerBase::SharedPtr autoaim_timer_;
    
    std::mutex img_mutex_;
    std::shared_ptr<FramePacket> latest_frame_packet_;
    uint64_t current_frame_id_ = 0;
    uint64_t consumed_frame_id_ = 0;

    std::mutex cmd_vel_mutex_;
    geometry_msgs::msg::Twist latest_cmd_vel_;
    bool has_latest_cmd_vel_ = false;

    std::mutex aimer_data_mutex_;
    io::AimerData latest_aimer_data_;
    bool has_aimer_data_ = false;
};
