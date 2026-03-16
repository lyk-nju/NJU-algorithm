#pragma once

#include "armor.hpp"
#include "serial_manager.hpp"
#include <atomic>
#include <cv_bridge/cv_bridge.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ROS2Manager : public rclcpp::Node
{
  public:
    ROS2Manager();

    bool get_img(cv::Mat &img);
    bool get_img_with_timestamp(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp);
    void publish_vision_result(const io::Command &cmd, const io::JudgerData &judger_data);
    bool get_cmd_vel(geometry_msgs::msg::Twist &twist);

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr autoaim_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    std::mutex img_mutex_;
    cv::Mat current_frame_;
    std::chrono::steady_clock::time_point current_frame_timestamp_;
    std::atomic<bool> new_frame_ready_;

    // 最近一次从导航模块接收到的 /cmd_vel
    std::mutex cmd_vel_mutex_;
    geometry_msgs::msg::Twist latest_cmd_vel_;
    std::atomic<bool> has_latest_cmd_vel_{false};
};