/*
camera_sub.cpp：
测试目标：
    查看相机图像是否能通过ros正常接受

测试内容：
    查看从ros2_topic接受的/image_raw话题是否能够正常显示，并将接受图像帧率保存至实验日志camera.log中
*/

#include "../../io/ros2_manager.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::cout << "=== Camera Subscriber Test ===" << std::endl;
    std::cout << "Note: Make sure rm_camera node is running in another terminal:" << std::endl;
    std::cout << "  ros2 run rm_camera rm_camera_pub" << std::endl;

    // 打开日志文件
    std::ofstream log_file("../test/deploy/log/camera.log", std::ios::app);
    if (!log_file.is_open())
    {
        std::cerr << "Warning: Cannot open log file, logging to console only" << std::endl;
    }

    // 创建 ROS2Manager 节点（封装了图像订阅功能）
    auto ros_node = std::make_shared<ROS2Manager>();
    std::thread spin_thread([&]() { rclcpp::spin(ros_node); });

    // 初始化帧率统计
    int frame_count = 0;
    auto last_time = std::chrono::steady_clock::now();


    while (rclcpp::ok())
    {
        cv::Mat img;
        // 使用 ROS2Manager 的 get_img 方法获取图像
        if (ros_node->get_img(img))
        {
            if (img.empty())
            {
                continue;
            }

            frame_count++;
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(current_time - last_time).count();

            // 每秒更新一次帧率显示和日志
            if (elapsed >= 1.0)
            {
                double fps = frame_count / elapsed;

                // 在图像上显示帧率
                cv::Mat display_frame = img.clone();
                std::string fps_text = "FPS: " + std::to_string(fps).substr(0, 5);
                cv::putText(display_frame, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

                // 显示分辨率
                std::string res_text = "Resolution: " + std::to_string(img.cols) + "x" + std::to_string(img.rows);
                cv::putText(display_frame, res_text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                // 显示图像
                cv::imshow("Camera Subscriber Test", display_frame);

                // 记录到日志文件
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");

                std::string log_entry = "[" + ss.str() + "] FPS: " + std::to_string(fps).substr(0, 5) + ", Resolution: " + std::to_string(img.cols) + "x" + std::to_string(img.rows) + "\n";

                if (log_file.is_open())
                {
                    log_file << log_entry;
                    log_file.flush();
                }

                std::cout << log_entry;

                // 重置统计
                frame_count = 0;
                last_time = current_time;
            }
            else
            {
                // 即使不更新日志，也要显示图像
                cv::imshow("Camera Subscriber Test", img);
            }

            // 处理键盘输入
            char key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27)
            { // 'q' 或 ESC
                break;
            }
        }
        else
        {
            // 如果没有新图像，短暂休眠
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    if (log_file.is_open())
    {
        log_file.close();
    }

    rclcpp::shutdown();
    spin_thread.join();
    cv::destroyAllWindows();

    std::cout << "\nCamera subscriber test finished." << std::endl;
    return 0;
}
