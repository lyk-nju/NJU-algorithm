/*
fake_publish.cpp:
测试目标：
    测试 vision 模块是否能正常订阅 /cmd_vel

测试内容：
    1. 启动一个 ROS2 节点
    2. 周期性向 /cmd_vel 发布 Twist 信息
    3. 在终端打印当前发布的数据，方便和 auto_aimer_test 联调
*/

#include <chrono>
#include <atomic>
#include <cstdlib>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <memory>
#include <thread>

#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>



class FakeCmdVelPublisher : public rclcpp::Node
{
  public:
    FakeCmdVelPublisher(double linear_x, double linear_y, double angular_z, double publish_hz)
        : Node("fake_cmd_vel_publisher"),
          linear_x_(linear_x),
          linear_y_(linear_y),
          angular_z_(angular_z),
          keep_moving_(false),
          keyboard_running_(true)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));

        auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / publish_hz));
        if (period.count() <= 0)
        {
            period = std::chrono::milliseconds(1);
        }
        timer_ = this->create_wall_timer(period, std::bind(&FakeCmdVelPublisher::publish_cmd, this));
        keyboard_thread_ = std::thread(&FakeCmdVelPublisher::keyboard_loop, this);

        RCLCPP_INFO(this->get_logger(), "Start publishing /cmd_vel at %.2f Hz", publish_hz);
        RCLCPP_INFO(this->get_logger(), "Configured command: linear.x=%.3f linear.y=%.3f angular.z=%.3f", linear_x_, linear_y_, angular_z_);
        RCLCPP_INFO(this->get_logger(), "Press 'a' to publish motion, press 'q' to publish zero Twist");
    }

    ~FakeCmdVelPublisher() override
    {
        keyboard_running_.store(false);
        if (keyboard_thread_.joinable())
        {
            keyboard_thread_.join();
        }
    }

  private:
    void publish_cmd()
    {
        geometry_msgs::msg::Twist msg;

        if (keep_moving_.load())
        {
            msg.linear.x = linear_x_;
            msg.linear.y = linear_y_;
            msg.angular.z = angular_z_;
        }

        publisher_->publish(msg);
        ++publish_count_;

        if (publish_count_ % 20 == 0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[%zu] /cmd_vel state=%s linear.x=%.3f linear.y=%.3f angular.z=%.3f",
                publish_count_,
                keep_moving_.load() ? "ACTIVE" : "STOPPED",
                msg.linear.x,
                msg.linear.y,
                msg.angular.z);
        }
    }

    void keyboard_loop()
    {
        if (!isatty(STDIN_FILENO))
        {
            RCLCPP_WARN(this->get_logger(), "stdin is not a terminal; keyboard control disabled");
            return;
        }

        termios old_termios {};
        if (tcgetattr(STDIN_FILENO, &old_termios) != 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to read terminal attributes; keyboard control disabled");
            return;
        }

        termios raw_termios = old_termios;
        raw_termios.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
        raw_termios.c_cc[VMIN] = 0;
        raw_termios.c_cc[VTIME] = 1;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw_termios) != 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to switch terminal to raw mode; keyboard control disabled");
            return;
        }

        while (keyboard_running_.load() && rclcpp::ok())
        {
            char key = 0;
            const auto bytes_read = read(STDIN_FILENO, &key, 1);
            if (bytes_read <= 0)
            {
                continue;
            }

            if (key == 'a')
            {
                keep_moving_.store(true);
                RCLCPP_WARN(this->get_logger(), "Keyboard command: ACTIVE, publishing configured /cmd_vel");
            }
            else if (key == 'q')
            {
                keep_moving_.store(false);
                RCLCPP_WARN(this->get_logger(), "Keyboard command: STOPPED, publishing zero /cmd_vel");
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread keyboard_thread_;
    double linear_x_;
    double linear_y_;
    double angular_z_;
    std::atomic<bool> keep_moving_;
    std::atomic<bool> keyboard_running_;
    std::size_t publish_count_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.05;
    double publish_hz = 20.0;

    if (argc > 1) linear_x = std::atof(argv[1]);
    if (argc > 2) linear_y = std::atof(argv[2]);
    if (argc > 3) angular_z = std::atof(argv[3]);
    if (argc > 4) publish_hz = std::atof(argv[4]);

    if (publish_hz <= 0.0)
    {
        std::cerr << "publish_hz must be positive" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    std::cout << "Usage: " << argv[0] << " [linear_x] [linear_y] [angular_z] [publish_hz]" << std::endl;
    std::cout << "Example: " << argv[0] << " 1.0 0.3 0.2 20" << std::endl;
    std::cout << "Press 'a' to start motion publishing, press 'q' to stop." << std::endl;

    auto node = std::make_shared<FakeCmdVelPublisher>(linear_x, linear_y, angular_z, publish_hz);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
