#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace pose_init
{

class PoseInitializerNode : public rclcpp::Node
{
public:
    PoseInitializerNode()
        : Node("pose_initializer_node")
    {
        declare_parameter<std::string>("nav_namespace", "");
        declare_parameter<double>("initial_pose.x", 1.0);
        declare_parameter<double>("initial_pose.y", -1.0);
        declare_parameter<double>("initial_pose.theta", 0.0);
        declare_parameter<double>("delay", 1.0);
        declare_parameter<int>("publish_count", 1);
        declare_parameter<double>("publish_interval", 0.5);

        std::string nav_ns = get_parameter("nav_namespace").as_string();
        if (!nav_ns.empty() && nav_ns.front() != '/')
        {
            nav_ns = "/" + nav_ns;
        }

        std::string initial_pose_topic = nav_ns + "/initialpose";

        RCLCPP_INFO(this->get_logger(), "Publishing initial pose to: %s", initial_pose_topic.c_str());

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic, rclcpp::QoS(rclcpp::KeepLast(10)));

        double delay = get_parameter("delay").as_double();
        publish_count_ = std::max(1, static_cast<int>(get_parameter("publish_count").as_int()));
        publish_interval_ = get_parameter("publish_interval").as_double();
        if (publish_interval_ <= 0.0)
        {
            publish_interval_ = 0.5;
        }

        auto timer_callback = [this, delay]() {
            start_timer_->cancel();
            publishInitialPose();

            ++published_count_;
            if (published_count_ >= publish_count_)
            {
                has_published_ = true;
                return;
            }

            publish_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(publish_interval_),
                [this]() {
                    if (published_count_ >= publish_count_)
                    {
                        publish_timer_->cancel();
                        has_published_ = true;
                        return;
                    }

                    publishInitialPose();
                    ++published_count_;

                    if (published_count_ >= publish_count_)
                    {
                        publish_timer_->cancel();
                        has_published_ = true;
                    }
                });
        };

        start_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(delay),
            timer_callback);

        RCLCPP_INFO(this->get_logger(),
            "Initial pose will be published %d times after %.2fs delay (interval=%.2fs)",
            publish_count_, delay, publish_interval_);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr start_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    bool has_published_ = false;
    int publish_count_ = 1;
    int published_count_ = 0;
    double publish_interval_ = 0.5;

    void publishInitialPose()
    {
        double x = get_parameter("initial_pose.x").as_double();
        double y = get_parameter("initial_pose.y").as_double();
        double theta = get_parameter("initial_pose.theta").as_double();

        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        pose_msg.header.frame_id = "map";

        pose_msg.pose.pose.position.x = x;
        pose_msg.pose.pose.position.y = y;
        pose_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta);
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();

        pose_msg.pose.covariance[0] = 0.25;
        pose_msg.pose.covariance[7] = 0.25;
        pose_msg.pose.covariance[35] = 0.06853891909122467;

        pose_pub_->publish(pose_msg);

        RCLCPP_INFO(this->get_logger(), "Published initial pose [%d/%d]: x=%.2f, y=%.2f, theta=%.2f",
                    published_count_ + 1, publish_count_, x, y, theta);
    }
};

}  // namespace pose_init

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pose_init::PoseInitializerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
