#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "decision/robot_decision.hpp"

using namespace std::chrono_literals;

namespace patrol
{

class PatrolNode : public rclcpp::Node
{
public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    PatrolNode()
        : Node("patrol_node")
    {
        declare_parameter<std::string>("nav_namespace", "");
        declare_parameter<std::string>("waypoints_path", "waypoints_test.json");
        declare_parameter<std::string>("decisions_path", "decisions_test.json");
        declare_parameter<std::string>("map_path", "RMUL.png");
        declare_parameter<float>("distance_thr", 1.0f);
        declare_parameter<float>("seek_thr", 5.0f);
        declare_parameter<float>("real_width", 12.0f);
        declare_parameter<float>("real_height", 8.0f);
        declare_parameter<float>("step_distance", 0.1f);
        declare_parameter<float>("car_seek_fov", 70.0f);
        declare_parameter<bool>("use_pure_patrol", true);

        std::string nav_ns = get_parameter("nav_namespace").as_string();
        if (!nav_ns.empty() && nav_ns.front() != '/')
        {
            nav_ns = "/" + nav_ns;
        }

        const std::string nav_action = nav_ns + "/navigate_through_poses";
        const std::string nav_feedback = nav_action + "/_action/feedback";
        const std::string nav_status = nav_action + "/_action/status";

        RCLCPP_INFO(this->get_logger(), "Connecting to Nav2 action: %s", nav_action.c_str());

        nav_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, nav_action);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        nav_feedback_sub_ = this->create_subscription<NavigateThroughPoses::Impl::FeedbackMessage>(
            nav_feedback, qos,
            std::bind(&PatrolNode::navFeedbackCallback, this, std::placeholders::_1));
        nav_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            nav_status, qos,
            std::bind(&PatrolNode::navStatusCallback, this, std::placeholders::_1));

        if (!nav_client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "NavigateThroughPoses action server not available!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully connected to Nav2 action server!");

        std::string package_share = "/home/nvidia/NJU-algorithm/decision_task";
        std::string waypoints_path = package_share + "/config/" + get_parameter("waypoints_path").as_string();
        std::string decisions_path = package_share + "/config/" + get_parameter("decisions_path").as_string();
        std::string map_path = package_share + "/config/" + get_parameter("map_path").as_string();

        float distance_thr = get_parameter("distance_thr").as_double();
        float seek_thr = get_parameter("seek_thr").as_double();
        float real_width = get_parameter("real_width").as_double();
        float real_height = get_parameter("real_height").as_double();
        float step_distance = get_parameter("step_distance").as_double();
        float car_seek_fov = get_parameter("car_seek_fov").as_double();

        rdsys_ = std::make_shared<rdsys::RobotDecisionSys>(
            distance_thr, seek_thr, real_width, real_height, map_path, step_distance, car_seek_fov);

        if (!rdsys_->decodeWayPoints(waypoints_path))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode waypoints from %s", waypoints_path.c_str());
            return;
        }

        if (!rdsys_->decodeDecisions(decisions_path))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode decisions from %s", decisions_path.c_str());
            return;
        }

        current_waypoint_id_ = -1;
        last_sent_target_ = -1;
        goal_status_ = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
        has_goal_succeeded_ = false;

        RCLCPP_INFO(this->get_logger(), "PatrolNode initialized with %zu waypoints", rdsys_->wayPointMap.size());
        RCLCPP_INFO(this->get_logger(), "Starting pure patrol mode...");

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.5),
            std::bind(&PatrolNode::checkAndSendGoal, this));
    }

private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
    rclcpp::Subscription<NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr nav_feedback_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<rdsys::RobotDecisionSys> rdsys_;

    int current_waypoint_id_;
    int last_sent_target_;
    int8_t goal_status_;
    bool has_goal_succeeded_;

    float current_x_;
    float current_y_;

    std::mutex status_mutex_;

    void navFeedbackCallback(const NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_x_ = msg->feedback.current_pose.pose.position.x;
        current_y_ = msg->feedback.current_pose.pose.position.y;
    }

    void navStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (!msg->status_list.empty())
        {
            int8_t prev_status = goal_status_;
            goal_status_ = msg->status_list.back().status;

            if (goal_status_ == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED && 
                prev_status != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Goal reached! Current position: x=%.2f, y=%.2f", current_x_, current_y_);
                has_goal_succeeded_ = true;
            }
        }
    }

    void checkAndSendGoal()
    {
        std::lock_guard<std::mutex> lock(status_mutex_);

        if (!nav_client_)
        {
            return;
        }

        bool is_goal_running = (goal_status_ == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                               goal_status_ == action_msgs::msg::GoalStatus::STATUS_EXECUTING);

        if (is_goal_running)
        {
            return;
        }

        if (has_goal_succeeded_)
        {
            int reached_waypoint = rdsys_->checkNowWayPoint(current_x_, current_y_);
            if (reached_waypoint >= 0)
            {
                current_waypoint_id_ = reached_waypoint;
                RCLCPP_INFO(this->get_logger(), "Robot at waypoint %d, finding next target...", current_waypoint_id_);
            }
            has_goal_succeeded_ = false;
        }

        int next_target = findNextTarget(current_waypoint_id_);
        if (next_target < 0)
        {
            RCLCPP_WARN(this->get_logger(), "No decision found for current waypoint %d", current_waypoint_id_);
            if (current_waypoint_id_ < 0)
            {
                next_target = 0;
                RCLCPP_INFO(this->get_logger(), "Starting from beginning, target waypoint %d", next_target);
            }
            else
            {
                return;
            }
        }

        if (next_target == last_sent_target_)
        {
            return;
        }

        sendNavGoal(next_target);
    }

    int findNextTarget(int current_wp_id)
    {
        for (const auto& dec : rdsys_->decisions)
        {
            if (!dec->wayPointID.empty() && dec->wayPointID[0] == current_wp_id)
            {
                RCLCPP_INFO(this->get_logger(), "Found decision: %s -> waypoint %d", 
                           dec->name.c_str(), dec->decide_wayPoint);
                return dec->decide_wayPoint;
            }
        }
        return -1;
    }

    void sendNavGoal(int target_waypoint_id)
    {
        if (!nav_client_)
        {
            return;
        }

        auto target_wp = rdsys_->getWayPointByID(target_waypoint_id);
        if (!target_wp)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid target waypoint id: %d", target_waypoint_id);
            return;
        }

        auto goal_msg = NavigateThroughPoses::Goal();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = target_wp->x;
        pose.pose.position.y = target_wp->y;
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, target_wp->theta);
        pose.pose.orientation = tf2::toMsg(q);

        goal_msg.poses.push_back(pose);

        RCLCPP_INFO(this->get_logger(), "Sending goal to waypoint %d: x=%.2f, y=%.2f, theta=%.2f",
                    target_waypoint_id, target_wp->x, target_wp->y, target_wp->theta);

        auto send_goal_options = typename rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();

        send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Navigation goal aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Navigation goal canceled");
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Navigation result unknown");
                break;
            }
        };

        nav_client_->async_send_goal(goal_msg, send_goal_options);
        last_sent_target_ = target_waypoint_id;
    }
};

}  // namespace patrol

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<patrol::PatrolNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
