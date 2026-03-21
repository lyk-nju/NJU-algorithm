#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "decision/judge_parser.hpp"
#include "decision/robot_decision.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace decision
{

class SimpleDecisionNode : public rclcpp::Node
{
  public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    SimpleDecisionNode() : Node("decision_node")
    {

        std::string nav_ns = this->declare_parameter<std::string>("nav_namespace", "");
        if (!nav_ns.empty() && nav_ns.front() != '/')
        {
            nav_ns = "/" + nav_ns;
        }
        const std::string nav_action = nav_ns + "/navigate_through_poses";
        const std::string nav_feedback = nav_action + "/_action/feedback";
        const std::string nav_status = nav_action + "/_action/status";

        // 订阅自瞄 + judge 数据（Decision 不再向 Vision 发送自瞄相关话题）
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        autoaim_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/autoaim_data", qos, std::bind(&SimpleDecisionNode::autoAimCallback, this, std::placeholders::_1));

        // Nav2 action client
        nav_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, nav_action);

        // 订阅 Nav2 反馈与状态（与拓扑、TUP 参考一致）
        nav_feedback_sub_ = this->create_subscription<NavigateThroughPoses::Impl::FeedbackMessage>(
            nav_feedback, qos,
            std::bind(&SimpleDecisionNode::nav2FeedbackCallback, this, std::placeholders::_1));
        nav_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            nav_status, qos,
            std::bind(&SimpleDecisionNode::nav2GoalStatusCallback, this, std::placeholders::_1));

        // 简化：立即等待一次 action server（失败也不退出，仅打印）
        if (!nav_client_->wait_for_action_server(5s))
        {
            RCLCPP_WARN(this->get_logger(), "NavigateThroughPoses action server not available yet.");
        }

        // 初始化决策核心参数与路径（支持 config.json 或 ROS 参数，与 ref 一致）
        float distance_thr = 1.0f;
        float seek_thr = 5.0f;
        float real_width = 12.0f;
        float real_height = 8.0f;
        std::string map_path = this->declare_parameter<std::string>("map_path", "RMUL.png");
        float step_distance = 0.1f;
        float car_seek_fov = 70.0f;
        std::string waypoints_path = this->declare_parameter<std::string>("waypoints_path", "waypoints.json");
        std::string decisions_path = this->declare_parameter<std::string>("decisions_path", "decisions.json");
        resume_cooldown_sec_ = this->declare_parameter<float>("resume_cooldown_sec", 0.5f);
        home_waypoint_id_ = this->declare_parameter<int>("home_waypoint_id", 1);
        recovery_hp_threshold_ = this->declare_parameter<int>("recovery_hp_threshold", 80);

        // 初始位置参数（当没有Nav2反馈时使用）
        this->declare_parameter<float>("initial_pose.x", 1.5f);
        this->declare_parameter<float>("initial_pose.y", 1.0f);
        this->declare_parameter<float>("initial_pose.theta", 0.0f);
        initial_x_ = this->get_parameter("initial_pose.x").as_double();
        initial_y_ = this->get_parameter("initial_pose.y").as_double();
        initial_theta_ = this->get_parameter("initial_pose.theta").as_double();

        std::string config_file = this->declare_parameter<std::string>("config_file", "");
        if (!config_file.empty())
        {
            std::ifstream f(config_file);
            if (f.good())
            {
                try
                {
                    nlohmann::json j;
                    f >> j;
                    f.close();
                    if (j.contains("config"))
                    {
                        const auto& c = j["config"];
                        std::string base_dir = config_file.substr(0, config_file.find_last_of("/\\") + 1);
                        if (c.contains("WayPointsPATH"))
                            waypoints_path = base_dir + c["WayPointsPATH"].get<std::string>();
                        if (c.contains("DecisionsPATH"))
                            decisions_path = base_dir + c["DecisionsPATH"].get<std::string>();
                        if (c.contains("MAP_PATH"))
                            map_path = base_dir + c["MAP_PATH"].get<std::string>();
                        if (c.contains("REAL_WIDTH"))
                            real_width = c["REAL_WIDTH"].get<float>();
                        if (c.contains("REAL_HEIGHT"))
                            real_height = c["REAL_HEIGHT"].get<float>();
                        if (c.contains("INIT_DISTANCE_THR"))
                            distance_thr = c["INIT_DISTANCE_THR"].get<float>();
                        if (c.contains("INIT_SEEK_THR"))
                            seek_thr = c["INIT_SEEK_THR"].get<float>();
                        if (c.contains("STEP_DISTANCE"))
                            step_distance = c["STEP_DISTANCE"].get<float>();
                        if (c.contains("CAR_SEEK_FOV"))
                            car_seek_fov = c["CAR_SEEK_FOV"].get<float>();
                        RCLCPP_INFO(this->get_logger(), "Loaded config from %s", config_file.c_str());
                    }
                }
                catch (const std::exception& e)
                {
                    RCLCPP_WARN(this->get_logger(), "config_file parse failed: %s, using ROS params", e.what());
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "config_file not found: %s, using ROS params", config_file.c_str());
            }
        }

        rdsys_ = std::make_shared<rdsys::RobotDecisionSys>(
            distance_thr, seek_thr, real_width, real_height, map_path, step_distance, car_seek_fov);

        if (!rdsys_->decodeWayPoints(waypoints_path))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to decode waypoints from %s", waypoints_path.c_str());
        }
        if (!rdsys_->decodeDecisions(decisions_path))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to decode decisions from %s", decisions_path.c_str());
        }

        // 与 ref 一致：由数据回调驱动决策，不再使用定时器
        RCLCPP_INFO(this->get_logger(), "SimpleDecisionNode initialized (callback-driven).");
    }

  private:
    // === ROS 接口 ===
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr autoaim_sub_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
    rclcpp::Subscription<NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr nav_feedback_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;

    // === 决策核心 ===
    std::shared_ptr<rdsys::RobotDecisionSys> rdsys_;

    // 最近一次自瞄 + judge 数据
    std::mutex data_mutex_;
    std_msgs::msg::Float32MultiArray latest_autoaim_msg_;

    // Navigation → Decision：Nav2 反馈与状态（与拓扑一致）
    std::mutex nav_feedback_mutex_;
    std::mutex nav_status_mutex_;
    NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr latest_nav_feedback_;
    int8_t goal_status_{action_msgs::msg::GoalStatus::STATUS_UNKNOWN};
    int last_sent_target_waypoint_{-1};
    int last_known_waypoint_id_{-1};
    int saved_target_waypoint_{-1};
    bool saved_if_succession_{false};
    bool has_saved_goal_{false};
    rclcpp::Time enemy_lost_time_;
    float resume_cooldown_sec_{0.5f};
    int home_waypoint_id_{1};
    int recovery_hp_threshold_{80};
    bool in_recovery_mode_{false};

    // 初始位置（当没有Nav2反馈时使用）
    float initial_x_{1.5f};
    float initial_y_{1.0f};
    float initial_theta_{0.0f};
    bool use_initial_pose_{true};

    void autoAimCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_autoaim_msg_ = *msg;
        }

        RCLCPP_INFO(this->get_logger(), 
            "[AutoAim] data[0]=%.2f (has_enemy), data[1]=%.2f (game_time), data[2]=%.2f (self_hp), size=%zu",
            msg->data[0], msg->data[1], msg->data[2], msg->data.size());

        bool has_enemy = (msg->data[0] != 0.0f);
        if (!has_enemy && has_saved_goal_)
        {
            rclcpp::Duration elapsed = this->now() - enemy_lost_time_;
            if (elapsed.seconds() >= resume_cooldown_sec_)
            {
                RCLCPP_INFO(this->get_logger(), "[Resume] Enemy lost, resuming saved goal after cooldown");
                resumeSavedGoal();
            }
        }

        runDecisionOnce(*msg);
    }

    void nav2FeedbackCallback(const NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(nav_feedback_mutex_);
        latest_nav_feedback_ = msg;
    }

    void nav2GoalStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        if (msg->status_list.empty())
            return;
        std::lock_guard<std::mutex> lock(nav_status_mutex_);
        goal_status_ = msg->status_list.back().status;
    }

    void runDecisionOnce(const std_msgs::msg::Float32MultiArray& autoaim)
    {
        if (autoaim.data.size() < 3)
        {
            return;
        }

        bool vision_valid = (autoaim.data[0] != 0.0f);
        if (vision_valid)
        {
            if (last_sent_target_waypoint_ >= 0)
            {
                saved_target_waypoint_ = last_sent_target_waypoint_;
                saved_if_succession_ = (goal_status_ == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) ? false : true;
                has_saved_goal_ = true;
                enemy_lost_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), "[Cancel] Enemy detected, saving goal (target=%d, if_succ=%d)",
                    saved_target_waypoint_, saved_if_succession_);
            }
            cancelNavGoal();
        }
        io::AimResult judge_raw;
        judge_raw.has_enemy = vision_valid;
        judge_raw.game_time = static_cast<int>(autoaim.data[1]);
        judge_raw.self_hp = static_cast<int>(autoaim.data[2]);

        ParsedJudgeInfo info;
        parse_judger_data(judge_raw, info);
        bool has_judge = (info.game_time > 0 || info.self_hp > 0);
        float self_x = 0.0f;
        float self_y = 0.0f;
        bool using_initial_pose = false;
        {
            std::lock_guard<std::mutex> lock(nav_feedback_mutex_);
            if (latest_nav_feedback_)
            {
                self_x = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.x);
                self_y = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.y);
                info.self_x = self_x;
                info.self_y = self_y;
            }
            else if (use_initial_pose_)
            {
                self_x = initial_x_;
                self_y = initial_y_;
                using_initial_pose = true;
                info.self_x = self_x;
                info.self_y = self_y;
                RCLCPP_INFO(this->get_logger(), "[Decision] Using initial pose: x=%.2f, y=%.2f", self_x, self_y);
            }
        }
        if (!has_judge)
        {
            info.game_time = 0;
            info.self_hp = 0;
        }

        int wayPointID = rdsys_->checkNowWayPoint(self_x, self_y);
        if (wayPointID < 0 && !using_initial_pose)
        {
            return;
        }
        if (wayPointID < 0 && using_initial_pose)
        {
            wayPointID = 1;
        }

        RCLCPP_INFO(this->get_logger(), 
            "[Decision] vision=%d, self_x=%.2f, self_y=%.2f, hp=%d, waypoint=%d, recovery=%d",
            vision_valid, self_x, self_y, info.self_hp, wayPointID, in_recovery_mode_);
        last_known_waypoint_id_ = wayPointID;

        int robot_mode = 0;
        int hp = info.self_hp;
        int nowtime = info.game_time;

        bool at_home = (wayPointID == home_waypoint_id_);
        if (at_home)
        {
            in_recovery_mode_ = true;
        }
        else if (hp > recovery_hp_threshold_)
        {
            in_recovery_mode_ = false;
        }

        std::vector<rdsys::RobotPosition> friend_pos = info.friend_positions;
        std::vector<rdsys::RobotPosition> enemy_pos = info.enemy_positions;
        std::vector<int> availableDecisionID;
        std::map<int, int> id_pos_f;
        std::map<int, int> id_pos_e;

        auto decision = rdsys_->decide(
            wayPointID, robot_mode, hp, nowtime,
            friend_pos, enemy_pos, availableDecisionID, id_pos_f, id_pos_e);

        if (decision)
        {
            if (in_recovery_mode_ && hp >= 40 && hp <= recovery_hp_threshold_)
            {
                if (decision->name != "return_home_when_low_hp")
                {
                    RCLCPP_INFO(this->get_logger(), 
                        "[Recovery] In recovery mode (hp=%d, at_home=%d), blocking combat decision: %s",
                        hp, at_home, decision->name.c_str());
                    decision = nullptr;
                }
            }
        }

        if (decision)
        {
            RCLCPP_INFO(this->get_logger(), 
                "[Decision] Selected: name=%s, target_wp=%d, if_succession=%d, weight=%d",
                decision->name.c_str(), decision->decide_wayPoint, 
                decision->if_succession, decision->weight);
            sendNavGoal(wayPointID, decision->decide_wayPoint, decision->if_succession);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[Decision] No decision matched!");
        }
    }

    // 函数功能：发送导航目标
    void sendNavGoal(int current_waypoint, int target_waypoint, bool if_succession)
    {
        (void)current_waypoint;
        if (!nav_client_)
        {
            return;
        }

        // 若当前目标仍在执行或已到达且目标路径点未变，不重复发送
        {
            std::lock_guard<std::mutex> lock(nav_status_mutex_);
            const bool same_target = (last_sent_target_waypoint_ == target_waypoint);
            const bool still_running = (goal_status_ == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                                        goal_status_ == action_msgs::msg::GoalStatus::STATUS_EXECUTING);
            const bool already_succeeded = (goal_status_ == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
            if (same_target && (still_running || already_succeeded))
            {
                return;
            }
        }

        auto goal_msg = NavigateThroughPoses::Goal();
        std::vector<std::shared_ptr<rdsys::WayPoint>> waypoints;

        if (!if_succession)
        {
            auto wp = rdsys_->getWayPointByID(target_waypoint);
            if (!wp)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "sendNavGoal: invalid target_waypoint %d", target_waypoint);
                return;
            }
            waypoints.push_back(wp);
        }
        else
        {
            waypoints = rdsys_->calculatePath(current_waypoint, target_waypoint);
        }

        if (waypoints.empty())
        {
            return;
        }

        // 过滤掉路径中的空指针（防御性，防止配置不一致）
        waypoints.erase(
            std::remove_if(waypoints.begin(), waypoints.end(), [](const std::shared_ptr<rdsys::WayPoint>& p) { return !p; }),
            waypoints.end());
        if (waypoints.empty())
        {
            return;
        }

        rclcpp::Time now = this->now();
        for (auto &wp : waypoints)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = now;
            pose.header.frame_id = "map";
            pose.pose.position.x = wp->x;
            pose.pose.position.y = wp->y;
            pose.pose.position.z = 0.0;

            // 简化：仅使用路径点朝向
            double yaw = wp->theta;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            pose.pose.orientation = tf2::toMsg(q);

            goal_msg.poses.push_back(pose);
        }

        last_sent_target_waypoint_ = target_waypoint;

        auto send_goal_options = typename rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void cancelNavGoal()
    {
        if (!nav_client_ || !nav_client_->action_server_is_ready())
        {
            return;
        }
        nav_client_->async_cancel_all_goals();
        last_sent_target_waypoint_ = -1;
    }

    void resumeSavedGoal()
    {
        if (!has_saved_goal_ || saved_target_waypoint_ < 0)
        {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "[Resume] Resuming goal: target=%d, if_succession=%d",
            saved_target_waypoint_, saved_if_succession_);
        sendNavGoal(last_known_waypoint_id_, saved_target_waypoint_, saved_if_succession_);
        has_saved_goal_ = false;
    }
};

} // namespace decision

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<decision::SimpleDecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

