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

    SimpleDecisionNode() : Node("simple_decision_node")
    {
        // 读取导航命名空间，默认适配 navigation_task2 的 red_standard_robot1
        std::string nav_ns = this->declare_parameter<std::string>("nav_namespace", "red_standard_robot1");
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
            "/auto_aim/result", qos, std::bind(&SimpleDecisionNode::autoAimCallback, this, std::placeholders::_1));

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
    bool has_latest_autoaim_{false};
    std_msgs::msg::Float32MultiArray latest_autoaim_msg_;

    // Navigation → Decision：Nav2 反馈与状态（与拓扑一致）
    std::mutex nav_feedback_mutex_;
    std::mutex nav_status_mutex_;
    NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr latest_nav_feedback_;
    int8_t goal_status_{action_msgs::msg::GoalStatus::STATUS_UNKNOWN};
    int last_sent_target_waypoint_{-1};  // 避免在目标未变且仍在执行时重复发目标

    // 简单状态机
    enum class State
    {
        PATROL,
        AUTO_AIM
    };
    State state_{State::PATROL};

    void autoAimCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_autoaim_msg_ = *msg;
            has_latest_autoaim_ = true;
        }
        // 与 ref 一致：每次收到自瞄+裁判数据就做一次决策并输出
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

    // 函数功能：用当前自瞄+裁判消息执行一次决策（由 /auto_aim/result 回调驱动，与 ref 的同步回调思路一致）
    void runDecisionOnce(const std_msgs::msg::Float32MultiArray& autoaim)
    {
        // 拆出自瞄和 judge 部分
        if (autoaim.data.size() < 3)
        {
            return;
        }

        bool vision_valid = (autoaim.data[0] != 0.0f);
        float vision_yaw = autoaim.data[1];
        float vision_pitch = autoaim.data[2];

        // 裁判数据当前仅两项：data[3]=game_time, data[4]=self_hp
        io::JudgerData judge_raw;
        if (autoaim.data.size() >= 5)
        {
            judge_raw.game_time = static_cast<int>(autoaim.data[3]);
            judge_raw.self_hp = static_cast<int>(autoaim.data[4]);
        }

        ParsedJudgeInfo info;
        float self_x = 0.0f;
        float self_y = 0.0f;
        bool has_judge = parse_judger_data(judge_raw, info);
        // 位置优先用 Nav2 feedback（当前协议无 self_x/self_y）
        {
            std::lock_guard<std::mutex> lock(nav_feedback_mutex_);
            if (latest_nav_feedback_)
            {
                self_x = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.x);
                self_y = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.y);
                info.self_x = self_x;
                info.self_y = self_y;
            }
        }
        if (!has_judge)
        {
            info.game_time = 0;
            info.self_hp = 0;
        }

        // 当前协议无位置，必须依赖 Nav2 feedback 才有路径点
        if (!latest_nav_feedback_)
        {
            simpleStateMachine(vision_valid, vision_yaw, vision_pitch);
            return;
        }

        int wayPointID = rdsys_->checkNowWayPoint(self_x, self_y);
        if (wayPointID < 0)
        {
            simpleStateMachine(vision_valid, vision_yaw, vision_pitch);
            return;
        }

        int robot_mode = 0; // 0 表示常规模式
        int hp = info.self_hp;
        int nowtime = info.game_time;

        std::vector<rdsys::RobotPosition> friend_pos = info.friend_positions;
        std::vector<rdsys::RobotPosition> enemy_pos = info.enemy_positions;
        std::vector<int> availableDecisionID;
        std::map<int, int> id_pos_f;
        std::map<int, int> id_pos_e;

        auto decision = rdsys_->decide(
            wayPointID, robot_mode, hp, nowtime,
            friend_pos, enemy_pos, availableDecisionID, id_pos_f, id_pos_e);

        if (!decision)
        {
            simpleStateMachine(vision_valid, vision_yaw, vision_pitch);
            return;
        }

        // 根据决策模式切换状态
        if (decision->decide_mode == 8)
        {
            state_ = State::AUTO_AIM;
        }
        else
        {
            state_ = State::PATROL;
        }

        // 若需要移动，则按照决策路径点发送导航目标
        if (state_ == State::PATROL)
        {
            sendNavGoal(wayPointID, decision->decide_wayPoint, decision->if_succession);
        }
    }

    // 函数功能：退化版的简单状态机：只看 vision_valid（仅用于内部状态与导航，不向 Vision 发话题）
    void simpleStateMachine(bool vision_valid, float /* vision_yaw */, float /* vision_pitch */)
    {
        if (vision_valid)
        {
            state_ = State::AUTO_AIM;
        }
        else
        {
            state_ = State::PATROL;
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

