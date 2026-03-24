#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "decision/judge_parser.hpp"
#include "decision/robot_decision.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
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
        const std::string initial_pose_topic = nav_ns + "/initialpose";
        const std::string amcl_pose_topic = nav_ns + "/amcl_pose";

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
        amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            amcl_pose_topic, qos,
            std::bind(&SimpleDecisionNode::amclPoseCallback, this, std::placeholders::_1));
        initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic, rclcpp::QoS(rclcpp::KeepLast(10)));

        // 简化：立即等待一次 action server（失败也不退出，仅打印）
        if (!nav_client_->wait_for_action_server(5s))
        {
            RCLCPP_WARN(this->get_logger(), "NavigateThroughPoses action server not available yet.");
        }

        // 初始化决策核心参数与路径（支持 config.json 或 ROS 参数，与 ref 一致）
        float distance_thr = this->declare_parameter<float>("distance_thr", 1.0f);
        float seek_thr = this->declare_parameter<float>("seek_thr", 5.0f);
        float real_width = this->declare_parameter<float>("real_width", 12.0f);
        float real_height = this->declare_parameter<float>("real_height", 8.0f);
        std::string map_path = this->declare_parameter<std::string>("map_path", "RMUL.png");
        float step_distance = this->declare_parameter<float>("step_distance", 0.1f);
        float car_seek_fov = this->declare_parameter<float>("car_seek_fov", 70.0f);
        std::string waypoints_path = this->declare_parameter<std::string>("waypoints_path", "waypoints.json");
        std::string decisions_path = this->declare_parameter<std::string>("decisions_path", "decisions.json");
        resume_cooldown_sec_ = this->declare_parameter<float>("resume_cooldown_sec", 0.5f);
        home_waypoint_id_ = this->declare_parameter<int>("home_waypoint_id", 1);
        center_waypoint_id_ = this->declare_parameter<int>("center_waypoint_id", 5);
        recovery_hp_threshold_ = this->declare_parameter<int>("recovery_hp_threshold", 80);
        low_hp_return_threshold_ = this->declare_parameter<int>("low_hp_return_threshold", recovery_hp_threshold_);
        require_amcl_ready_before_nav_ = this->declare_parameter<bool>("require_amcl_ready_before_nav", true);
        amcl_pose_timeout_sec_ = this->declare_parameter<double>("amcl_pose_timeout_sec", 1.5);
        amcl_pose_min_samples_ = this->declare_parameter<int>("amcl_pose_min_samples", 3);
        aborted_reinit_threshold_ = this->declare_parameter<int>("aborted_reinit_threshold", 3);
        aborted_reinit_cooldown_sec_ = this->declare_parameter<double>("aborted_reinit_cooldown_sec", 2.0);
        last_good_pose_max_age_sec_ = this->declare_parameter<double>("last_good_pose_max_age_sec", 8.0);
        enable_aborted_reinit_ = this->declare_parameter<bool>("enable_aborted_reinit", true);

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

        // 创建位置自检定时器，每 1 秒执行一次
        position_check_threshold_ = this->declare_parameter<float>("position_check_threshold", 1.0f);
        position_check_activation_radius_ = this->declare_parameter<float>(
            "position_check_activation_radius", std::max(3.0f, position_check_threshold_ * 3.0f));
        if (position_check_activation_radius_ <= 0.0f)
        {
            position_check_activation_radius_ = std::max(3.0f, position_check_threshold_ * 3.0f);
            RCLCPP_WARN(this->get_logger(),
                "Invalid position_check_activation_radius, fallback to %.2f",
                position_check_activation_radius_);
        }
        position_check_timer_ = this->create_wall_timer(
            1s, std::bind(&SimpleDecisionNode::positionCheckTimerCallback, this));
        RCLCPP_INFO(this->get_logger(),
            "Position check timer created (threshold=%.2f, activation_radius=%.2f).",
            position_check_threshold_, position_check_activation_radius_);
        RCLCPP_INFO(this->get_logger(),
            "ABORTED reinit: enable=%d, threshold=%d, cooldown=%.2f, pose_max_age=%.2f",
            enable_aborted_reinit_, aborted_reinit_threshold_, aborted_reinit_cooldown_sec_, last_good_pose_max_age_sec_);
        RCLCPP_INFO(this->get_logger(),
            "AMCL gate: enable=%d, timeout=%.2fs, min_samples=%d",
            require_amcl_ready_before_nav_, amcl_pose_timeout_sec_, amcl_pose_min_samples_);
    }

  private:
    // === ROS 接口 ===
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr autoaim_sub_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
    rclcpp::Subscription<NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr nav_feedback_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;

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
    std::array<uint8_t, 16> active_goal_uuid_{};
    bool has_active_goal_uuid_{false};
    int last_sent_target_waypoint_{-1};
    int last_known_waypoint_id_{-1};
    int saved_target_waypoint_{-1};
    bool saved_if_succession_{false};
    bool has_saved_goal_{false};
    rclcpp::Time enemy_lost_time_;
    float resume_cooldown_sec_{0.5f};
    int home_waypoint_id_{1};
    int recovery_hp_threshold_{80};
    int low_hp_return_threshold_{80};
    bool in_recovery_mode_{false};
    int consecutive_aborted_count_{0};
    int aborted_reinit_threshold_{3};
    double aborted_reinit_cooldown_sec_{2.0};
    double last_good_pose_max_age_sec_{8.0};
    bool enable_aborted_reinit_{true};
    rclcpp::Time last_reinit_time_{0, 0, RCL_ROS_TIME};

    bool has_last_good_pose_{false};
    float last_good_x_{0.0f};
    float last_good_y_{0.0f};
    float last_good_theta_{0.0f};
    rclcpp::Time last_good_pose_time_{0, 0, RCL_ROS_TIME};

    std::mutex amcl_pose_mutex_;
    rclcpp::Time last_amcl_pose_time_{0, 0, RCL_ROS_TIME};
    int amcl_pose_samples_{0};
    bool require_amcl_ready_before_nav_{true};
    double amcl_pose_timeout_sec_{1.5};
    int amcl_pose_min_samples_{3};

    // 初始位置（当没有Nav2反馈时使用）
    float initial_x_{-1.5f};
    float initial_y_{-2.5f};
    float initial_theta_{0.0f};
    bool use_initial_pose_{true};

    // 位置自检定时器
    rclcpp::TimerBase::SharedPtr position_check_timer_;
    float position_check_threshold_{1.0f};
    float position_check_activation_radius_{3.0f};
    int center_waypoint_id_{5};

    bool isAmclReady()
    {
        if (!require_amcl_ready_before_nav_)
            return true;

        std::lock_guard<std::mutex> lock(amcl_pose_mutex_);
        if (amcl_pose_samples_ < amcl_pose_min_samples_)
            return false;

        const double dt = (this->now() - last_amcl_pose_time_).seconds();
        return dt >= 0.0 && dt <= amcl_pose_timeout_sec_;
    }

    void publishInitialPoseForRecovery(const char* reason)
    {
        double x = initial_x_;
        double y = initial_y_;
        double theta = initial_theta_;
        bool use_last_good = false;

        {
            std::lock_guard<std::mutex> lock(nav_feedback_mutex_);
            if (has_last_good_pose_)
            {
                const double age = (this->now() - last_good_pose_time_).seconds();
                if (age >= 0.0 && age <= last_good_pose_max_age_sec_)
                {
                    x = last_good_x_;
                    y = last_good_y_;
                    theta = last_good_theta_;
                    use_last_good = true;
                }
            }
        }

        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = x;
        pose_msg.pose.pose.position.y = y;
        pose_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta);
        pose_msg.pose.pose.orientation = tf2::toMsg(q);

        pose_msg.pose.covariance[0] = 0.25;
        pose_msg.pose.covariance[7] = 0.25;
        pose_msg.pose.covariance[35] = 0.06853891909122467;

        initialpose_pub_->publish(pose_msg);
        RCLCPP_WARN(this->get_logger(),
            "[Reinit] %s: publish /initialpose by %s (x=%.2f, y=%.2f, yaw=%.2f)",
            reason,
            use_last_good ? "last_good_pose" : "startup_initial_pose",
            x, y, theta);
    }

    bool isLowHp(int hp) const
    {
        return hp > 0 && hp <= low_hp_return_threshold_;
    }

    bool trySendReturnHomeGoal(const char* reason)
    {
        float current_x = 0.0f;
        float current_y = 0.0f;
        int current_waypoint_id = -1;
        {
            std::lock_guard<std::mutex> lock(nav_feedback_mutex_);
            if (latest_nav_feedback_)
            {
                current_x = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.x);
                current_y = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.y);
                current_waypoint_id = rdsys_->checkNowWayPoint(current_x, current_y);
            }
        }

        if (has_saved_goal_)
        {
            RCLCPP_INFO(this->get_logger(), "[LowHP] Clear saved goal before return home.");
            has_saved_goal_ = false;
            saved_target_waypoint_ = -1;
        }

        RCLCPP_WARN(this->get_logger(),
            "[LowHP] %s: hp<=%d, force return home wp=%d (current_wp=%d, x=%.2f, y=%.2f)",
            reason, low_hp_return_threshold_, home_waypoint_id_, current_waypoint_id, current_x, current_y);

        return sendNavGoal(current_waypoint_id, home_waypoint_id_, false);
    }

    void autoAimCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.empty())
        {
            RCLCPP_WARN(this->get_logger(), "[AutoAim] empty message, ignored.");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_autoaim_msg_ = *msg;
        }

        if (msg->data.size() >= 3)
        {
            const int hp = static_cast<int>(msg->data[2]);
            if (isLowHp(hp))
            {
                in_recovery_mode_ = true;
                (void)trySendReturnHomeGoal("AutoAimCallback");
                return;
            }
        }

        bool has_enemy = (msg->data[0] != 0.0f);
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[AutoAim] has_enemy=%d, has_saved_goal=%d, saved_target=%d, resume_cooldown=%.2f",
            has_enemy, has_saved_goal_, saved_target_waypoint_, resume_cooldown_sec_);

        if (has_saved_goal_)
        {
            RCLCPP_INFO(this->get_logger(), "[AutoAim] In saved goal mode, checking enemy status...");
            if (!has_enemy)
            {
                rclcpp::Duration elapsed = this->now() - enemy_lost_time_;
                RCLCPP_INFO(this->get_logger(), 
                    "[AutoAim] Enemy lost, elapsed=%.3f, cooldown=%.2f",
                    elapsed.seconds(), resume_cooldown_sec_);
                if (elapsed.seconds() >= resume_cooldown_sec_)
                {
                    RCLCPP_INFO(this->get_logger(), "[AutoAim] Cooldown reached, calling resumeSavedGoal!");
                    resumeSavedGoal();
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "[AutoAim] Still waiting for cooldown...");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "[AutoAim] Enemy still visible, updating position...");
                std::lock_guard<std::mutex> lock(nav_feedback_mutex_);
                if (latest_nav_feedback_)
                {
                    float cx = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.x);
                    float cy = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.y);
                    last_known_waypoint_id_ = rdsys_->checkNowWayPoint(cx, cy);
                    RCLCPP_INFO(this->get_logger(), 
                        "[AutoAim] Updated last_known_waypoint_id_ to %d (x=%.2f, y=%.2f)",
                        last_known_waypoint_id_, cx, cy);
                }
            }
        }
        else
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[AutoAim] No saved goal, running decision...");
            runDecisionOnce(*msg);
        }
    }

    void nav2FeedbackCallback(const NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(nav_feedback_mutex_);
        latest_nav_feedback_ = msg;

        const auto& p = msg->feedback.current_pose.pose.position;
        const auto& q = msg->feedback.current_pose.pose.orientation;
        if (std::isfinite(p.x) && std::isfinite(p.y) &&
            std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w))
        {
            has_last_good_pose_ = true;
            last_good_x_ = static_cast<float>(p.x);
            last_good_y_ = static_cast<float>(p.y);
            last_good_theta_ = static_cast<float>(tf2::getYaw(q));
            last_good_pose_time_ = this->now();
        }
    }

    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(amcl_pose_mutex_);
        last_amcl_pose_time_ = this->now();
        if (amcl_pose_samples_ < 1000000)
            ++amcl_pose_samples_;
        (void)msg;
    }

    void nav2GoalStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        if (msg->status_list.empty())
            return;

        std::lock_guard<std::mutex> lock(nav_status_mutex_);

        // 只跟踪当前活动goal_id，避免误读旧目标状态
        if (has_active_goal_uuid_)
        {
            const action_msgs::msg::GoalStatus* matched = nullptr;
            for (const auto& st : msg->status_list)
            {
                if (st.goal_info.goal_id.uuid == active_goal_uuid_)
                {
                    matched = &st;
                    break;
                }
            }

            if (!matched)
            {
                return;
            }

            int8_t old_status = goal_status_;
            goal_status_ = matched->status;
            RCLCPP_INFO(this->get_logger(),
                "[NavStatus] goal_status changed(by goal_id): %d -> %d (last_sent_target=%d, saved_target=%d, has_saved=%d)",
                old_status, goal_status_, last_sent_target_waypoint_, saved_target_waypoint_, has_saved_goal_);

            if (goal_status_ == action_msgs::msg::GoalStatus::STATUS_ABORTED &&
                old_status != action_msgs::msg::GoalStatus::STATUS_ABORTED)
            {
                consecutive_aborted_count_++;
                RCLCPP_WARN(this->get_logger(),
                    "[NavStatus] ABORTED count=%d/%d",
                    consecutive_aborted_count_, aborted_reinit_threshold_);

                if (enable_aborted_reinit_ && consecutive_aborted_count_ >= aborted_reinit_threshold_)
                {
                    const double dt = (this->now() - last_reinit_time_).seconds();
                    if (dt >= aborted_reinit_cooldown_sec_)
                    {
                        publishInitialPoseForRecovery("Consecutive ABORTED");
                        last_reinit_time_ = this->now();
                        consecutive_aborted_count_ = 0;
                    }
                }
            }
            else if (goal_status_ == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
                     goal_status_ == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                     goal_status_ == action_msgs::msg::GoalStatus::STATUS_ACCEPTED)
            {
                consecutive_aborted_count_ = 0;
            }

            // 终态后清空当前goal_id，等待下一次发送重新绑定
            if (goal_status_ == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
                goal_status_ == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
                goal_status_ == action_msgs::msg::GoalStatus::STATUS_CANCELED)
            {
                has_active_goal_uuid_ = false;
            }
            return;
        }

        // 兼容：尚未拿到活动goal_id时，退化为旧逻辑
        int8_t old_status = goal_status_;
        goal_status_ = msg->status_list.back().status;
        RCLCPP_INFO(this->get_logger(), 
            "[NavStatus] goal_status changed(fallback back): %d -> %d (last_sent_target=%d, saved_target=%d, has_saved=%d)",
            old_status, goal_status_, last_sent_target_waypoint_, saved_target_waypoint_, has_saved_goal_);

        if (goal_status_ == action_msgs::msg::GoalStatus::STATUS_ABORTED &&
            old_status != action_msgs::msg::GoalStatus::STATUS_ABORTED)
        {
            consecutive_aborted_count_++;
            RCLCPP_WARN(this->get_logger(),
                "[NavStatus] ABORTED count(fallback)=%d/%d",
                consecutive_aborted_count_, aborted_reinit_threshold_);
            if (enable_aborted_reinit_ && consecutive_aborted_count_ >= aborted_reinit_threshold_)
            {
                const double dt = (this->now() - last_reinit_time_).seconds();
                if (dt >= aborted_reinit_cooldown_sec_)
                {
                    publishInitialPoseForRecovery("Consecutive ABORTED (fallback)");
                    last_reinit_time_ = this->now();
                    consecutive_aborted_count_ = 0;
                }
            }
        }
        else if (goal_status_ == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
                 goal_status_ == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                 goal_status_ == action_msgs::msg::GoalStatus::STATUS_ACCEPTED)
        {
            consecutive_aborted_count_ = 0;
        }
    }

    void runDecisionOnce(const std_msgs::msg::Float32MultiArray& autoaim)
    {
        if (autoaim.data.size() < 3)
        {
            return;
        }

        bool vision_valid = (autoaim.data[0] != 0.0f);
        const int current_hp = static_cast<int>(autoaim.data[2]);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[RunDecision] vision_valid=%d, last_sent_target=%d, goal_status=%d",
            vision_valid, last_sent_target_waypoint_, goal_status_);

        if (isLowHp(current_hp))
        {
            in_recovery_mode_ = true;
            (void)trySendReturnHomeGoal("RunDecisionOnce");
            return;
        }
        
        if (vision_valid)
        {
            RCLCPP_INFO(this->get_logger(), "[RunDecision] Enemy detected, checking if should save goal...");
            if (last_sent_target_waypoint_ >= 0)
            {
                saved_target_waypoint_ = last_sent_target_waypoint_;
                saved_if_succession_ = (goal_status_ == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) ? false : true;
                has_saved_goal_ = true;
                enemy_lost_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), 
                    "[RunDecision] SAVED goal: target=%d, if_succ=%d, has_saved=%d",
                    saved_target_waypoint_, saved_if_succession_, has_saved_goal_);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), 
                    "[RunDecision] No goal to save (last_sent_target=%d), only canceling",
                    last_sent_target_waypoint_);
            }
            cancelNavGoal();
            return;
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
    bool sendNavGoal(int current_waypoint, int target_waypoint, bool if_succession, bool force_send = false)
    {
        RCLCPP_INFO(this->get_logger(), 
            "[SendNavGoal] ENTER: current_wp=%d, target_wp=%d, if_succession=%d, force_send=%d, has_saved_goal=%d",
            current_waypoint, target_waypoint, if_succession, force_send, has_saved_goal_);
        
        (void)current_waypoint;
        if (!nav_client_)
        {
            RCLCPP_WARN(this->get_logger(), "[SendNavGoal] nav_client_ is null!");
            return false;
        }

        if (!isAmclReady())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[SendNavGoal] AMCL not ready yet, skip send goal.");
            return false;
        }

        // 若当前目标仍在执行或已到达且目标路径点未变，不重复发送
        if (!force_send)
        {
            std::lock_guard<std::mutex> lock(nav_status_mutex_);
            const bool same_target = (last_sent_target_waypoint_ == target_waypoint);
            const bool still_running = (goal_status_ == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                                        goal_status_ == action_msgs::msg::GoalStatus::STATUS_EXECUTING);
            const bool already_succeeded = (goal_status_ == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
            
            RCLCPP_INFO(this->get_logger(), 
                "[SendNavGoal] Check: same_target=%d, still_running=%d, already_succeeded=%d, "
                "last_sent=%d, goal_status=%d",
                same_target, still_running, already_succeeded, last_sent_target_waypoint_, goal_status_);
            
            if (same_target && (still_running || already_succeeded))
            {
                RCLCPP_INFO(this->get_logger(), "[SendNavGoal] SKIP: same target and still running or succeeded");
                return false;
            }
        }

        auto goal_msg = NavigateThroughPoses::Goal();
        std::vector<std::shared_ptr<rdsys::WayPoint>> waypoints;

        if (!if_succession)
        {
            RCLCPP_INFO(this->get_logger(), "[SendNavGoal] Mode: single target (no succession)");
            auto wp = rdsys_->getWayPointByID(target_waypoint);
            if (!wp)
            {
                RCLCPP_WARN(this->get_logger(), "[SendNavGoal] ERROR: invalid target_waypoint %d", target_waypoint);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "[SendNavGoal] Target wp: id=%d, x=%.2f, y=%.2f, theta=%.2f",
                wp->id, wp->x, wp->y, wp->theta);
            waypoints.push_back(wp);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[SendNavGoal] Mode: calculate path from %d to %d", 
                current_waypoint, target_waypoint);
            waypoints = rdsys_->calculatePath(current_waypoint, target_waypoint);
            RCLCPP_INFO(this->get_logger(), "[SendNavGoal] Path calculated: %zu waypoints", waypoints.size());
        }

        if (waypoints.empty())
        {
            RCLCPP_WARN(this->get_logger(), "[SendNavGoal] ERROR: waypoints empty!");
            return false;
        }

        // 过滤掉路径中的空指针（防御性，防止配置不一致）
        waypoints.erase(
            std::remove_if(waypoints.begin(), waypoints.end(), [](const std::shared_ptr<rdsys::WayPoint>& p) { return !p; }),
            waypoints.end());
        if (waypoints.empty())
        {
            RCLCPP_WARN(this->get_logger(), "[SendNavGoal] ERROR: waypoints empty after filtering!");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "[SendNavGoal] Sending goal with %zu poses", goal_msg.poses.size());

        rclcpp::Time now = this->now();
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            auto &wp = waypoints[i];
            RCLCPP_INFO(this->get_logger(), "[SendNavGoal] Pose %zu: x=%.2f, y=%.2f, theta=%.2f",
                i, wp->x, wp->y, wp->theta);
            
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
        RCLCPP_INFO(this->get_logger(), "[SendNavGoal] Sending goal to nav2... (last_sent_target_=%d)", 
            last_sent_target_waypoint_);

        auto send_goal_options = typename rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const GoalHandleNav::SharedPtr goal_handle)
            {
                if (!goal_handle)
                {
                    RCLCPP_WARN(this->get_logger(), "[SendNavGoal] Goal rejected by nav2");
                    return;
                }

                std::lock_guard<std::mutex> lock(nav_status_mutex_);
                active_goal_uuid_ = goal_handle->get_goal_id();
                has_active_goal_uuid_ = true;
                goal_status_ = action_msgs::msg::GoalStatus::STATUS_ACCEPTED;
                RCLCPP_INFO(this->get_logger(), "[SendNavGoal] Goal accepted, bind goal_id for status tracking");
            };

        {
            std::lock_guard<std::mutex> lock(nav_status_mutex_);
            // 新目标发出前先清空旧goal_id绑定，避免误匹配旧状态
            has_active_goal_uuid_ = false;
            goal_status_ = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
        }

        nav_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "[SendNavGoal] EXIT: goal sent successfully");
        return true;
    }

    void cancelNavGoal()
    {
        RCLCPP_INFO(this->get_logger(), 
            "[CancelNavGoal] ENTER: nav_client_=%p, server_ready=%d, last_sent_target=%d, saved_target=%d, has_saved=%d",
            (void*)nav_client_.get(), 
            nav_client_ ? nav_client_->action_server_is_ready() : 0,
            last_sent_target_waypoint_, saved_target_waypoint_, has_saved_goal_);
        
        if (!nav_client_)
        {
            RCLCPP_WARN(this->get_logger(), "[CancelNavGoal] nav_client_ is null!");
            return;
        }
        if (!nav_client_->action_server_is_ready())
        {
            RCLCPP_WARN(this->get_logger(), "[CancelNavGoal] action server not ready!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "[CancelNavGoal] Sending cancel request...");
        nav_client_->async_cancel_all_goals();
        last_sent_target_waypoint_ = -1;
        {
            std::lock_guard<std::mutex> lock(nav_status_mutex_);
            has_active_goal_uuid_ = false;
            goal_status_ = action_msgs::msg::GoalStatus::STATUS_CANCELED;
        }
        RCLCPP_INFO(this->get_logger(), "[CancelNavGoal] Done. last_sent_target_ set to -1");
    }

    void resumeSavedGoal()
    {
        RCLCPP_INFO(this->get_logger(), 
            "[ResumeSavedGoal] ENTER: has_saved=%d, saved_target=%d, saved_if_succ=%d, "
            "last_sent=%d, goal_status=%d",
            has_saved_goal_, saved_target_waypoint_, saved_if_succession_,
            last_sent_target_waypoint_, goal_status_);
        
        if (!has_saved_goal_ || saved_target_waypoint_ < 0)
        {
            RCLCPP_WARN(this->get_logger(), "[ResumeSavedGoal] SKIP: no saved goal or invalid target");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(nav_status_mutex_);
            if ((goal_status_ == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                 goal_status_ == action_msgs::msg::GoalStatus::STATUS_EXECUTING) &&
                last_sent_target_waypoint_ != saved_target_waypoint_)
            {
                RCLCPP_INFO(this->get_logger(), 
                    "[ResumeSavedGoal] SKIP: a newer goal (wp=%d) is already in progress (saved=%d)",
                    last_sent_target_waypoint_, saved_target_waypoint_);
                has_saved_goal_ = false;
                return;
            }
        }

        RCLCPP_INFO(this->get_logger(), "[ResumeSavedGoal] Proceeding to get current position...");

        float current_x = 0.0f;
        float current_y = 0.0f;
        int current_waypoint_id = -1;
        bool has_feedback = false;
        {
            std::lock_guard<std::mutex> lock(nav_feedback_mutex_);
            if (latest_nav_feedback_)
            {
                has_feedback = true;
                current_x = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.x);
                current_y = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.y);
                current_waypoint_id = rdsys_->checkNowWayPoint(current_x, current_y);
                RCLCPP_INFO(this->get_logger(), 
                    "[ResumeSavedGoal] Got feedback: x=%.2f, y=%.2f, waypoint_id=%d",
                    current_x, current_y, current_waypoint_id);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[ResumeSavedGoal] No nav feedback available!");
            }
        }

        // 注意：即使 current_waypoint_id < 0 也要继续！
        // 因为 if_succession=false 时不需要当前路径点，Nav2 会自动规划
        if (current_waypoint_id < 0)
        {
            RCLCPP_WARN(this->get_logger(), 
                "[ResumeSavedGoal] WARNING: Cannot determine current waypoint (id=%d), but will try anyway with if_succession=false",
                current_waypoint_id);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), 
                "[ResumeSavedGoal] Current position: x=%.2f, y=%.2f, waypoint_id=%d",
                current_x, current_y, current_waypoint_id);
        }

        RCLCPP_INFO(this->get_logger(), 
            "[ResumeSavedGoal] Calling sendNavGoal: to wp=%d, if_succession=false",
            saved_target_waypoint_);
        
        const bool sent = sendNavGoal(current_waypoint_id, saved_target_waypoint_, false);

        if (sent)
        {
            RCLCPP_INFO(this->get_logger(), "[ResumeSavedGoal] Goal resumed successfully, clearing has_saved_goal_");
            has_saved_goal_ = false;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[ResumeSavedGoal] Resume failed, keep saved goal for next retry.");
        }
    }

    void positionCheckTimerCallback()
    {
        int8_t current_goal_status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
        {
            std::lock_guard<std::mutex> lock(nav_status_mutex_);
            current_goal_status = goal_status_;
        }

        bool enemy_visible = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!latest_autoaim_msg_.data.empty())
                enemy_visible = (latest_autoaim_msg_.data[0] != 0.0f);
        }

        // 如果有保存的目标或正在执行导航，跳过自检
        if (has_saved_goal_ ||
            enemy_visible ||
            current_goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
            current_goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
        {
            return;
        }

        // 获取当前位置
        float current_x, current_y;
        {
            std::lock_guard<std::mutex> lock(nav_feedback_mutex_);
            if (!latest_nav_feedback_)
                return;
            current_x = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.x);
            current_y = static_cast<float>(latest_nav_feedback_->feedback.current_pose.pose.position.y);
        }

        auto home_wp = rdsys_->getWayPointByID(home_waypoint_id_);
        auto center_wp = rdsys_->getWayPointByID(center_waypoint_id_);
        if (!home_wp || !center_wp)
        {
            RCLCPP_WARN(this->get_logger(), "[PositionCheck] home/center waypoint not found, skip.");
            return;
        }

        const float home_dx = home_wp->x - current_x;
        const float home_dy = home_wp->y - current_y;
        const float center_dx = center_wp->x - current_x;
        const float center_dy = center_wp->y - current_y;

        const float dist_to_home = std::sqrt(home_dx * home_dx + home_dy * home_dy);
        const float dist_to_center = std::sqrt(center_dx * center_dx + center_dy * center_dy);

        int key_wp_id = -1;
        float dist_to_key = 0.0f;
        if (dist_to_home <= dist_to_center)
        {
            key_wp_id = home_waypoint_id_;
            dist_to_key = dist_to_home;
        }
        else
        {
            key_wp_id = center_waypoint_id_;
            dist_to_key = dist_to_center;
        }

        // 仅在“足够靠近关键点”时做闭环校正，避免误吸附
        if (dist_to_key > position_check_activation_radius_)
            return;

        // 偏离超过阈值，发送校正目标
        if (dist_to_key > position_check_threshold_)
        {
            RCLCPP_INFO(this->get_logger(),
                "[PositionCheck] Adjusting position: wp=%d, dist=%.2f > %.2f, "
                "current=(%.2f, %.2f), target=(%.2f, %.2f)",
                key_wp_id, dist_to_key, position_check_threshold_,
                current_x, current_y,
                (key_wp_id == home_waypoint_id_ ? home_wp->x : center_wp->x),
                (key_wp_id == home_waypoint_id_ ? home_wp->y : center_wp->y));
            sendNavGoal(key_wp_id, key_wp_id, false, true);
        }
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

