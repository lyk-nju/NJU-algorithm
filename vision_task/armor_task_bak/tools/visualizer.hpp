#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "../include/armor.hpp"
#include "../tasks/target.hpp"
#include "../tasks/pnp_solver.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>

namespace armor_task
{

/**
 * @brief 3D可视化器，用于在RViz2中显示坐标、轨迹等信息
 */
class Visualizer3D
{
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针，用于发布marker消息
     * @param frame_id 坐标系ID，默认为"world"
     */
    explicit Visualizer3D(rclcpp::Node::SharedPtr node, const std::string &frame_id = "world");

    /**
     * @brief 发布装甲板位置（相机坐标系）
     * @param armors 装甲板列表
     */
    void publishArmors(const ArmorArray &armors);

    /**
     * @brief 发布装甲板位置（世界坐标系）
     * @param armors 装甲板列表
     */
    void publishArmorsWorld(const ArmorArray &armors);

    /**
     * @brief 发布目标预测位置
     * @param targets 目标列表
     */
    void publishTargets(const std::vector<Target> &targets);

    /**
     * @brief 发布弹道轨迹
     * @param trajectory_points 轨迹点列表（世界坐标系）
     * @param color_rgb RGB颜色值 (0-1)
     */
    void publishTrajectory(const std::vector<Eigen::Vector3d> &trajectory_points, 
                          const std::vector<double> &color_rgb = {1.0, 0.0, 0.0});

    /**
     * @brief 发布坐标系（相机、云台、世界）
     * @param pnp_solver PnP解算器，包含坐标系转换信息
     */
    void publishCoordinateFrames(const PnpSolver &pnp_solver);

    /**
     * @brief 发布瞄准点
     * @param aim_point 瞄准点（世界坐标系）
     * @param color_rgb RGB颜色值 (0-1)
     */
    void publishAimPoint(const Eigen::Vector3d &aim_point, 
                        const std::vector<double> &color_rgb = {0.0, 1.0, 0.0});

    /**
     * @brief 发布相机位置（世界坐标系）
     * @param pnp_solver PnP解算器，包含坐标系转换信息
     * @param gimbal_position_world 云台在世界坐标系中的位置（默认为原点）
     */
    void publishCameraPosition(const PnpSolver &pnp_solver, 
                              const Eigen::Vector3d &gimbal_position_world = Eigen::Vector3d::Zero());

    /**
     * @brief 清除所有marker
     */
    void clearAll();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::string frame_id_;
    int marker_id_counter_;

    /**
     * @brief 创建箭头marker（用于显示坐标系）
     */
    visualization_msgs::msg::Marker createArrowMarker(int id, 
                                                      const Eigen::Vector3d &origin,
                                                      const Eigen::Vector3d &direction,
                                                      const std::vector<double> &color_rgb,
                                                      double scale = 0.1);

    /**
     * @brief 创建球体marker（用于显示点）
     */
    visualization_msgs::msg::Marker createSphereMarker(int id,
                                                        const Eigen::Vector3d &position,
                                                        const std::vector<double> &color_rgb,
                                                        double radius = 0.05);

    /**
     * @brief 创建线条marker（用于显示轨迹）
     */
    visualization_msgs::msg::Marker createLineMarker(int id,
                                                     const std::vector<Eigen::Vector3d> &points,
                                                     const std::vector<double> &color_rgb,
                                                     double line_width = 0.01);

    /**
     * @brief 创建文本marker
     */
    visualization_msgs::msg::Marker createTextMarker(int id,
                                                      const Eigen::Vector3d &position,
                                                      const std::string &text,
                                                      const std::vector<double> &color_rgb,
                                                      double scale = 0.1);
};

} // namespace armor_task

#endif // VISUALIZER_HPP
