#include "visualizer.hpp"
#include <chrono>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace armor_task
{

Visualizer3D::Visualizer3D(rclcpp::Node::SharedPtr node, const std::string &frame_id) : node_(node), frame_id_(frame_id), marker_id_counter_(0)
{
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
}

void Visualizer3D::publishArmors(const ArmorArray &armors)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto &armor : armors)
    {
        if (armor.p_camera.norm() > 0)
        {
            // 发布装甲板位置（相机坐标系）
            auto sphere = createSphereMarker(id++, armor.p_camera, {1.0, 0.0, 0.0}, 0.05);
            marker_array.markers.push_back(sphere);

            // 发布文本标签
            std::string text = "Armor " + std::to_string(armor.detect_id) + " (Cam)";
            auto text_marker = createTextMarker(id++, armor.p_camera, text, {1.0, 1.0, 1.0});
            marker_array.markers.push_back(text_marker);
        }
    }

    if (!marker_array.markers.empty())
    {
        marker_pub_->publish(marker_array);
    }
}

void Visualizer3D::publishArmorsWorld(const ArmorArray &armors)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto &armor : armors)
    {
        if (armor.p_world.norm() > 0)
        {
            // 发布装甲板位置（世界坐标系）
            auto sphere = createSphereMarker(id++, armor.p_world, {0.0, 1.0, 0.0}, 0.08);
            marker_array.markers.push_back(sphere);

            // 发布文本标签（包含坐标信息）
            std::ostringstream text_ss;
            text_ss << std::fixed << std::setprecision(2);
            text_ss << "Armor " << armor.detect_id << " (" << armor.p_world.x() << ", " << armor.p_world.y() << ", " << armor.p_world.z() << ")";
            // auto text_marker = createTextMarker(id++, armor.p_world, text_ss.str(), {1.0, 1.0, 1.0});
            // marker_array.markers.push_back(text_marker);

            // 发布姿态箭头（yaw方向）
            // if (armor.ypr_in_world.norm() > 0)
            // {
            //     double yaw = armor.ypr_in_world[0];
            //     Eigen::Vector3d direction(std::cos(yaw), std::sin(yaw), 0.0);
            //     auto arrow = createArrowMarker(id++, armor.p_world, direction, {0.0, 0.0, 1.0}, 0.2);
            //     marker_array.markers.push_back(arrow);
            // }
        }
    }

    if (!marker_array.markers.empty())
    {
        marker_pub_->publish(marker_array);
    }
}

void Visualizer3D::publishTargets(const std::vector<Target> &targets)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // 首先发送一个DELETE ALL marker来清除所有旧的target markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id_;
    delete_marker.header.stamp = rclcpp::Clock().now();
    delete_marker.ns = "targets";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // 为每个 target 使用固定的 ID 范围，而不是全局递增
    // Target 0: ID 1000-1020
    // Target 1: ID 1021-1040
    // 等等...
    const int markers_per_target = 20; // 每个 target 最多 20 个 marker
    const int base_id = 1000;          // 起始 ID

    for (size_t i = 0; i < targets.size(); ++i)
    {
        const auto &target = targets[i];

        // 每个 target 使用固定的 ID 范围起始点
        // 注意：每次循环都重新设置 id，不累加到下一个 target
        int id = base_id + i * markers_per_target;

        Eigen::VectorXd ekf_x = target.ekf_x();
        if (ekf_x.size() < 5)
        {
            // std::cout << "[Visualizer] Target " << i << " ekf_x size < 5: " << ekf_x.size() << std::endl;
            continue;
        }

        // 发布目标中心位置
        Eigen::Vector3d center(ekf_x[0], ekf_x[2], ekf_x[4]);

        // 检查坐标是否有效
        if (std::isnan(center.x()) || std::isnan(center.y()) || std::isnan(center.z()) || std::isinf(center.x()) || std::isinf(center.y()) || std::isinf(center.z()))
        {
            continue;
        }

        auto sphere = createSphereMarker(id++, center, {1.0, 1.0, 0.0}, 0.1);
        marker_array.markers.push_back(sphere);

        // 发布文本标签 (注释掉以减少marker数量)
        // std::string text = "Target " + std::to_string(i) + " (Car " + std::to_string(target.car_num) + ")";
        // auto text_marker = createTextMarker(id++, center, text, {1.0, 1.0, 0.0});
        // marker_array.markers.push_back(text_marker);

        // 获取所有装甲板位置（当前观测到的 + 估计的其他三个）
        auto xyza_list = target.armor_xyza_list();
        int observed_armor_id = target.last_id; // 当前观测到的装甲板ID

        // std::cout << "[Visualizer] Target " << i << " center=(" << center.x() << ", " << center.y() << ", " << center.z() << "), armors=" << xyza_list.size() << ", observed_id=" << observed_armor_id << std::endl;

        // 发布所有四个装甲板位置
        for (size_t j = 0; j < xyza_list.size(); ++j)
        {
            const auto &xyza = xyza_list[j];
            Eigen::Vector3d armor_pos = xyza.head<3>();

            // 输出每个装甲板的坐标用于调试
            // std::cout << "[Visualizer]   Armor[" << j << "] pos=(" << armor_pos.x() << ", " << armor_pos.y() << ", " << armor_pos.z() << ")"
            //           << (j == static_cast<size_t>(observed_armor_id) ? " [OBSERVED]" : " [PREDICTED]") << std::endl;

            // 检查坐标是否有效
            if (std::isnan(armor_pos.x()) || std::isnan(armor_pos.y()) || std::isnan(armor_pos.z()) || std::isinf(armor_pos.x()) || std::isinf(armor_pos.y()) || std::isinf(armor_pos.z()))
            {
                // std::cout << "[Visualizer]   Armor " << j << " has invalid coordinates, skipping" << std::endl;
                continue;
            }

            // 区分当前观测到的装甲板和估计的装甲板
            if (j == static_cast<size_t>(observed_armor_id))
            {
                // 当前观测到的装甲板：绿色，较大
                auto armor_sphere = createSphereMarker(id++, armor_pos, {0.0, 1.0, 0.0}, 0.1);
                marker_array.markers.push_back(armor_sphere);

                // 装甲板文本标签 (注释掉以减少marker数量)
                // std::ostringstream armor_text_ss;
                // armor_text_ss << "Observed Armor " << j << " (Car " << target.car_num << ")";
                // auto armor_text = createTextMarker(id++, armor_pos, armor_text_ss.str(), {0.0, 1.0, 0.0}, 0.12);
                // marker_array.markers.push_back(armor_text);
            }
            else
            {
                // 估计的其他装甲板：红色（更醒目），较大
                auto armor_sphere = createSphereMarker(id++, armor_pos, {1.0, 0.0, 0.0}, 0.12);
                marker_array.markers.push_back(armor_sphere);

                // 装甲板文本标签 (注释掉以减少marker数量)
                // std::ostringstream armor_text_ss;
                // armor_text_ss << "Predicted Armor " << j << " (Car " << target.car_num << ")";
                // auto armor_text = createTextMarker(id++, armor_pos, armor_text_ss.str(), {1.0, 0.0, 0.0}, 0.12);
                // marker_array.markers.push_back(armor_text);
            }
        }

        // 可选：添加连接线显示车辆轮廓（连接相邻的装甲板）
        if (xyza_list.size() >= 2)
        {
            std::vector<Eigen::Vector3d> vehicle_outline;
            bool all_valid = true;
            for (size_t j = 0; j < xyza_list.size(); ++j)
            {
                Eigen::Vector3d pos = xyza_list[j].head<3>();
                if (std::isnan(pos.x()) || std::isnan(pos.y()) || std::isnan(pos.z()) || std::isinf(pos.x()) || std::isinf(pos.y()) || std::isinf(pos.z()))
                {
                    all_valid = false;
                    break;
                }
                vehicle_outline.push_back(pos);
            }

            if (all_valid)
            {
                // 闭合轮廓（连接最后一个到第一个）
                vehicle_outline.push_back(xyza_list[0].head<3>());

                auto outline_line = createLineMarker(id++, vehicle_outline, {0.5, 0.5, 0.5}, 0.01);
                marker_array.markers.push_back(outline_line);
            }
        }
    }

    // 不再需要更新全局计数器，因为使用固定 ID
    // marker_id_counter_ = id;

    if (!marker_array.markers.empty())
    {
        // std::cout << "[Visualizer] Publishing " << marker_array.markers.size() << " target markers" << std::endl;
        marker_pub_->publish(marker_array);
    }
    else
    {
        // std::cout << "[Visualizer] No markers to publish for targets (targets.size()=" << targets.size() << ")" << std::endl;
    }
}

void Visualizer3D::publishTrajectory(const std::vector<Eigen::Vector3d> &trajectory_points, const std::vector<double> &color_rgb)
{
    if (trajectory_points.size() < 2) return;

    visualization_msgs::msg::MarkerArray marker_array;
    auto line_marker = createLineMarker(0, trajectory_points, color_rgb, 0.02);
    marker_array.markers.push_back(line_marker);

    // 在起点和终点添加特殊标记
    auto start_sphere = createSphereMarker(1, trajectory_points.front(), {0.0, 1.0, 0.0}, 0.08);
    marker_array.markers.push_back(start_sphere);

    auto end_sphere = createSphereMarker(2, trajectory_points.back(), {1.0, 0.0, 0.0}, 0.08);
    marker_array.markers.push_back(end_sphere);

    marker_pub_->publish(marker_array);
}

void Visualizer3D::publishCoordinateFrames(const PnpSolver &pnp_solver)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    // 世界坐标系原点
    Eigen::Vector3d origin(0, 0, 0);

    // X轴（红色）
    auto x_arrow = createArrowMarker(id++, origin, Eigen::Vector3d(1, 0, 0), {1.0, 0.0, 0.0}, 0.3);
    marker_array.markers.push_back(x_arrow);

    // Y轴（绿色）
    auto y_arrow = createArrowMarker(id++, origin, Eigen::Vector3d(0, 1, 0), {0.0, 1.0, 0.0}, 0.3);
    marker_array.markers.push_back(y_arrow);

    // Z轴（蓝色）
    auto z_arrow = createArrowMarker(id++, origin, Eigen::Vector3d(0, 0, 1), {0.0, 0.0, 1.0}, 0.3);
    marker_array.markers.push_back(z_arrow);

    // 云台坐标系（如果已知云台位置）
    // 这里假设云台在世界坐标系原点，实际应该从配置或IMU获取
    // 可以添加云台位置的箭头

    marker_pub_->publish(marker_array);
}

void Visualizer3D::publishAimPoint(const Eigen::Vector3d &aim_point, const std::vector<double> &color_rgb)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // 发布瞄准点（较大的球体）
    auto sphere = createSphereMarker(0, aim_point, color_rgb, 0.12);
    marker_array.markers.push_back(sphere);

    // 发布文本标签
    std::string text = "Aim Point";
    auto text_marker = createTextMarker(1, aim_point, text, color_rgb);
    marker_array.markers.push_back(text_marker);

    marker_pub_->publish(marker_array);
}

void Visualizer3D::publishCameraPosition(const PnpSolver &pnp_solver, const Eigen::Vector3d &gimbal_position_world)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    // 计算相机在世界坐标系中的位置
    // 相机在云台坐标系中的位置：t_camera2gimbal
    // 云台在世界坐标系中的位置：gimbal_position_world（通常为原点）
    // 相机在世界坐标系中的位置 = R_gimbal2world * t_camera2gimbal + gimbal_position_world
    Eigen::Vector3d camera_position_gimbal = pnp_solver.t_camera2gimbal_;
    Eigen::Vector3d camera_position_world = pnp_solver.R_gimbal2world_ * camera_position_gimbal + gimbal_position_world;

    // 发布相机位置（紫色球体，较大）
    auto camera_sphere = createSphereMarker(id++, camera_position_world, {1.0, 0.0, 1.0}, 0.15);
    marker_array.markers.push_back(camera_sphere);

    // 发布文本标签
    std::ostringstream text_ss;
    text_ss << std::fixed << std::setprecision(2);
    text_ss << "Camera (" << camera_position_world.x() << ", " << camera_position_world.y() << ", " << camera_position_world.z() << ")";
    auto text_marker = createTextMarker(id++, camera_position_world, text_ss.str(), {1.0, 0.0, 1.0}, 0.15);
    marker_array.markers.push_back(text_marker);

    // 发布相机坐标系箭头（从相机位置出发）
    // X轴（红色）- 相机右方向
    Eigen::Vector3d camera_x_world = pnp_solver.R_gimbal2world_ * pnp_solver.R_camera2gimbal_.transpose() * Eigen::Vector3d(1, 0, 0);
    auto x_arrow = createArrowMarker(id++, camera_position_world, camera_x_world, {1.0, 0.0, 0.0}, 0.2);
    marker_array.markers.push_back(x_arrow);

    // Y轴（绿色）- 相机下方向
    Eigen::Vector3d camera_y_world = pnp_solver.R_gimbal2world_ * pnp_solver.R_camera2gimbal_.transpose() * Eigen::Vector3d(0, 1, 0);
    auto y_arrow = createArrowMarker(id++, camera_position_world, camera_y_world, {0.0, 1.0, 0.0}, 0.2);
    marker_array.markers.push_back(y_arrow);

    // Z轴（蓝色）- 相机前方向
    Eigen::Vector3d camera_z_world = pnp_solver.R_gimbal2world_ * pnp_solver.R_camera2gimbal_.transpose() * Eigen::Vector3d(0, 0, 1);
    auto z_arrow = createArrowMarker(id++, camera_position_world, camera_z_world, {0.0, 0.0, 1.0}, 0.2);
    marker_array.markers.push_back(z_arrow);

    // 发布云台位置（如果不在原点）
    if (gimbal_position_world.norm() > 0.01)
    {
        auto gimbal_sphere = createSphereMarker(id++, gimbal_position_world, {0.5, 0.5, 0.5}, 0.1);
        marker_array.markers.push_back(gimbal_sphere);

        std::ostringstream gimbal_text_ss;
        gimbal_text_ss << "Gimbal (" << gimbal_position_world.x() << ", " << gimbal_position_world.y() << ", " << gimbal_position_world.z() << ")";
        auto gimbal_text = createTextMarker(id++, gimbal_position_world, gimbal_text_ss.str(), {0.5, 0.5, 0.5}, 0.12);
        marker_array.markers.push_back(gimbal_text);
    }

    marker_pub_->publish(marker_array);
}

void Visualizer3D::clearAll()
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    marker_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker Visualizer3D::createArrowMarker(int id, const Eigen::Vector3d &origin, const Eigen::Vector3d &direction, const std::vector<double> &color_rgb, double scale)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "arrows";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start, end;
    start.x = origin.x();
    start.y = origin.y();
    start.z = origin.z();

    Eigen::Vector3d normalized_dir = direction.normalized();
    end.x = origin.x() + normalized_dir.x() * scale;
    end.y = origin.y() + normalized_dir.y() * scale;
    end.z = origin.z() + normalized_dir.z() * scale;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.02; // 箭头直径
    marker.scale.y = 0.04; // 箭头头部直径
    marker.scale.z = 0.0;

    marker.color.r = color_rgb[0];
    marker.color.g = color_rgb[1];
    marker.color.b = color_rgb[2];
    marker.color.a = 1.0;

    // 设置 lifetime，让 marker 在 0.1 秒后自动过期，避免闪烁
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 100000000; // 0.1 秒

    return marker;
}

visualization_msgs::msg::Marker Visualizer3D::createSphereMarker(int id, const Eigen::Vector3d &position, const std::vector<double> &color_rgb, double radius)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "spheres";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;

    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = radius * 2;

    marker.color.r = color_rgb[0];
    marker.color.g = color_rgb[1];
    marker.color.b = color_rgb[2];
    marker.color.a = 1.0;

    // 设置 lifetime，让 marker 在 0.1 秒后自动过期，避免闪烁
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 100000000; // 0.1 秒

    return marker;
}

visualization_msgs::msg::Marker Visualizer3D::createLineMarker(int id, const std::vector<Eigen::Vector3d> &points, const std::vector<double> &color_rgb, double line_width)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "trajectory";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    for (const auto &pt : points)
    {
        geometry_msgs::msg::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        marker.points.push_back(p);
    }

    marker.scale.x = line_width;
    marker.color.r = color_rgb[0];
    marker.color.g = color_rgb[1];
    marker.color.b = color_rgb[2];
    marker.color.a = 1.0;

    // 设置 lifetime，让 marker 在 0.1 秒后自动过期，避免闪烁
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 100000000; // 0.1 秒

    return marker;
}

visualization_msgs::msg::Marker Visualizer3D::createTextMarker(int id, const Eigen::Vector3d &position, const std::string &text, const std::vector<double> &color_rgb, double scale)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "text";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z() + 0.1; // 稍微抬高一点
    marker.pose.orientation.w = 1.0;

    marker.scale.z = scale;
    marker.text = text;

    marker.color.r = color_rgb[0];
    marker.color.g = color_rgb[1];
    marker.color.b = color_rgb[2];
    marker.color.a = 1.0;

    // 设置 lifetime，让 marker 在 0.1 秒后自动过期，避免闪烁
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 100000000; // 0.1 秒

    return marker;
}

} // namespace armor_task
