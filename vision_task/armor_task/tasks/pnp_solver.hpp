#pragma once

#include "armor.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// 相机x对应世界-y,相机y对应世界-z,相机z对于世界x

namespace YAML
{
class Node;
}

class PnpSolver
{
  public:
    // 构造函数（通过yaml配置文件初始化）
    PnpSolver(const std::string &config_path);
    PnpSolver(const YAML::Node &config);

    // only debug use
    PnpSolver(const cv::Mat &camera_matrix, const cv::Mat &distortion_coeffs);
    ~PnpSolver() = default;

    // PnP位姿解算函数，直接修改Armor对象中的yaw和p_camera
    bool solvePnP(Armor &armor);

    // 判断是否是大装甲板
    bool Islarge(Armor &armor);

    // 批量解算装甲板数组，返回解算成功的数量
    int solveArmorArray(ArmorArray &armor_array);

    // 获取装甲板角点
    void getArmorCorners(Armor &armor);

    // 设置云台到世界坐标系旋转
    void set_R_gimbal2world(const Eigen::Quaterniond &q);

    // 相关旋转、平移矩阵
    Eigen::Matrix3d R_gimbal2imubody_;
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Vector3d t_camera2gimbal_;
    const Eigen::Matrix3d &gimbal2world() const { return R_gimbal2world_; }

    void optimize_yaw(Armor &armor) const;

    // 从灯条获取角点
    std::vector<cv::Point2f> getLightbarCorners(const LightBar &left_lightbar, const LightBar &right_lightbar);

    double oupost_reprojection_error(Armor armor, const double &pitch);

    double armor_reprojection_error(const Armor &armor, double yaw, const double &inclined) const;

    std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d &p_world, double yaw, int car_num, bool islarge) const;
    std::vector<cv::Point2f> fast_project_armor(const Eigen::Vector3d& p_world, double yaw, bool islarge) const;
    double SJTU_cost(const std::vector<cv::Point2f> &cv_refs, const std::vector<cv::Point2f> &cv_pts, const double &inclined) const;

    const cv::Mat &camera_matrix() const { return camera_matrix_; }
    const cv::Mat &distort_coeffs() const { return distort_coeffs_; }

  private:
    cv::Mat camera_matrix_;                 // 相机内参矩阵
    cv::Mat distort_coeffs_;                // 畸变系数
    std::vector<cv::Point3f> armor_points_; // 装甲板3D模型点
    Eigen::Matrix3d R_gimbal2world_;

    static constexpr float SMALL_ARMOR_WIDTH = 0.135;  // m
    static constexpr float SMALL_ARMOR_HEIGHT = 0.056; // m
    static constexpr float LARGE_ARMOR_WIDTH = 0.230;  // m
    static constexpr float LARGE_ARMOR_HEIGHT = 0.056; // m

    // 初始化装甲板3D模型点
    void initArmorPoints(bool is_large_armor);

    void initFromConfig(const YAML::Node &config);
};
