#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <string>

namespace io
{

// 相机的内外参与元信息，按 camera_id 在 ICameraRig / CameraRegistry 中索引。
// 多相机系统里每一路都有独立一份。单相机时这就是当前配置里的那一套参数。
struct CameraInfo
{
    int camera_id = 0;                       // 全系统唯一 ID
    std::string name;                        // "front" / "left" / ... 仅用于日志

    cv::Mat camera_matrix;                   // 3x3 内参
    cv::Mat distort_coeffs;                  // 1x5 畸变

    // 相机坐标系 -> 云台坐标系的刚体变换
    Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_camera2gimbal = Eigen::Vector3d::Zero();
};

} // namespace io
