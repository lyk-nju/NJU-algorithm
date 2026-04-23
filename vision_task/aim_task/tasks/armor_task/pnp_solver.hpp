#pragma once

#include "../../io/structs/camera_info.hpp"
#include "armor.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// camera x -> world -y, camera y -> world -z, camera z -> world x

namespace armor_task
{
class Target;
struct AimPoint;

class PnpSolver
{
  public:
    PnpSolver(const std::string &config_path);

    // only debug use
    PnpSolver(const cv::Mat &camera_matrix, const cv::Mat &distortion_coeffs);
    ~PnpSolver() = default;

    bool solve_pnp(Armor &armor);

    bool is_large(Armor &armor);

    int solveArmorArray(ArmorArray &armor_array);

    void set_R_gimbal2world(const Eigen::Quaterniond &q);

    // 切换当前相机参数（内??+ 外参 + distortion）。多相机场景下每帧由
    // AimPipeline 根据 FrameBundle.camera 调用；单相机场景??AutoAimSystem
    // 也会每帧调一次，数值等于构造时??yaml 读入的那一组——因此新增此接口
    // 不改变现行行为??
    //
    // 快速路径：若本??info ??camera_id 与对象地址都与上次相同，直??return??
    // 不做任何复制。单相机场景??AutoAimSystem::camera_info_ 是稳定的成员对象??
    // 每帧都命中快速路径，零开销。切换相机或遇到新的 CameraInfo 对象时才
    // clone 一??cv::Mat，避??cv::Mat shallow-copy 踩到外部对象销??改写
    // 的悬空风险??
    //
    // 契约：info.camera_matrix / info.distort_coeffs 必须非空；否则会??
    // solve_pnp 崩在 cv::solvePnP 上。AutoAimSystem::start() 负责??
    // yaml（或 PnpSolver 已读入的值）填好 CameraInfo 再传进来??
    void set_camera(const io::CameraInfo &info);

    Eigen::Matrix3d R_gimbal2imubody_;
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Vector3d t_camera2gimbal_;
    Eigen::Matrix3d R_gimbal2world_;

    void optimize_yaw(Armor &armor) const;

    double armor_reprojection_error(const Armor &armor, double yaw, const double &inclined) const;

    std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d &p_world, double yaw, bool is_large) const;
    std::vector<std::vector<cv::Point2f>> reproject_armor(const Target &target) const;
    std::vector<cv::Point2f> reproject_armor(const AimPoint &aim_point, bool is_large = false) const;
    double SJTU_cost(const std::vector<cv::Point2f> &cv_refs, const std::vector<cv::Point2f> &cv_pts, const double &inclined) const;

    const cv::Mat &camera_matrix() const { return camera_matrix_; }
    const cv::Mat &distort_coeffs() const { return distort_coeffs_; }

    // 把当前相机参数打包成 CameraInfo 导出。Step A 过渡期：上层可以??
    // 构造期??yaml 读入的那组参数直接拿出来??FrameBundle.camera，避??
    // 重复解析 yaml 带来的潜在差异。Step C 接多相机后，上层会直接从
    // 配置构??CameraInfo，此 getter 仍然可用于调??fallback??
    io::CameraInfo as_camera_info(int camera_id = 0, std::string name = {}) const;

  private:
    cv::Mat camera_matrix_;
    cv::Mat distort_coeffs_;

    // set_camera ??fast-path 缓存键：
    //   - last_camera_id_ == -1 表示"还没??set_camera 调过"（构造期直接??yaml 的状态）??
    //   - last_info_ptr_ 记录上次传入 CameraInfo 的对象地址；即??camera_id 相同??
    //     对象地址不同也视为换了一份，要重刷（防御"??id 不同对象且内容被改过"的场景）??
    //   - 同时命中 id + 地址才跳过复制??
    int last_camera_id_ = -1;
    const io::CameraInfo *last_info_ptr_ = nullptr;
};

}  // namespace armor_task
