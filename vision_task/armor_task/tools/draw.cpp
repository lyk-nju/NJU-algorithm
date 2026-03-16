#include "draw.hpp"
#include "../tasks/aimer.hpp"
#include "../tasks/trajectory_normal.hpp"
#include "pharser.hpp"
#include <Eigen/Dense>
#include <iomanip>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

// ============================================================================
// 第一部分：基础绘图工具函数（tools 命名空间）
// ============================================================================
namespace tools
{

void draw_point(cv::Mat &img, const cv::Point &point, const cv::Scalar &color, int radius) { cv::circle(img, point, radius, color, -1); }

void draw_points(cv::Mat &img, const std::vector<cv::Point> &points, const cv::Scalar &color, int thickness)
{
    std::vector<std::vector<cv::Point>> contours = {points};
    cv::drawContours(img, contours, -1, color, thickness);
}

void draw_points(cv::Mat &img, const std::vector<cv::Point2f> &points, const cv::Scalar &color, int thickness)
{
    std::vector<cv::Point> int_points(points.begin(), points.end());
    draw_points(img, int_points, color, thickness);
}

void draw_text(cv::Mat &img, const std::string &text, const cv::Point &point, const cv::Scalar &color, double font_scale, int thickness)
{
    cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
}

std::vector<cv::Point2f> project3DPointsTo2D(const std::vector<cv::Point3f> &points_3d, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs)
{
    std::vector<cv::Point2f> points_2d;
    cv::projectPoints(points_3d, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camera_matrix, distort_coeffs, points_2d);
    return points_2d;
}

void draw_info_box(cv::Mat &image, const std::string &text, int font_scale, int thickness)
{
    // 1. 拆分多行
    std::vector<std::string> lines;
    std::stringstream ss(text);
    std::string line;
    while (std::getline(ss, line, '\n'))
    {
        lines.push_back(line);
    }

    // 2. 字体参数
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    int baseline = 0;
    int line_height = 0;
    int max_width = 0;

    // 3. 获取最大宽度和总高度
    for (const auto &l : lines)
    {
        cv::Size text_size = cv::getTextSize(l, font_face, font_scale, thickness, &baseline);
        line_height = std::max(line_height, text_size.height + baseline);
        max_width = std::max(max_width, text_size.width);
    }

    int padding = 10;
    int box_width = max_width + padding * 2;
    int box_height = line_height * lines.size() + padding * 2;

    // 4. 设置背景位置（右下角）
    int x = image.cols - box_width - 20;
    int y = image.rows - box_height - 20;

    // 5. 绘制背景矩形（半透明黑）
    cv::Mat roi = image(cv::Rect(x, y, box_width, box_height));
    cv::Mat overlay;
    roi.copyTo(overlay);
    cv::rectangle(overlay, cv::Point(0, 0), cv::Point(box_width, box_height), cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(overlay, 0.5, roi, 0.5, 0, roi);

    // 6. 绘制文字
    for (size_t i = 0; i < lines.size(); ++i)
    {
        cv::putText(image, lines[i], cv::Point(x + padding, y + padding + line_height * (i + 1) - baseline), font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
    }
}

void plotArmors(const ArmorArray &armors, cv::Mat &img)
{
    if (armors.empty())
    {
        cv::imshow("Armors", img);
        cv::waitKey(1);
        return;
    }

    for (const auto &armor : armors)
    {
        // 绘制检测框
        cv::rectangle(img, armor.box, cv::Scalar(0, 255, 0), 2);

        // 绘制中心点
        draw_point(img, armor.center, cv::Scalar(0, 0, 255), 4);

        // 绘制灯条位置
        draw_point(img, armor.left_lightbar.top, cv::Scalar(255, 0, 0), 3);
        draw_point(img, armor.left_lightbar.bottom, cv::Scalar(255, 0, 0), 3);
        draw_point(img, armor.right_lightbar.top, cv::Scalar(0, 0, 255), 3);
        draw_point(img, armor.right_lightbar.bottom, cv::Scalar(0, 0, 255), 3);

        // 装甲板信息
        std::stringstream ss;
        ss << "ID:" << armor.car_num << " Conf:" << std::fixed << std::setprecision(2) << armor.confidence;
        std::string color_str = (armor.color == Color::red ? "RED" : armor.color == Color::blue ? "BLUE" : "PURPLE");
        ss << " " << color_str;
        ss << " Pos:[" << std::fixed << std::setprecision(1) << armor.p_camera[0] << "," << armor.p_camera[1] << "," << armor.p_camera[2] << "]";
        ss << " Yaw:" << std::fixed << std::setprecision(1) << armor.yaw;

        draw_text(img, ss.str(), cv::Point(armor.box.x, std::max(0, armor.box.y - 10)), cv::Scalar(0, 255, 255), 0.5, 1);
    }

    cv::imshow("Armors", img);
    cv::waitKey(1);
}

void plotSingleArmor(const Armor &target, const Armor &armor, cv::Mat &img)
{
    if (armor.detect_id == -1)
    {
        cv::imshow("SingleArmor", img);
        cv::waitKey(1);
        return;
    }

    // 绘制检测框
    cv::rectangle(img, armor.box, cv::Scalar(0, 255, 0), 2);

    // 绘制中心点
    draw_point(img, target.center, cv::Scalar(0, 0, 255), 4);
    draw_point(img, armor.center, cv::Scalar(255, 0, 0), 4);

    // 绘制灯条
    draw_point(img, armor.left_lightbar.top, cv::Scalar(0, 255, 0), 3);
    draw_point(img, armor.left_lightbar.bottom, cv::Scalar(0, 255, 0), 3);
    draw_point(img, armor.right_lightbar.top, cv::Scalar(0, 255, 0), 3);
    draw_point(img, armor.right_lightbar.bottom, cv::Scalar(0, 255, 0), 3);

    // 装甲板信息
    std::stringstream ss;
    ss << "ID:" << armor.car_num << " Conf:" << std::fixed << std::setprecision(2) << armor.confidence;
    std::string color_str = (armor.color == Color::red ? "RED" : armor.color == Color::blue ? "BLUE" : "PURPLE");
    ss << " " << color_str;
    ss << " Pos:[" << std::fixed << std::setprecision(1) << armor.p_camera[0] << "," << armor.p_camera[1] << "," << armor.p_camera[2] << "]";
    ss << " Yaw:" << std::fixed << std::setprecision(1) << armor.yaw;

    draw_text(img, ss.str(), cv::Point(armor.box.x, std::max(0, armor.box.y - 10)), cv::Scalar(0, 255, 255), 0.5, 1);

    cv::imshow("SingleArmor", img);
    cv::waitKey(1);
}

} // namespace tools

// ============================================================================
// 第二部分：弹道相关辅助函数（匿名命名空间）
// ============================================================================
namespace
{
// 重力加速度
constexpr double kGravity = 9.7833;

// 计算弹道轨迹上的 3D 点（世界坐标系）- 基于目标位置（旧方法，假设相机已对准目标）
std::vector<Eigen::Vector3d> calculateTrajectoryPoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &target_pos, double bullet_speed, int num_points = 50)
{
    std::vector<Eigen::Vector3d> trajectory_points;

    // 水平距离和高度差
    // 世界坐标系定义：x向前，y向左，z向上
    // target_pos: (2.860, 0.398, -0.231) 表示：向前2.860m，向左0.398m，向下0.231m
    // std::cout << "target_pos: " << target_pos.x() << ", " << target_pos.y() << ", " << target_pos.z() << std::endl;
    // std::cout << "start_pos: " << start_pos.x() << ", " << start_pos.y() << ", " << start_pos.z() << std::endl;
    Eigen::Vector3d diff = target_pos - start_pos;
    // 水平距离：在xy平面（向前-向左平面）上的距离
    double d = std::sqrt(diff.x() * diff.x() + diff.y() * diff.y()); // 水平距离
    // 高度差：z方向（向上-向下方向）的差值
    double h = diff.z(); // 高度差

    // 使用解析解计算弹道参数
    armor_task::Trajectory traj(bullet_speed, d, h);
    if (traj.unsolvable)
    {
        return trajectory_points;
    }

    // 水平面上的yaw角：在xy平面（向前-向左平面）上的角度
    // atan2(y, x) 给出从x轴（向前）到目标点的角度
    double yaw = std::atan2(diff.y(), diff.x());

    // 初始速度分量（在世界坐标系中）
    // v0_x: 向前方向的速度分量
    // v0_y: 向左方向的速度分量
    // v0_z: 向上方向的速度分量
    double v0_x = bullet_speed * std::cos(traj.pitch) * std::cos(yaw);
    double v0_y = bullet_speed * std::cos(traj.pitch) * std::sin(yaw);
    double v0_z = bullet_speed * std::sin(traj.pitch);

    // 生成轨迹点
    double dt = traj.fly_time / static_cast<double>(num_points);
    for (int i = 0; i <= num_points; ++i)
    {
        double t = i * dt;
        if (t > traj.fly_time) t = traj.fly_time;

        double x = start_pos.x() + v0_x * t;
        double y = start_pos.y() + v0_y * t;
        double z = start_pos.z() + v0_z * t - 0.5 * kGravity * t * t;

        trajectory_points.emplace_back(x, y, z);
        // std::cout << "trajectory_points: " << x << ", " << y << ", " << z << std::endl;
    }

    return trajectory_points;
}

// 计算弹道轨迹上的 3D 点（世界坐标系）- 基于云台yaw和pitch角度（用于视频演示）
std::vector<Eigen::Vector3d> calculateTrajectoryPointsFromAngles(const Eigen::Vector3d &start_pos, double yaw_gimbal, double pitch_gimbal, double bullet_speed, int num_points = 50)
{
    std::vector<Eigen::Vector3d> trajectory_points;

    // 检查输入参数有效性
    if (bullet_speed <= 0 || std::isnan(bullet_speed) || std::isinf(bullet_speed))
    {
        std::cerr << "[calculateTrajectoryPointsFromAngles] Invalid bullet_speed: " << bullet_speed << std::endl;
        return trajectory_points;
    }

    if (std::isnan(yaw_gimbal) || std::isnan(pitch_gimbal) || std::isinf(yaw_gimbal) || std::isinf(pitch_gimbal))
    {
        std::cerr << "[calculateTrajectoryPointsFromAngles] Invalid angles: yaw=" << yaw_gimbal << ", pitch=" << pitch_gimbal << std::endl;
        return trajectory_points;
    }

    // 初始速度分量（在云台坐标系中）
    // 注意：pitch_gimbal 在 aimer 中已经取了负号，所以这里直接使用
    // yaw_gimbal 是云台需要转动的角度（相对于云台初始朝向）
    double cos_pitch = std::cos(pitch_gimbal);
    double sin_pitch = std::sin(pitch_gimbal);
    double cos_yaw = std::cos(yaw_gimbal);
    double sin_yaw = std::sin(yaw_gimbal);

    double v0_x_gimbal = bullet_speed * cos_pitch * cos_yaw;
    double v0_y_gimbal = bullet_speed * cos_pitch * sin_yaw;
    double v0_z_gimbal = bullet_speed * sin_pitch;

    // 计算飞行时间：需要知道目标距离
    // 由于我们不知道目标距离，我们需要迭代计算，或者使用一个合理的最大飞行时间
    // 这里我们使用一个简化的方法：假设最大飞行距离为 10 米，计算最大飞行时间
    double max_distance = 10.0;                       // 米
    if (std::abs(cos_pitch) < 1e-6) cos_pitch = 1e-6; // 防止除零
    double max_fly_time = max_distance / (bullet_speed * cos_pitch);
    if (max_fly_time <= 0 || std::isnan(max_fly_time) || std::isinf(max_fly_time))
    {
        max_fly_time = 1.0; // 防止除零或无效值
    }

    // 生成轨迹点（在云台坐标系中，从start_pos开始）
    // 注意：start_pos 已经是世界坐标系中的点，但由于 R_gimbal2world = Identity，所以也是云台坐标系中的点
    double dt = max_fly_time / static_cast<double>(num_points);
    for (int i = 0; i <= num_points; ++i)
    {
        double t = i * dt;
        if (t > max_fly_time) t = max_fly_time;

        // 在云台坐标系中计算轨迹点（从start_pos开始）
        double x_gimbal = start_pos.x() + v0_x_gimbal * t;
        double y_gimbal = start_pos.y() + v0_y_gimbal * t;
        double z_gimbal = start_pos.z() + v0_z_gimbal * t - 0.5 * kGravity * t * t;

        // 由于 R_gimbal2world = Identity（在测试环境中），云台坐标系与世界坐标系重合
        trajectory_points.emplace_back(x_gimbal, y_gimbal, z_gimbal);
    }

    return trajectory_points;
}

// 从 YAML 中加载相机到云台、云台到世界的变换矩阵
void loadTransformMatrices(const std::string &config_path, Eigen::Matrix3d &R_camera2gimbal, Eigen::Vector3d &t_camera2gimbal, Eigen::Matrix3d &R_gimbal2world)
{
    YAML::Node yaml = YAML::LoadFile(config_path);

    auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
    auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();

    R_camera2gimbal = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
    t_camera2gimbal = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

    // 当前测试环境下没有 IMU，先假设云台坐标系与世界坐标系重合
    R_gimbal2world = Eigen::Matrix3d::Identity();
}

// 将世界坐标系下的 3D 点投影到图像平面
std::vector<cv::Point2f> projectTrajectoryToImage(const std::vector<Eigen::Vector3d> &world_points,
                                                  const Eigen::Matrix3d &R_camera2gimbal,
                                                  const Eigen::Vector3d &t_camera2gimbal,
                                                  const Eigen::Matrix3d &R_gimbal2world,
                                                  const cv::Mat &camera_matrix,
                                                  const cv::Mat &distort_coeffs)
{
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> camera_points;

    for (size_t i = 0; i < world_points.size(); ++i)
    {
        const auto &pt_world = world_points[i];

        // 世界 -> 云台 -> 相机
        // 参考 pnp_solver.cpp 中的转换方式：
        // p_gimbal = R_gimbal2world * p_world (如果云台坐标系与世界坐标系重合，则 R_gimbal2world = I)
        // p_camera = R_camera2gimbal.transpose() * (p_gimbal - t_camera2gimbal)
        // 合并：p_camera = R_camera2gimbal.transpose() * (R_gimbal2world.transpose() * p_world - t_camera2gimbal)
        Eigen::Vector3d pt_camera = R_camera2gimbal.transpose() * (R_gimbal2world.transpose() * pt_world - t_camera2gimbal);

        // 调试输出：每10个点输出一次
        // if (i % 10 == 0)
        // {
        //     Eigen::Vector3d pt_gimbal = R_gimbal2world.transpose() * pt_world;
        //     std::cout << "[Project] pt_world: (" << pt_world.x() << ", " << pt_world.y() << ", " << pt_world.z() << ")" << std::endl;
        //     std::cout << "[Project] pt_gimbal: (" << pt_gimbal.x() << ", " << pt_gimbal.y() << ", " << pt_gimbal.z() << ")" << std::endl;
        //     std::cout << "[Project] pt_camera: (" << pt_camera.x() << ", " << pt_camera.y() << ", " << pt_camera.z() << ")" << std::endl;
        // }

        if (pt_camera.z() > 0.1)
        {
            camera_points.emplace_back(static_cast<float>(pt_camera.x()), static_cast<float>(pt_camera.y()), static_cast<float>(pt_camera.z()));
        }
    }

    if (camera_points.empty())
    {
        return image_points;
    }

    // 使用 cv::projectPoints 将相机坐标系中的3D点投影到图像平面
    // 注意：cv::projectPoints 期望的相机坐标系是 OpenCV 标准：x向右，y向下，z向前
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::projectPoints(camera_points, rvec, tvec, camera_matrix, distort_coeffs, image_points);
    // std::cout << "image_points.size(): " << image_points.size() << std::endl;
    // for (size_t i = 0; i < image_points.size(); ++i)
    // {
    //     std::cout << "image_points[" << i << "]: (" << image_points[i].x << ", " << image_points[i].y << ")" << std::endl;
    // }

    return image_points;
}

} // namespace

// ============================================================================
// 第三部分：主要绘图函数（全局命名空间）
// ============================================================================

// 绘制弹道轨迹（使用 Eigen::Vector3d 目标位置）
void drawTrajectory(cv::Mat &img, const Eigen::Vector3d &target_pos, double bullet_speed, const std::string &config_path)
{
    // 加载相机内参
    auto camera_params = loadCameraParameters(config_path);
    cv::Mat camera_matrix = camera_params.first;
    cv::Mat distort_coeffs = camera_params.second;

    // 加载外参
    Eigen::Matrix3d R_camera2gimbal;
    Eigen::Vector3d t_camera2gimbal;
    Eigen::Matrix3d R_gimbal2world;
    loadTransformMatrices(config_path, R_camera2gimbal, t_camera2gimbal, R_gimbal2world);

    // 发射点：从相机坐标系原点（相机光心）开始
    // 相机坐标系原点 -> 云台坐标系 -> 世界坐标系
    Eigen::Vector3d start_camera(0.15, 0.15, 0.1);                                   // 相机坐标系原点（相机光心）
    Eigen::Vector3d start_gimbal = R_camera2gimbal * start_camera + t_camera2gimbal; // 转换到云台坐标系
    Eigen::Vector3d start_pos = R_gimbal2world * start_gimbal;                       // 转换到世界坐标系

    // 计算弹道世界坐标系轨迹
    auto trajectory_points = calculateTrajectoryPoints(start_pos, target_pos, bullet_speed, 50);
    if (trajectory_points.empty())
    {
        return;
    }

    // 投影到图像平面
    auto image_points = projectTrajectoryToImage(trajectory_points, R_camera2gimbal, t_camera2gimbal, R_gimbal2world, camera_matrix, distort_coeffs);

    if (image_points.size() < 2)
    {
        std::cout << "image_points.size() < 2" << std::endl;
        return;
    }

    // 绘制弹道曲线
    cv::Scalar trajectory_color(0, 255, 255); // 黄色
    for (size_t i = 0; i + 1 < image_points.size(); ++i)
    {
        const auto &p1 = image_points[i];
        const auto &p2 = image_points[i + 1];
        if (p1.x >= 0 && p1.x < img.cols && p1.y >= 0 && p1.y < img.rows && p2.x >= 0 && p2.x < img.cols && p2.y >= 0 && p2.y < img.rows)
        {
            // std::cout << "p1: " << p1.x << ", " << p1.y << std::endl;
            // std::cout << "p2: " << p2.x << ", " << p2.y << std::endl;
            cv::line(img, p1, p2, trajectory_color, 2);
        }
    }

    // 起点（相机光心）- 通过重投影计算
    // 相机光心在相机坐标系中是 (0, 0, 0)，无法直接投影，使用一个非常接近的点
    cv::Point3f start_camera_pt(0.0f, 0.0f, 0.001f); // 使用很小的 z 值以便投影
    std::vector<cv::Point3f> start_camera_points = {start_camera_pt};
    std::vector<cv::Point2f> start_image_points;
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::projectPoints(start_camera_points, rvec, tvec, camera_matrix, distort_coeffs, start_image_points);

    // 终点
    const auto &p_end = image_points.back();
    if (p_end.x >= 0 && p_end.x < img.cols && p_end.y >= 0 && p_end.y < img.rows)
    {
        cv::circle(img, p_end, 5, cv::Scalar(0, 0, 255), -1);
    }
}

// 绘制弹道轨迹（使用 AimPoint，接受相机参数）
void drawTrajectory(cv::Mat &img, const AimPoint &aim_point, double bullet_speed, const std::string &config_path, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs, const Eigen::Matrix3d &R_gimbal2world)
{
    if (!aim_point.valid) return;

    // 1. 加载矩阵 (用于后续的物理轨迹计算)
    Eigen::Matrix3d R_camera2gimbal;
    Eigen::Vector3d t_camera2gimbal;
    Eigen::Matrix3d R_gimbal2world_cfg;
    loadTransformMatrices(config_path, R_camera2gimbal, t_camera2gimbal, R_gimbal2world_cfg);

    // =========================================================
    // 步骤 A: 定义枪口在相机系的固定位置 (Muzzle Offset)
    // =========================================================
    // 假设：右 15cm, 下 15cm, 前 20cm
    // 这个坐标相对于相机是永远不变的！
    Eigen::Vector3d muzzle_pos_cam_eig(0.1, 0.1, 0.5);
    cv::Point3f muzzle_pos_cam_cv(0.1f, 0.1f, 0.5);

    // =========================================================
    // 步骤 B: 计算物理轨迹 (在世界系中进行)
    // =========================================================
    // 起点转到世界系 (为了物理计算的正确性)
    Eigen::Vector3d start_gimbal = R_camera2gimbal * muzzle_pos_cam_eig + t_camera2gimbal;
    Eigen::Vector3d start_pos_world = R_gimbal2world * start_gimbal;
    Eigen::Vector3d target_pos_world = aim_point.xyza.head<3>();

    auto trajectory_points = calculateTrajectoryPoints(start_pos_world, target_pos_world, bullet_speed, 50);

    if (trajectory_points.empty()) return;

    // =========================================================
    // 步骤 C: 投影轨迹
    // =========================================================
    auto image_points = projectTrajectoryToImage(trajectory_points, R_camera2gimbal, t_camera2gimbal, R_gimbal2world, camera_matrix, distort_coeffs);

    if (image_points.size() > 1)
    {
        // 绘制轨迹
        cv::Scalar trajectory_color(0, 255, 255); // 黄色

        // 也可以画个圈标记固定的枪口位置
        {
            std::vector<cv::Point3f> local_points = {muzzle_pos_cam_cv};
            std::vector<cv::Point2f> local_pixels;
            cv::projectPoints(local_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), camera_matrix, distort_coeffs, local_pixels);
            const cv::Point2f fixed_start_pixel = local_pixels[0];
            if (fixed_start_pixel.x >= 0 && fixed_start_pixel.x < img.cols && fixed_start_pixel.y >= 0 && fixed_start_pixel.y < img.rows)
            {
                cv::circle(img, fixed_start_pixel, 6, cv::Scalar(0, 255, 0), 2);
            }
        }

        // std::cout << "Muzzle Pixel: (" << fixed_start_pixel.x << ", " << fixed_start_pixel.y << ")"
        //   << " | Image Size: " << img.cols << "x" << img.rows << std::endl;

        // 连线
        for (size_t i = 0; i < image_points.size() - 1; ++i)
        {
            // 简单的越界检查
            if (image_points[i].x >= 0 && image_points[i].x < img.cols && image_points[i + 1].x >= 0 && image_points[i + 1].x < img.cols)
            {
                cv::line(img, image_points[i], image_points[i + 1], trajectory_color, 2, cv::LINE_AA);
            }
        }

        // 绘制终点（目标点）
        if (!image_points.empty())
        {
            cv::circle(img, image_points.back(), 4, cv::Scalar(0, 0, 255), -1);
        }
    }
}

// 绘制弹道轨迹（使用云台yaw和pitch角度，用于视频演示）
void drawTrajectory(cv::Mat &img, double yaw_gimbal, double pitch_gimbal, double bullet_speed, const std::string &config_path, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs)
{
    // 加载变换矩阵
    Eigen::Matrix3d R_camera2gimbal;
    Eigen::Vector3d t_camera2gimbal;
    Eigen::Matrix3d R_gimbal2world;
    loadTransformMatrices(config_path, R_camera2gimbal, t_camera2gimbal, R_gimbal2world);

    // 发射点：从相机坐标系原点（相机光心）开始
    Eigen::Vector3d start_camera(0.15, 0.15, 0.1);                                   // 相机坐标系原点（相机光心）
    Eigen::Vector3d start_gimbal = R_camera2gimbal * start_camera + t_camera2gimbal; // 转换到云台坐标系
    Eigen::Vector3d start_pos = R_gimbal2world * start_gimbal;                       // 转换到世界坐标系

    // 使用基于角度的弹道计算函数
    auto trajectory_points = calculateTrajectoryPointsFromAngles(start_pos, yaw_gimbal, pitch_gimbal, bullet_speed, 50);

    if (trajectory_points.empty())
    {
        return;
    }

    // 将3D点投影到图像平面
    auto image_points = projectTrajectoryToImage(trajectory_points, R_camera2gimbal, t_camera2gimbal, R_gimbal2world, camera_matrix, distort_coeffs);

    // 绘制弹道曲线
    if (image_points.size() > 1)
    {
        // 使用黄色绘制弹道
        cv::Scalar trajectory_color(0, 255, 255); // 黄色 (BGR)
        for (size_t i = 0; i < image_points.size() - 1; ++i)
        {
            // 检查点是否在图像范围内
            if (image_points[i].x >= 0 && image_points[i].x < img.cols && image_points[i].y >= 0 && image_points[i].y < img.rows && image_points[i + 1].x >= 0 && image_points[i + 1].x < img.cols &&
                image_points[i + 1].y >= 0 && image_points[i + 1].y < img.rows)
            {
                cv::line(img, image_points[i], image_points[i + 1], trajectory_color, 2);
            }
        }

        // 绘制起点（发射点）- 通过重投影计算相机光心在图像上的投影
        cv::Point3f start_camera_pt(0.0f, 0.0f, 0.001f); // 使用很小的 z 值以便投影
        std::vector<cv::Point3f> start_camera_points = {start_camera_pt};
        std::vector<cv::Point2f> start_image_points;
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::projectPoints(start_camera_points, rvec, tvec, camera_matrix, distort_coeffs, start_image_points);

        // 绘制终点（目标点）
        if (!image_points.empty() && image_points.back().x >= 0 && image_points.back().x < img.cols && image_points.back().y >= 0 && image_points.back().y < img.rows)
        {
            cv::circle(img, image_points.back(), 5, cv::Scalar(0, 0, 255), -1); // 红色圆点
        }
    }
}

// 绘制装甲板检测结果（仅视觉元素：检测框、中心点、角点）
void drawArmorDetection(cv::Mat &img, const ArmorArray &armors)
{
    for (const auto &armor : armors)
    {
        // 绘制装甲板检测框
        // cv::rectangle(img, armor.box, cv::Scalar(0, 255, 0), 2);

        // 绘制装甲板中心点
        tools::draw_point(img, armor.center, cv::Scalar(0, 0, 255), 3);

        // 绘制装甲板角点（如果已解算）
        if (armor.p_camera != Eigen::Vector3d::Zero() && !armor.corners.empty())
        {
            tools::draw_points(img, armor.corners, cv::Scalar(0, 255, 0), 2);
            double yaw_cam = std::atan2(armor.p_camera.y(), armor.p_camera.x());
            std::ostringstream ss;
            ss.setf(std::ios::fixed);
            ss.precision(1);
            ss << "yaw:" << yaw_cam;
            tools::draw_text(img, ss.str(), armor.center + cv::Point2f(5, -5), cv::Scalar(255, 255, 0), 0.4, 1);
        }
    }
}

// 绘制Target详细信息
void drawTargetInfo(cv::Mat &img, const std::vector<Target> &targets, const std::string &tracker_state, const PnpSolver &pnp_solver)
{
    // 从camera1.yaml中加载相机参数
    auto camera_params = loadCameraParameters("../config/camera1.yaml");
    cv::Mat camera_matrix = camera_params.first;

    // 显示追踪器状态
    tools::draw_text(img, "Tracker State: " + tracker_state, cv::Point(10, 30), cv::Scalar(0, 255, 0), 0.7, 2);

    if (targets.empty())
    {
        tools::draw_text(img, "No Target", cv::Point(10, 60), cv::Scalar(0, 0, 255), 0.6, 2);
        return;
    }

    const auto &target = targets.front();
    int y_offset = 60;
    int line_height = 20;

    // Target基本信息
    tools::draw_text(img, "=== Target Info ===", cv::Point(10, y_offset), cv::Scalar(255, 255, 255), 0.6, 2);
    y_offset += line_height;

    // 获取所有装甲板的预测位置和角度
    auto xyza_list = target.armor_xyza_list();

    // 遍历所有装甲板并重投影
    for (size_t i = 0; i < xyza_list.size(); ++i)
    {
        // std::cout << "[Draw] Armor " << i << " world: (" << xyza_list[i][0] << ", " << xyza_list[i][1] << ", " << xyza_list[i][2] << ")" << std::endl;
        auto image_points = pnp_solver.reproject_armor(xyza_list[i].head<3>(), xyza_list[i][3], target.car_num, false);

        // 打印第一个角点的像素坐标
        // if (!image_points.empty())
        // {
        //     std::cout << "[Draw] Armor " << i << " pixel (corner 0): (" << image_points[0].x << ", " << image_points[0].y << ")" << std::endl;
        // }

        tools::draw_points(img, image_points, cv::Scalar(255, 0, 0), 2); // 使用蓝色绘制预测装甲板
    }

    Eigen::VectorXd ekf_state = target.ekf_x();

    if (ekf_state.size() >= 5)
    {
        Eigen::Vector3d ekf_world(ekf_state[0], ekf_state[2], ekf_state[4]);

        // std::cout << "[Draw] EKF center world: (" << ekf_world[0] << ", " << ekf_world[1] << ", " << ekf_world[2] << ")" << std::endl;

        // 使用 pnp_solver 的变换矩阵（与装甲板重投影保持一致）
        Eigen::Vector3d ekf_camera = pnp_solver.R_camera2gimbal_.transpose() * (pnp_solver.R_gimbal2world_.transpose() * ekf_world - pnp_solver.t_camera2gimbal_);

        // std::cout << "[Draw] EKF center camera: (" << ekf_camera[0] << ", " << ekf_camera[1] << ", " << ekf_camera[2] << ")" << std::endl;

        if (ekf_camera[2] > 0)
        {
            cv::Point2f ekf_proj;
            ekf_proj.x = camera_matrix.at<double>(0, 0) * ekf_camera[0] / ekf_camera[2] + camera_matrix.at<double>(0, 2);
            ekf_proj.y = camera_matrix.at<double>(1, 1) * ekf_camera[1] / ekf_camera[2] + camera_matrix.at<double>(1, 2);

            // std::cout << "[Draw] EKF center pixel: (" << ekf_proj.x << ", " << ekf_proj.y << ")" << std::endl;

            if (ekf_proj.x >= 0 && ekf_proj.x < img.cols && ekf_proj.y >= 0 && ekf_proj.y < img.rows)
            {
                tools::draw_point(img, ekf_proj, cv::Scalar(0, 0, 255), 5);
                tools::draw_text(img, "EKF", cv::Point(ekf_proj.x + 10, ekf_proj.y - 10), cv::Scalar(0, 0, 255), 0.4, 1);
            }
            else
            {
                std::cout << "[Draw] EKF center pixel out of bounds!" << std::endl;
            }
        }
    } // 是否切换标志
    if (target.is_switch_)
    {
        tools::draw_text(img, "TARGET SWITCHED!", cv::Point(10, y_offset), cv::Scalar(0, 0, 255), 0.6, 2);
        y_offset += line_height;
    }

    // 是否跳跃
    if (target.jumped)
    {
        tools::draw_text(img, "TARGET JUMPED!", cv::Point(10, y_offset), cv::Scalar(0, 165, 255), 0.6, 2);
        y_offset += line_height;
    }

    // EKF状态信息
    tools::draw_text(img, "=== EKF State ===", cv::Point(10, y_offset), cv::Scalar(255, 255, 255), 0.6, 2);
    y_offset += line_height;

    // 发散状态检查
    bool diverged = target.diverged();
    std::string diverge_status = diverged ? "DIVERGED!" : "Converged";
    cv::Scalar diverge_color = diverged ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
    tools::draw_text(img, "Status: " + diverge_status, cv::Point(10, y_offset), diverge_color, 0.5, 2);
    y_offset += line_height;
}

// 显示性能信息
void drawPerformanceInfo(cv::Mat &img, double fps, double detect_time, double track_time)
{
    int x_pos = img.cols - 200;
    int y_offset = 30;
    int line_height = 20;

    // FPS信息
    std::string fps_text = "FPS: " + std::to_string(fps).substr(0, 5);
    tools::draw_text(img, fps_text, cv::Point(x_pos, y_offset), cv::Scalar(0, 255, 0), 0.6, 2);
    y_offset += line_height;

    // 检测时间
    std::string detect_text = "Detect: " + std::to_string(detect_time).substr(0, 5) + "ms";
    tools::draw_text(img, detect_text, cv::Point(x_pos, y_offset), cv::Scalar(255, 255, 255), 0.5, 1);
    y_offset += line_height;

    // 追踪时间
    std::string track_text = "Track: " + std::to_string(track_time).substr(0, 5) + "ms";
    tools::draw_text(img, track_text, cv::Point(x_pos, y_offset), cv::Scalar(255, 255, 255), 0.5, 1);
}

// 显示性能信息（PnP版本）
void drawPerformanceInfo(cv::Mat &img, double fps, double detect_time, double pnp_time, int success_count)
{
    int x_pos = img.cols - 200;
    int y_offset = 30;
    int line_height = 20;

    // FPS信息
    std::string fps_text = "FPS: " + std::to_string(fps).substr(0, 5);
    tools::draw_text(img, fps_text, cv::Point(x_pos, y_offset), cv::Scalar(0, 255, 0), 0.6, 2);
    y_offset += line_height;

    // 检测时间
    std::string detect_text = "Detect: " + std::to_string(detect_time).substr(0, 5) + "ms";
    tools::draw_text(img, detect_text, cv::Point(x_pos, y_offset), cv::Scalar(255, 255, 255), 0.5, 1);
    y_offset += line_height;

    // PnP时间
    std::string pnp_text = "PnP: " + std::to_string(pnp_time).substr(0, 5) + "ms";
    tools::draw_text(img, pnp_text, cv::Point(x_pos, y_offset), cv::Scalar(255, 255, 255), 0.5, 1);
    y_offset += line_height;

    // PnP成功数量
    std::string success_text = "PnP Success: " + std::to_string(success_count);
    tools::draw_text(img, success_text, cv::Point(x_pos, y_offset), cv::Scalar(0, 255, 0), 0.5, 1);
}

// 在识别到的装甲板附近显示PnP解算结果（位置和姿态信息）
void drawPnPresult(cv::Mat &img, const ArmorArray &armors)
{
    for (const auto &armor : armors)
    {
        // 只显示成功解算的装甲板信息
        if (armor.p_camera.norm() > 0)
        {
            int text_x = armor.box.x;
            int text_y = armor.box.y + armor.box.height + 15;
            int line_height = 18;

            // 显示装甲板ID和编号（在检测框上方）
            if (armor.box.y > 20)
            {
                std::string id_text = "ID:" + std::to_string(armor.detect_id) + " Num:" + std::to_string(armor.car_num);
                tools::draw_text(img, id_text, cv::Point(text_x, armor.box.y - 5), cv::Scalar(0, 255, 0), 0.5, 1);
            }

            // 显示位置信息（相机坐标系，单位：mm）
            std::ostringstream pos_ss;
            pos_ss << std::fixed << std::setprecision(1);
            pos_ss << "Pos:(" << armor.p_camera[0] << "," << armor.p_camera[1] << "," << armor.p_camera[2] << ")mm";
            tools::draw_text(img, pos_ss.str(), cv::Point(text_x, text_y), cv::Scalar(0, 255, 255), 0.45, 1);

            // 显示姿态角信息（世界坐标系，单位：度）
            if (armor.ypr_in_world != Eigen::Vector3d::Zero())
            {
                std::ostringstream ypr_ss;
                ypr_ss << std::fixed << std::setprecision(2);
                ypr_ss << "YPR:(" << armor.ypr_in_world[0] * 180.0 / CV_PI << "," << armor.ypr_in_world[1] * 180.0 / CV_PI << "," << armor.ypr_in_world[2] * 180.0 / CV_PI << ")deg";
                tools::draw_text(img, ypr_ss.str(), cv::Point(text_x, text_y + line_height), cv::Scalar(255, 255, 0), 0.45, 1);
            }
        }
    }
}

// 绘制Target详细信息 (deprecated)
[[deprecated("Use the other overload of drawTargetInfo instead")]]
void drawTargetInfo(cv::Mat &img, const std::vector<armor_task::Target> &targets, const std::string &state, const cv::Mat & /* unused */ camera_matrix)
{
    int y_offset = 60;
    int line_height = 20;

    // 显示追踪状态
    std::string state_text = "State: " + state;
    cv::putText(img, state_text, cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
    y_offset += line_height;

    // 显示目标数量
    std::string target_count = "Targets: " + std::to_string(targets.size());
    cv::putText(img, target_count, cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    y_offset += line_height;

    // 为每个目标绘制信息
    for (size_t i = 0; i < targets.size(); ++i)
    {
        const auto &target = targets[i];
        Eigen::VectorXd ekf_x = target.ekf_x();

        // 显示目标位置
        std::string pos_text = "T" + std::to_string(i) + ": (" + std::to_string((int)ekf_x[0]) + ", " + std::to_string((int)ekf_x[2]) + ", " + std::to_string((int)ekf_x[4]) + ")";
        cv::putText(img, pos_text, cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
        y_offset += line_height;

        // 显示速度
        std::string vel_text = "V: (" + std::to_string((int)ekf_x[1]) + ", " + std::to_string((int)ekf_x[3]) + ", " + std::to_string((int)ekf_x[5]) + ")";
        cv::putText(img, vel_text, cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
        y_offset += line_height;
    }
}
