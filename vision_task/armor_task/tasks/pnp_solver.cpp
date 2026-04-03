#include "pnp_solver.hpp"
#include "../tools/math_tools.hpp"
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

constexpr double LIGHTBAR_LENGTH = 56e-3;    // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;   // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3; // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{{0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
                                                {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{{0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
                                                  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

PnpSolver::PnpSolver(const std::string &config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
    initFromConfig(YAML::LoadFile(config_path));
}

PnpSolver::PnpSolver(const YAML::Node &config) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
    initFromConfig(config);
}

void PnpSolver::initFromConfig(const YAML::Node &yaml)
{
    auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
    auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
    auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
    R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
    R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
    t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

    auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
    auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
    Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
    cv::eigen2cv(camera_matrix, camera_matrix_);
    cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

// only debug use
PnpSolver::PnpSolver(const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
    camera_matrix_ = camera_matrix.clone();
    distort_coeffs_ = distort_coeffs.clone();
    R_gimbal2imubody_ = Eigen::Matrix3d::Identity();
    R_camera2gimbal_ = Eigen::Matrix3d::Identity();
    t_camera2gimbal_ = Eigen::Vector3d::Zero();
}

// 后续需要优化，等规则出来看具体哪些是大装甲板
bool PnpSolver::Islarge(Armor &armor)
{
    //std::cout<<armor.car_num<<std::endl;
    if (armor.car_num == 1 )
    {
        armor.islarge = true; // 大装甲板
        return true;
    }
    else
    {
        armor.islarge = false; // 小装甲板
        return false;
    }
}

std::vector<cv::Point2f> PnpSolver::getLightbarCorners(const LightBar &left_lightbar, const LightBar &right_lightbar)
{
    std::vector<cv::Point2f> corners;

    corners.push_back(left_lightbar.top);
    corners.push_back(right_lightbar.top);
    corners.push_back(right_lightbar.bottom);
    corners.push_back(left_lightbar.bottom);

    return corners;
}

void PnpSolver::getArmorCorners(Armor &armor) { armor.corners = getLightbarCorners(armor.left_lightbar, armor.right_lightbar); }

void PnpSolver::set_R_gimbal2world(const Eigen::Quaterniond &q)
{
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

bool PnpSolver::solvePnP(Armor &armor)
{
    if (armor.left_lightbar.center == cv::Point2f(0, 0) || armor.right_lightbar.center == cv::Point2f(0, 0))
    {
        return false;
    }

    Islarge(armor) == true ? armor_points_ = BIG_ARMOR_POINTS : armor_points_ = SMALL_ARMOR_POINTS;

    getArmorCorners(armor);

    if (armor.corners.size() != 4)
    {
        return false;
    }

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(armor_points_, armor.corners, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    // std::cout<<"armor_points_"<<armor_points_<<std::endl;
    // std::cout<<"corners"<<armor.corners<<std::endl;
    if (!success)
    {
        return false;
    }

    armor.p_camera = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    Eigen::Vector3d p_gimbal = R_camera2gimbal_ * armor.p_camera + t_camera2gimbal_;
    //std::cout<<"R_camera2gimbal:"<<R_camera2gimbal_<<std::endl;
    armor.p_gimbal = R_camera2gimbal_ * armor.p_camera + t_camera2gimbal_;
    armor.p_world = R_gimbal2world_ * armor.p_gimbal;
     //std::cout<<"R_gimbal2world:"<<R_gimbal2world_<<std::endl;
    // std::cout << "p_world: " << armor.p_world.transpose() << std::endl;
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    // armor.yaw = std::atan2(rmat.at<double>(1, 0), rmat.at<double>(0, 0));
    //std::cout<< "tvec"<<tvec<< std::endl;
   
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
    armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
    armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);
    armor.ypd_in_world = tools::xyz2ypd(armor.p_world);

    // 平衡不做yaw优化，因为pitch假设不成立
    // 待优化
    // auto is_balance = armor.islarge &&
    //                 (armor.car_num == 3 || armor.car_num == 4 ||
    //                  armor.car_num == 5);
    // if (is_balance) return true;

    optimize_yaw(armor);

    return true;
}

int PnpSolver::solveArmorArray(ArmorArray &armor_array)
{
    int success_count = 0;

    for (size_t i = 0; i < armor_array.size(); ++i)
    {
        if (solvePnP(armor_array[i]))
        {
            success_count++;
        }
    }

    return success_count;
}

double PnpSolver::oupost_reprojection_error(Armor armor, const double &pitch)
{
    // solve
    const auto &object_points = (PnpSolver::Islarge(armor)) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    cv::Vec3d rvec, tvec;
    cv::solvePnP(object_points, armor.corners, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);
    armor.p_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    armor.p_world = R_gimbal2world_ * armor.p_gimbal;

    Eigen::Vector3d p_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_gimbal = Eigen::Vector3d::Zero();

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
    armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
    armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

    armor.ypd_in_world = tools::xyz2ypd(armor.p_world);

    auto yaw = armor.ypr_in_world[0];
    p_world = armor.p_world;

    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
    // clang-format on

    // get R_armor2camera t_armor2camera
    const Eigen::Vector3d &t_armor2world = p_world;
    Eigen::Matrix3d _R_armor2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
    Eigen::Vector3d t_armor2camera = R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    // get rvec tvec
    cv::Vec3d _rvec;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, _rvec);
    cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    // reproject
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

    auto error = 0.0;
    for (int i = 0; i < 4; i++) error += cv::norm(armor.corners[i] - image_points[i]);
    return error;
}
// 在 PnpSolver 类中添加辅助函数
std::vector<cv::Point2f> PnpSolver::fast_project_armor(const Eigen::Vector3d& p_world, double yaw, bool islarge) const
{
    // 1. 构建旋转矩阵 R (手动构建，跳过 Rodrigues)
    double sy = std::sin(yaw);
    double cosy = std::cos(yaw);
    // 假设 pitch 固定为 15度 (根据你的原代码)
    double sp = std::sin(15.0 * CV_PI / 180.0);
    double cp = std::cos(15.0 * CV_PI / 180.0);
    
    // R_armor2world
    Eigen::Matrix3d R_a2w;
    R_a2w << cosy*cp, -sy, cosy*sp,
             sy*cp,  cosy, sy*sp,
             -sp,     0,    cp;

    // 变换到相机坐标系: P_cam = R_c2g^T * (R_g2w^T * (R_a2w * P_obj + p_world) - t_c2g)
    // 注意：优化的时候，p_world已经是世界坐标系下的中心了，
    // 所以只需要旋转 4 个角点 offsets，加上中心，再变换到相机。
    
    // 预计算复合旋转 R_total = R_cam2gimbal^T * R_gimbal2world^T * R_a2w
    Eigen::Matrix3d R_total = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_a2w;
    
    // 预计算平移 t_total
    Eigen::Vector3d t_total = R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * p_world - t_camera2gimbal_);

    const auto& object_points = islarge ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    std::vector<cv::Point2f> image_points;
    image_points.reserve(4);

    // 获取相机内参 (假设已经存为 eigen 或者 double 变量，避免 cv::Mat 访问)
    double fx = camera_matrix_.at<double>(0, 0);
    double fy = camera_matrix_.at<double>(1, 1);
    double cx = camera_matrix_.at<double>(0, 2);
    double cy = camera_matrix_.at<double>(1, 2);

    for(const auto& pt : object_points) {
        Eigen::Vector3d p_obj(pt.x, pt.y, pt.z);
        // 变换到相机坐标系
        Eigen::Vector3d p_cam = R_total * p_obj + t_total;
        
        // 透视投影
        double inv_z = 1.0 / p_cam.z();
        double u = fx * p_cam.x() * inv_z + cx;
        double v = fy * p_cam.y() * inv_z + cy;
        
        image_points.emplace_back(u, v);
    }
    return image_points;
}
// void PnpSolver::optimize_yaw(Armor &armor) const
// {
//     Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

//     constexpr double SEARCH_RANGE = 140; // degree
//     auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

//     auto min_error = 1e10;
//     auto best_yaw = armor.ypr_in_world[0];

//     for (int i = 0; i < SEARCH_RANGE; i++)
//     {
//         double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
//         auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

//         if (error < min_error)
//         {
//             min_error = error;
//             best_yaw = yaw;
//         }
//     }

//     // armor.yaw_raw = armor.ypr_in_world[0];
//     armor.ypr_in_world[0] = best_yaw;
// }
void PnpSolver::optimize_yaw(Armor &armor) const
{
    // 1. 准备工作
    Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);
    double yaw_center = tools::limit_rad(gimbal_ypr[0]); 
    
    // 搜索参数配置
    constexpr double SEARCH_RANGE_DEG = 120.0;     // 搜索总范围
    constexpr double COARSE_STEP_DEG = 4.0;        // 粗搜索步长 (越小越准，但越慢)
    constexpr double FINE_SEARCH_RANGE_DEG = 6.0;  // 细搜索范围 (粗步长的1.5倍左右)

    double min_error = 1e18;
    double best_yaw = yaw_center;

    // lambda: 计算误差的辅助函数
    auto calc_error = [&](double yaw_rad) -> double {
        double yaw_norm = tools::limit_rad(yaw_rad);
        double inclined = yaw_rad - yaw_center; // 保持你原有的逻辑
        return armor_reprojection_error(armor, yaw_norm, inclined);
    };

    // -----------------------------------------------------------------
    // 阶段一：粗搜索 (Coarse Search) - 快速定位全局最优所在的谷底
    // -----------------------------------------------------------------
    int step_count = SEARCH_RANGE_DEG / COARSE_STEP_DEG;
    double start_deg = -SEARCH_RANGE_DEG / 2.0;
    
    for (int i = 0; i <= step_count; ++i)
    {
        double deg_offset = start_deg + i * COARSE_STEP_DEG;
        double current_yaw = yaw_center + deg_offset * CV_PI / 180.0;
        
        double error = calc_error(current_yaw);
        
        if (error < min_error)
        {
            min_error = error;
            best_yaw = current_yaw;
        }
    }

    // -----------------------------------------------------------------
    // 阶段二：细搜索 (Fine Search) - 在最优值附近使用黄金分割法
    // -----------------------------------------------------------------
    // 此时的 best_yaw 是粗搜索得到的近似值，以此为中心进行高精度收敛
    
    double range_rad = FINE_SEARCH_RANGE_DEG * CV_PI / 180.0;
    double a = best_yaw - range_rad;
    double b = best_yaw + range_rad;

    constexpr double PHI = 0.618033988749895; 
    double c = b - PHI * (b - a);
    double d = a + PHI * (b - a);
    
    double error_c = calc_error(c);
    double error_d = calc_error(d);

    // 提高精度要求
    constexpr double EPSILON = 0.05 * CV_PI / 180.0; 

    while (std::abs(b - a) > EPSILON)
    {
        if (error_c < error_d)
        {
            b = d;
            d = c;
            error_d = error_c;
            c = b - PHI * (b - a);
            error_c = calc_error(c);
        }
        else
        {
            a = c;
            c = d;
            error_c = error_d;
            d = a + PHI * (b - a);
            error_d = calc_error(d);
        }
    }

    // 取区间中点
    armor.ypr_in_world[0] = tools::limit_rad((a + b) / 2.0);
}

// void PnpSolver::optimize_yaw(Armor &armor) const
// {
//     // 1. 确定搜索的中心和范围
//     Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);
//     double yaw_center = tools::limit_rad(gimbal_ypr[0]); // 初始猜测值（云台Yaw或PnP初值）
    
//     // 搜索范围 (弧度制)
//     constexpr double SEARCH_RANGE_DEG = 140.0;
//     double range_rad = SEARCH_RANGE_DEG * CV_PI / 180.0;
    
//     // 定义左右边界 [a, b]
//     double a = yaw_center - range_rad / 2.0;
//     double b = yaw_center + range_rad / 2.0;

//     // 黄金分割比例 (sqrt(5)-1)/2
//     constexpr double PHI = 0.618033988749895; 
    
//     // 2. 预计算两个试探点 c 和 d
//     double c = b - PHI * (b - a);
//     double d = a + PHI * (b - a);

//     // 计算这两个点的误差 (封装成 lambda 方便调用，保持 inclined 逻辑)
//     auto calc_error = [&](double yaw) -> double {
//         double yaw_norm = tools::limit_rad(yaw);
//         // inclined 原逻辑是：当前角度与搜索起始点的差值。
//         // 这里为了兼容你的 SJTU_cost，建议传入 (yaw - yaw_center) 或是你原本逻辑中的相对值
//         double inclined = yaw - yaw_center; 
//         return armor_reprojection_error(armor, yaw_norm, inclined);
//     };

//     double error_c = calc_error(c);
//     double error_d = calc_error(d);

//     // 3. 迭代收敛
//     // 设置停止阈值，例如 0.1 度 (转化为弧度)
//     constexpr double EPSILON = 0.1 * CV_PI / 180.0; 

//     // 当区间宽度大于阈值时，继续缩小
//     while (std::abs(b - a) > EPSILON)
//     {
//         if (error_c < error_d)
//         {
//             // 最小值在 [a, d] 之间，丢弃 d 右边的部分
//             b = d;
//             d = c;           // 旧的 c 变成新的 d
//             error_d = error_c; // 误差直接复用，不用重算
            
//             // 只需要算一个新的 c
//             c = b - PHI * (b - a);
//             error_c = calc_error(c);
//         }
//         else
//         {
//             // 最小值在 [c, b] 之间，丢弃 c 左边的部分
//             a = c;
//             c = d;           // 旧的 d 变成新的 c
//             error_c = error_d; // 误差直接复用
            
//             // 只需要算一个新的 d
//             d = a + PHI * (b - a);
//             error_d = calc_error(d);
//         }
//     }

//     // 4. 取区间中点作为最终结果
//     double best_yaw = (a + b) / 2.0;
//     armor.ypr_in_world[0] = tools::limit_rad(best_yaw);
// }

double PnpSolver::SJTU_cost(const std::vector<cv::Point2f> &cv_refs, const std::vector<cv::Point2f> &cv_pts, const double &inclined) const
{
    std::size_t size = cv_refs.size();
    std::vector<Eigen::Vector2d> refs;
    std::vector<Eigen::Vector2d> pts;
    for (std::size_t i = 0u; i < size; ++i)
    {
        refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
        pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
    }
    double cost = 0.;
    for (std::size_t i = 0u; i < size; ++i)
    {
        std::size_t p = (i + 1u) % size;
        // i - p 构成线段。过程：先移动起点，再补长度，再旋转
        Eigen::Vector2d ref_d = refs[p] - refs[i]; // 标准
        Eigen::Vector2d pt_d = pts[p] - pts[i];
        // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
        double pixel_dis = // dis 是指方差平面内到原点的距离
            (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) + std::fabs(ref_d.norm() - pt_d.norm())) / ref_d.norm();
        double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
        // 平方可能是为了配合 sin 和 cos
        // 弧度差代价（0 度左右占比应该大）
        double cost_i = tools::square(pixel_dis * std::sin(inclined)) + tools::square(angular_dis * std::cos(inclined)) * 2.0; // DETECTOR_ERROR_PIXEL_BY_SLOPE
        // 重投影像素误差越大，越相信斜率
        cost += std::sqrt(cost_i);
    }
    return cost;
}

double PnpSolver::armor_reprojection_error(const Armor &armor, double yaw, const double &inclined) const
{
    auto image_points = reproject_armor(armor.p_world, yaw, armor.car_num, armor.islarge);
    //auto image_points = fast_project_armor(armor.p_world, yaw, armor.islarge);
    auto error = 0.0;
    // for (int i = 0; i < 4; i++) error += cv::norm(armor.corners[i] - image_points[i]);
    error = SJTU_cost(image_points, armor.corners, inclined);

    return error;
}

std::vector<cv::Point2f> PnpSolver::reproject_armor(const Eigen::Vector3d &p_world, double yaw, int car_num, bool islarge) const
{
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    auto pitch = 15.0 * CV_PI / 180.0;
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
    // clang-format on

    // get R_armor2camera t_armor2camera
    const Eigen::Vector3d &t_armor2world = p_world;
    Eigen::Matrix3d R_armor2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
    Eigen::Vector3d t_armor2camera = R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    // get rvec tvec
    cv::Vec3d rvec;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, rvec);
    cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    // reproject
    std::vector<cv::Point2f> image_points;
    const auto &object_points = islarge ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
    return image_points;
}
