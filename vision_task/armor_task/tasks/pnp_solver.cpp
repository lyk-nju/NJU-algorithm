#include "pnp_solver.hpp"
#include "../tools/math_tools.hpp"
#include <cmath>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

namespace armor_task
{

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
    auto yaml = YAML::LoadFile(config_path);

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

bool PnpSolver::Islarge(Armor &armor)
{
    if (armor.car_num == 1)
    {
        armor.islarge = true;
        return true;
    }

    armor.islarge = false;
    return false;
}

void PnpSolver::set_R_gimbal2world(const Eigen::Quaterniond &q)
{
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

bool PnpSolver::_solve_pnp(Armor &armor)
{
    if (armor.left_lightbar.center == cv::Point2f(0, 0) || armor.right_lightbar.center == cv::Point2f(0, 0))
    {
        return false;
    }

    const auto &object_points = Islarge(armor) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    armor.corners = {
        armor.left_lightbar.top,
        armor.right_lightbar.top,
        armor.right_lightbar.bottom,
        armor.left_lightbar.bottom,
    };

    if (armor.corners.size() != 4)
    {
        return false;
    }

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(object_points, armor.corners, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    if (!success)
    {
        return false;
    }

    armor.p_camera = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    armor.p_gimbal = R_camera2gimbal_ * armor.p_camera + t_camera2gimbal_;
    armor.p_world = R_gimbal2world_ * armor.p_gimbal;

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);

    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;

    armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
    armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);
    armor.ypd_in_world = tools::xyz2ypd(armor.p_world);

    optimize_yaw(armor);
    return true;
}

int PnpSolver::solveArmorArray(ArmorArray &armor_array)
{
    int success_count = 0;
    for (size_t i = 0; i < armor_array.size(); ++i)
    {
        if (_solve_pnp(armor_array[i]))
        {
            success_count++;
        }
    }
    return success_count;
}

void PnpSolver::optimize_yaw(Armor &armor) const
{
    Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);
    double yaw_center = tools::limit_rad(gimbal_ypr[0]);

    constexpr double SEARCH_RANGE_DEG = 120.0;
    constexpr double COARSE_STEP_DEG = 4.0;
    constexpr double FINE_SEARCH_RANGE_DEG = 6.0;

    double min_error = 1e18;
    double best_yaw = yaw_center;

    auto calc_error = [&](double yaw_rad) -> double {
        double yaw_norm = tools::limit_rad(yaw_rad);
        double inclined = yaw_rad - yaw_center;
        return armor_reprojection_error(armor, yaw_norm, inclined);
    };

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

    double range_rad = FINE_SEARCH_RANGE_DEG * CV_PI / 180.0;
    double a = best_yaw - range_rad;
    double b = best_yaw + range_rad;

    constexpr double PHI = 0.618033988749895;
    double c = b - PHI * (b - a);
    double d = a + PHI * (b - a);

    double error_c = calc_error(c);
    double error_d = calc_error(d);

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

    armor.ypr_in_world[0] = tools::limit_rad((a + b) / 2.0);
}

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
        Eigen::Vector2d ref_d = refs[p] - refs[i];
        Eigen::Vector2d pt_d = pts[p] - pts[i];

        double pixel_dis =
            (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) + std::fabs(ref_d.norm() - pt_d.norm())) / ref_d.norm();
        double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();

        double cost_i = tools::square(pixel_dis * std::sin(inclined)) + tools::square(angular_dis * std::cos(inclined)) * 2.0;
        cost += std::sqrt(cost_i);
    }
    return cost;
}

double PnpSolver::armor_reprojection_error(const Armor &armor, double yaw, const double &inclined) const
{
    auto image_points = reproject_armor(armor.p_world, yaw, armor.islarge);
    return SJTU_cost(image_points, armor.corners, inclined);
}

std::vector<cv::Point2f> PnpSolver::reproject_armor(const Eigen::Vector3d &p_world, double yaw, bool islarge) const
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

    const Eigen::Vector3d &t_armor2world = p_world;
    Eigen::Matrix3d R_armor2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
    Eigen::Vector3d t_armor2camera = R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    cv::Vec3d rvec;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, rvec);
    cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    std::vector<cv::Point2f> image_points;
    const auto &object_points = islarge ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
    return image_points;
}

}  // namespace armor_task
