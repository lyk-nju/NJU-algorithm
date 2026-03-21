#ifndef ARMOR_HPP
#define ARMOR_HPP

#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
enum Color
{
    red,
    blue,
    purple
};

struct LightBar
{
    cv::Point2f center; // 中心点
    cv::Point2f top;    // 上方点
    cv::Point2f bottom; // 下方点
    Color color;
    cv::Point2f top2bottom;

    LightBar() = default;

    LightBar(const cv::Point2f &top_, const cv::Point2f &bottom_) : top(top_), bottom(bottom_)
    {
        center = (top + bottom) * 0.5f;
        top2bottom = bottom - top;
    }
};

struct Armor
{
    cv::Rect box;                     // 装甲板矩形框（yolo结果检测框）
    int detect_id = -1;               // 唯一检测编号（yolo结果id）
    int car_num = -1;                 // 数字识别编号（ResNet输出）
    float confidence = 0.0f;          // ResNet 分类置信度
    cv::Point2f center;               // 装甲板二维中心
    std::vector<cv::Point2f> corners; // 装甲板四个角点

    LightBar left_lightbar;  // 左灯条
    LightBar right_lightbar; // 右灯条
    Color color;             // 装甲板颜色

    double r = 0.3; // 装甲板旋转半径

    float grade = 0.0f;                                      // 打击评分
    float yaw = 0.0f;                                        // 相机坐标系下的偏航角
    Eigen::Vector3d p_camera = Eigen::Vector3d::Zero();      // PnP 计算的相机坐标系三维坐标
    Eigen::Vector3d p_world = Eigen::Vector3d::Zero();       // PnP 计算的世界坐标系三维坐标
    Eigen::Vector3d p_gimbal = Eigen::Vector3d::Zero();      // PnP 计算的云台坐标系三维坐标
    Eigen::Vector3d ypr_in_gimbal = Eigen::Vector3d::Zero(); // 装甲板云台坐标系欧拉角
    Eigen::Vector3d ypr_in_world = Eigen::Vector3d::Zero();  // 装甲板世界坐标系欧拉角
    Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();  // 球坐标
    bool islarge = false;                                    // 是否为大装甲板

    Armor() = default;
    Armor(const LightBar &left, const LightBar &right, int detect_id, cv::Rect box) : box(box), detect_id(detect_id), left_lightbar(left), right_lightbar(right)
    {
        color = left.color;
        center = (left.center + right.center) / 2;
    };
};

using ArmorArray = std::vector<Armor>;

#endif