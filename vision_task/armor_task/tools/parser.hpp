#ifndef PARSER_HPP
#define PARSER_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <utility>

/**
 * @brief 从配置文件读取串口端口
 * @param config_path 配置文件路径，默认为 "../config/deploy_test.yaml"
 * @return 返回一对字符串，第一个是发送端口，第二个是接收端口
 */
std::pair<std::string, std::string> load_ports_from_config(const std::string &config_path = "../config/deploy_test.yaml");

/**
 * @brief 从YAML配置文件加载相机参数
 * @param config_path 配置文件路径
 * @return 返回一对cv::Mat，第一个是相机内参矩阵，第二个是畸变系数
 */
std::pair<cv::Mat, cv::Mat> loadCameraParameters(const std::string &config_path);

/**
 * @brief 测试配置结构体
 */
struct TestConfig
{
    std::string yolo_model_path;
    std::string video_path;
    std::string config_path;  // demo.yaml 路径
    double bullet_speed;
    std::string send_port;
    std::string receive_port;
};

/**
 * @brief 从配置文件加载测试配置（用于 deploy 测试）
 * @param config_path 配置文件路径，默认为 "../config/deploy_test.yaml"
 * @return TestConfig 结构体
 */
TestConfig load_deploy_test_config(const std::string &config_path = "../config/deploy_test.yaml");

/**
 * @brief 从配置文件加载测试配置（用于 video 测试）
 * @param config_path 配置文件路径，默认为 "../config/video_test.yaml"
 * @return TestConfig 结构体（不包含串口端口）
 */
TestConfig load_video_test_config(const std::string &config_path = "../config/video_test.yaml");

#endif // PARSER_HPP

