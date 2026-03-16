#include "pharser.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

std::pair<std::string, std::string> load_ports_from_config(const std::string &config_path)
{
    std::string send_port = "/dev/ttyUSB0";
    std::string receive_port = "/dev/ttyUSB1";

    try
    {
        struct stat buffer;
        if (stat(config_path.c_str(), &buffer) == 0)
        {
            YAML::Node config = YAML::LoadFile(config_path);
            if (config["send_port"] && !config["send_port"].as<std::string>().empty())
            {
                send_port = config["send_port"].as<std::string>();
            }
            if (config["receive_port"] && !config["receive_port"].as<std::string>().empty())
            {
                receive_port = config["receive_port"].as<std::string>();
            }
            std::cout << "Loaded ports from config: " << send_port << " (send), " << receive_port << " (receive)" << std::endl;
        }
        else
        {
            std::cout << "Config file not found, using default ports" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Warning: Failed to load config: " << e.what() << ", using default ports" << std::endl;
    }

    return {send_port, receive_port};
}

// 从YAML配置文件加载相机参数
std::pair<cv::Mat, cv::Mat> loadCameraParameters(const std::string &config_path)
{
    cv::Mat camera_matrix;
    cv::Mat distort_coeffs;

    try
    {
        YAML::Node config = YAML::LoadFile(config_path);

        if (config["camera_matrix"])
        {
            std::vector<double> cam_params = config["camera_matrix"].as<std::vector<double>>();
            camera_matrix = (cv::Mat_<double>(3, 3) << cam_params[0], cam_params[1], cam_params[2], cam_params[3], cam_params[4], cam_params[5], cam_params[6], cam_params[7], cam_params[8]);
        }

        if (config["distort_coeffs"])
        {
            std::vector<double> dist_params = config["distort_coeffs"].as<std::vector<double>>();
            distort_coeffs = (cv::Mat_<double>(1, 5) << dist_params[0], dist_params[1], dist_params[2], dist_params[3], dist_params[4]);
        }

        // std::cout << "Camera parameters loaded from: " << config_path << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading camera parameters from " << config_path << ": " << e.what() << std::endl;
        camera_matrix = (cv::Mat_<double>(3, 3) << 610, 0, 320, 0, 613, 240, 0, 0, 1);
        distort_coeffs = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
        std::cout << "Using default camera parameters" << std::endl;
    }

    return std::make_pair(camera_matrix, distort_coeffs);
}

// 从配置文件加载 deploy 测试配置
TestConfig load_deploy_test_config(const std::string &config_path)
{
    TestConfig config;
    
    // 设置默认值
    config.yolo_model_path = "../models/yolov8_armor.onnx";
    config.video_path = "";
    config.config_path = "../config/demo.yaml";
    config.bullet_speed = 24.0;
    config.send_port = "/dev/ttyUSB0";
    config.receive_port = "/dev/ttyUSB1";

    try
    {
        struct stat buffer;
        if (stat(config_path.c_str(), &buffer) == 0)
        {
            YAML::Node yaml = YAML::LoadFile(config_path);
            
            if (yaml["yolo_model_path"] && !yaml["yolo_model_path"].as<std::string>().empty())
            {
                config.yolo_model_path = yaml["yolo_model_path"].as<std::string>();
            }
            
            if (yaml["video_path"] && !yaml["video_path"].as<std::string>().empty())
            {
                config.video_path = yaml["video_path"].as<std::string>();
            }
            
            if (yaml["config_path"] && !yaml["config_path"].as<std::string>().empty())
            {
                config.config_path = yaml["config_path"].as<std::string>();
            }
            
            if (yaml["bullet_speed"])
            {
                config.bullet_speed = yaml["bullet_speed"].as<double>();
            }
            
            if (yaml["send_port"] && !yaml["send_port"].as<std::string>().empty())
            {
                config.send_port = yaml["send_port"].as<std::string>();
            }
            
            if (yaml["receive_port"] && !yaml["receive_port"].as<std::string>().empty())
            {
                config.receive_port = yaml["receive_port"].as<std::string>();
            }
            
            std::cout << "Loaded deploy test config from: " << config_path << std::endl;
            std::cout << "  YOLO model: " << config.yolo_model_path << std::endl;
            std::cout << "  Video path: " << config.video_path << std::endl;
            std::cout << "  Config path: " << config.config_path << std::endl;
            std::cout << "  Bullet speed: " << config.bullet_speed << " m/s" << std::endl;
            std::cout << "  Send port: " << config.send_port << std::endl;
            std::cout << "  Receive port: " << config.receive_port << std::endl;
        }
        else
        {
            std::cout << "Config file not found: " << config_path << ", using default values" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Warning: Failed to load deploy test config from " << config_path << ": " << e.what() << ", using default values" << std::endl;
    }

    return config;
}

// 从配置文件加载 video 测试配置
TestConfig load_video_test_config(const std::string &config_path)
{
    TestConfig config;
    
    // 设置默认值
    config.yolo_model_path = "../models/best.onnx";
    config.video_path = "../assets/circular.avi";
    config.config_path = "../config/demo.yaml";
    config.bullet_speed = 24.0;
    config.send_port = "";  // video 测试不需要串口
    config.receive_port = "";

    try
    {
        struct stat buffer;
        if (stat(config_path.c_str(), &buffer) == 0)
        {
            YAML::Node yaml = YAML::LoadFile(config_path);
            
            if (yaml["yolo_model_path"] && !yaml["yolo_model_path"].as<std::string>().empty())
            {
                config.yolo_model_path = yaml["yolo_model_path"].as<std::string>();
            }
            
            if (yaml["video_path"] && !yaml["video_path"].as<std::string>().empty())
            {
                config.video_path = yaml["video_path"].as<std::string>();
            }
            
            if (yaml["config_path"] && !yaml["config_path"].as<std::string>().empty())
            {
                config.config_path = yaml["config_path"].as<std::string>();
            }
            
            if (yaml["bullet_speed"])
            {
                config.bullet_speed = yaml["bullet_speed"].as<double>();
            }
            
            std::cout << "Loaded video test config from: " << config_path << std::endl;
            std::cout << "  YOLO model: " << config.yolo_model_path << std::endl;
            std::cout << "  Video path: " << config.video_path << std::endl;
            std::cout << "  Config path: " << config.config_path << std::endl;
            std::cout << "  Bullet speed: " << config.bullet_speed << " m/s" << std::endl;
        }
        else
        {
            std::cout << "Config file not found: " << config_path << ", using default values" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Warning: Failed to load video test config from " << config_path << ": " << e.what() << ", using default values" << std::endl;
    }

    return config;
}