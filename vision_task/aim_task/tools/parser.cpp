#include "parser.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <sys/stat.h>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <serial/serial.h>

// 自动检测可用的 ttyACM* 串口设备
std::string detect_serial_port()
{
    const auto ports = serial::list_ports();
    if (ports.empty())
    {
        std::cerr << "Warning: No serial port found" << std::endl;
        return "";
    }

    auto pick_port = [&](const std::string &keyword) -> std::string
    {
        auto it = std::find_if(ports.begin(), ports.end(), [&](const serial::PortInfo &info)
                               { return info.port.find(keyword) != std::string::npos; });
        return it == ports.end() ? "" : it->port;
    };

    std::string selected_port = pick_port("ttyACM");
    if (selected_port.empty())
    {
        selected_port = pick_port("ttyUSB");
    }
    if (selected_port.empty())
    {
        selected_port = ports.front().port;
    }

    std::cout << "Auto-detected serial port: " << selected_port << std::endl;
    return selected_port;
}

// 检查端口是否需要自动检测（为空或包?"*"?
bool needs_auto_detect(const std::string& port)
{
    return port.empty() || port == "*" || 
           port.find("*") != std::string::npos || 
           port.find("auto") != std::string::npos;
}

std::pair<std::string, std::string> load_ports_from_config(const std::string &config_path)
{
    std::string send_port = "/dev/ttyACM0";
    std::string receive_port = "/dev/ttyACM0";

    try
    {
        struct stat buffer;
        if (stat(config_path.c_str(), &buffer) == 0)
        {
            YAML::Node config = YAML::LoadFile(config_path);
            if (config["send_port"])
            {
                std::string configured_port = config["send_port"].as<std::string>();
                if (needs_auto_detect(configured_port))
                {
                    send_port = detect_serial_port();
                }
                else if (!configured_port.empty())
                {
                    send_port = configured_port;
                }
            }
            if (config["receive_port"])
            {
                std::string configured_port = config["receive_port"].as<std::string>();
                if (needs_auto_detect(configured_port))
                {
                    receive_port = detect_serial_port();
                }
                else if (!configured_port.empty())
                {
                    receive_port = configured_port;
                }
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

// 从配置文件加载deploy 测试配置
TestConfig load_deploy_test_config(const std::string &config_path)
{
    TestConfig config;
    
    // 设置默认
    config.yolo_model_path = "../models/best.onnx";
    config.video_path = "";
    config.config_path = "../config/demo.yaml";
    config.bullet_speed = 24.0;
    config.playback_fps = 10;
    config.send_port = "/dev/ttyACM0";
    config.receive_port = "/dev/ttyACM1";
    config.image_source = "ros2";

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
            if (yaml["playback_fps"])
            {
                config.playback_fps = yaml["playback_fps"].as<int>();
            }
            else if (yaml["max_frames"])
            {
                // Backward compatibility: previously misunderstood as frame count.
                config.playback_fps = yaml["max_frames"].as<int>();
            }
            
            if (yaml["send_port"])
            {
                std::string configured_port = yaml["send_port"].as<std::string>();
                if (needs_auto_detect(configured_port))
                {
                    config.send_port = detect_serial_port();
                }
                else
                {
                    config.send_port = configured_port;
                }
            }
            
            if (yaml["receive_port"])
            {
                std::string configured_port = yaml["receive_port"].as<std::string>();
                if (needs_auto_detect(configured_port))
                {
                    config.receive_port = detect_serial_port();
                }
                else
                {
                    config.receive_port = configured_port;
                }
            }

            if (yaml["image_source"])
            {
                config.image_source = yaml["image_source"].as<std::string>();
            }

            std::cout << "Loaded deploy test config from: " << config_path << std::endl;
            std::cout << "  YOLO model: " << config.yolo_model_path << std::endl;
            std::cout << "  Video path: " << config.video_path << std::endl;
            std::cout << "  Config path: " << config.config_path << std::endl;
            std::cout << "  Bullet speed: " << config.bullet_speed << " m/s" << std::endl;
            std::cout << "  Playback FPS: " << config.playback_fps << std::endl;
            std::cout << "  Send port: " << config.send_port << std::endl;
            std::cout << "  Receive port: " << config.receive_port << std::endl;
            std::cout << "  Image source: " << config.image_source << std::endl;
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

// 从配置文件加载video 测试配置
TestConfig load_video_test_config(const std::string &config_path)
{
    TestConfig config;
    
    // 设置默认?
    config.yolo_model_path = "../models/best.engine";
    config.video_path = "../assets/circular.avi";
    config.config_path = "../config/demo.yaml";
    config.bullet_speed = 24.0;
    config.playback_fps = 10;
    config.send_port = "";  // video 测试不需要串?
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
            if (yaml["playback_fps"])
            {
                config.playback_fps = yaml["playback_fps"].as<int>();
            }
            else if (yaml["max_frames"])
            {
                // Backward compatibility: previously misunderstood as frame count.
                config.playback_fps = yaml["max_frames"].as<int>();
            }
            
            std::cout << "Loaded video test config from: " << config_path << std::endl;
            std::cout << "  YOLO model: " << config.yolo_model_path << std::endl;
            std::cout << "  Video path: " << config.video_path << std::endl;
            std::cout << "  Config path: " << config.config_path << std::endl;
            std::cout << "  Bullet speed: " << config.bullet_speed << " m/s" << std::endl;
            std::cout << "  Playback FPS: " << config.playback_fps << std::endl;
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
