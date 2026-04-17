#include "tasks/auto_aim_system.hpp"
#include "tools/parser.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>

using namespace armor_task;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const std::string test_config_path =
        (argc > 1) ? argv[1] : "../config/deploy_test.yaml";

    const TestConfig cfg = load_deploy_test_config(test_config_path);

    const ImageSourceType source_type =
        (cfg.image_source == "camera")
            ? ImageSourceType::DIRECT_CAMERA
            : ImageSourceType::ROS2_TOPIC;

    std::cout << "[auto_aim] image_source = "
              << (source_type == ImageSourceType::DIRECT_CAMERA ? "camera (USB direct)" : "ros2 (/image_raw)")
              << std::endl;

    AutoAimSystem system(cfg.yolo_model_path, cfg.config_path, cfg.bullet_speed, source_type);
    system.start();
    system.run();
    system.stop();

    rclcpp::shutdown();
    return 0;
}
