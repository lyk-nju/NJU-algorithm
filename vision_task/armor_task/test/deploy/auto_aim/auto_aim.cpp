#include "tasks/auto_aim_system.hpp"
#include "tools/pharser.hpp"

#include <rclcpp/rclcpp.hpp>

using namespace armor_task;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::string test_config_path = "../config/deploy_test.yaml";
    bool use_direct_camera = false;

    const TestConfig test_config = load_deploy_test_config(test_config_path);
    AutoAimSystem system(test_config.yolo_model_path,test_config.config_path,test_config.bullet_speed);
    system.enableDirectCameraInput(use_direct_camera);

    system.start();
    system.run();
    system.stop();

    rclcpp::shutdown();
    return 0;
}

