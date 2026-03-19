#include "rclcpp/rclcpp.hpp"
#include "hik_camera_node.cpp" 

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<hik_camera::HikCameraNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}