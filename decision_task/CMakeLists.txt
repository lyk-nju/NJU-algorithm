cmake_minimum_required(VERSION 3.8)
project(decision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json 3.2 QUIET CONFIG)
if(NOT nlohmann_json_FOUND)
  find_package(nlohmann_json CONFIG QUIET)
endif()
if(NOT nlohmann_json_FOUND)
  message(FATAL_ERROR "nlohmann_json not found. Install: sudo apt install nlohmann-json3-dev")
endif()

include_directories(include)

add_executable(decision_node
  src/decision_node.cpp
  src/robot_decision.cpp
)

target_include_directories(decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(decision_node
  rclcpp
  rclcpp_action
  action_msgs
  std_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(decision_node
  ${OpenCV_LIBS}
)

target_link_libraries(decision_node nlohmann_json::nlohmann_json)

# Pose Initializer Node
add_executable(pose_initializer_node
  src/pose_initializer_node.cpp
)

ament_target_dependencies(pose_initializer_node
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Patrol Node (基于 RobotDecisionSys)
add_executable(patrol_node
  src/patrol_node.cpp
  src/robot_decision.cpp
)

ament_target_dependencies(patrol_node
  rclcpp
  rclcpp_action
  action_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(patrol_node
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
)

# Patrol Sim Node (订阅 /autoaim_data，has_enemy=false 时巡逻，has_enemy=true 时停止)
add_executable(patrol_sim_node
  src/patrol_sim_node.cpp
  src/robot_decision.cpp
)

ament_target_dependencies(patrol_sim_node
  rclcpp
  rclcpp_action
  action_msgs
  std_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(patrol_sim_node
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
)

install(TARGETS
  decision_node
  pose_initializer_node
  patrol_node
  patrol_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# 安装 config，便于 install 后通过 config_file 参数加载
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
