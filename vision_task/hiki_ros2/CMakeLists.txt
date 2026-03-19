cmake_minimum_required(VERSION 3.8)
project(hik_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

# ---------------------------------------------------------
# 强制指定使用自定义编译的 cv_bridge 和 OpenCV
# ---------------------------------------------------------
set(cv_bridge_DIR "/usr/local/share/cv_bridge/cmake")
find_package(cv_bridge REQUIRED)

set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

# Include Directories
include_directories(
    src
    hikSDK/include
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${camera_info_manager_INCLUDE_DIRS}
    ${image_transport_INCLUDE_DIRS}
)

# Set Hikvision SDK Library Path
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(HIK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/amd64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(HIK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64")
else()
    message(FATAL_ERROR "Unsupported system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

link_directories(${HIK_LIB_DIR})

# Create Executable
# Note: src/main.cpp includes hik_camera_node.cpp directly
add_executable(hik_camera_node src/main.cpp)

# Use ament_target_dependencies for standard ROS 2 packages
ament_target_dependencies(hik_camera_node
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  camera_info_manager
)

target_link_libraries(hik_camera_node
    # ${rclcpp_LIBRARIES} # Handled by ament_target_dependencies
    # ${sensor_msgs_LIBRARIES}
    # ${image_transport_LIBRARIES}
    # ${camera_info_manager_LIBRARIES}
    ${cv_bridge_LIBRARIES} # cv_bridge
    ${OpenCV_LIBS}
    MvCameraControl
    MvUsb3vTL
    MediaProcess
    FormatConversion
    MVRender
    X11 # Often needed for MVRender if on linux
)

# Installation
install(TARGETS hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()