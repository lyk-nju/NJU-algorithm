cmake_minimum_required(VERSION 3.10)
project(ArmorTask)
enable_language(CUDA)
add_subdirectory(io/serial)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "/usr/lib/x86_64-linux-gnu:${CMAKE_INSTALL_RPATH}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "OpenCV library dirs: ${OpenCV_LIB_PATH}")

set(cv_bridge_DIR "/usr/local/share/cv_bridge/cmake")
list(INSERT CMAKE_PREFIX_PATH 0 "/usr/local")

find_package(cv_bridge REQUIRED)
message(STATUS "Using cv_bridge from: ${cv_bridge_DIR}")

find_package(Eigen3 REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(spdlog REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(cv_bridge_FOUND)
    message(STATUS "cv_bridge found")
    message(STATUS "cv_bridge_INCLUDE_DIRS: ${cv_bridge_INCLUDE_DIRS}")
    message(STATUS "cv_bridge_LIBRARIES: ${cv_bridge_LIBRARIES}")
endif()

include_directories(/opt/ros/humble/include/rcl_interfaces/rcl_interfaces)
include_directories(/opt/ros/humble/include/rclcpp)
include_directories(/opt/ros/humble/include/rosidl_typesupport)
include_directories(/opt/ros/humble/include/rcutils)
include_directories(/opt/ros/humble/include/cv_bridge)
include_directories(/usr/local/include/cv_bridge)
include_directories(/home/user/vision_opencv/cv_bridge/include)
include_directories(/usr/local/cuda/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(HIK_SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/io/camera/hik/lib/arm64)
else()
    set(HIK_SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/io/camera/hik/lib/amd64)
endif()
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tasks
    ${CMAKE_CURRENT_SOURCE_DIR}/tools
    ${PROJECT_SOURCE_DIR}/tools
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${CMAKE_CURRENT_SOURCE_DIR}/test/utils
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/io
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${rclcpp_INCLUDE_DIRS}
)

if(visualization_msgs_FOUND)
    if(DEFINED visualization_msgs_INCLUDE_DIRS)
        include_directories(${visualization_msgs_INCLUDE_DIRS})
        message(STATUS "visualization_msgs_INCLUDE_DIRS: ${visualization_msgs_INCLUDE_DIRS}")
    else()
        set(ROS_DISTRO $ENV{ROS_DISTRO})
        if(NOT ROS_DISTRO)
            set(ROS_DISTRO "jazzy")
        endif()
        include_directories(/opt/ros/${ROS_DISTRO}/include)
        message(STATUS "Added ROS2 include directory: /opt/ros/${ROS_DISTRO}/include")
    endif()
endif()


set(SOURCES
    tasks/postprocess.cu
    tasks/auto_aim_system.cpp
    tasks/detector.cpp
    tasks/pnp_solver.cpp
    tasks/tracker.cpp
    tasks/ekf.cpp
    tasks/aimer.cpp
    tasks/target.cpp
    tasks/trajectory_normal.cpp
    tools/math_tools.cpp  
    tools/draw.cpp
    tools/pharser.cpp
    tools/visualizer.cpp
    io/usb/cboard.cpp
    io/usb/unpack.cpp
    cuda/yolo_decode.cu
)

set(SOURCES_NO_VISUALIZER
    tasks/auto_aim_system.cpp
    tasks/detector.cpp
    tasks/pnp_solver.cpp
    tasks/tracker.cpp
    tasks/ekf.cpp
    tasks/aimer.cpp
    tasks/target.cpp
    tasks/trajectory_normal.cpp
    tools/math_tools.cpp  
    tools/draw.cpp
    tools/pharser.cpp
    io/usb/cboard.cpp
    io/usb/unpack.cpp
    cuda/yolo_decode.cu
    tasks/postprocess.cu
)

set(ROS2_SOURCES
    io/ros2_manager.cpp
)


add_executable(video_test test/video/video_test.cpp ${SOURCES})
 add_executable(auto_aim_test test/deploy/auto_aim/auto_aim.cpp ${SOURCES} ${ROS2_SOURCES})
add_executable(camera_test
    test/scripts/camera_test.cpp
    io/camera/camera_base.cpp
    io/camera/hik/hik.cpp
)



target_link_libraries(video_test 
    ${OpenCV_LIBS} 
    ${EIGEN3_LIBRARIES} 
    yaml-cpp 
    spdlog::spdlog 
    pthread 
    rclcpp::rclcpp
    visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
    nvinfer
    cudart
    -L/usr/local/cuda/lib64
)


target_link_directories(camera_test PRIVATE ${HIK_SDK_LIB_DIR})
target_link_libraries(camera_test
    ${OpenCV_LIBS}
    yaml-cpp
    pthread
    usb-1.0
    MvCameraControl
    rclcpp::rclcpp
    cv_bridge::cv_bridge
)

target_link_libraries(auto_aim_test 
    ${OpenCV_LIBS} 
    ${EIGEN3_LIBRARIES} 
    yaml-cpp 
    spdlog::spdlog 
    pthread 
    rclcpp::rclcpp
    cv_bridge::cv_bridge
    visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
    nvinfer
    cudart
    -L/usr/local/cuda/lib64
    serial
)

set_target_properties(
    video_test 
    auto_aim_test 
    camera_test
    PROPERTIES
        BUILD_RPATH "/usr/lib/x86_64-linux-gnu;${HIK_SDK_LIB_DIR}"
        INSTALL_RPATH "/usr/lib/x86_64-linux-gnu;${HIK_SDK_LIB_DIR}"
        LINK_FLAGS "-Wl,--disable-new-dtags"
)

target_compile_options(video_test PRIVATE -Wall -Wextra)
target_compile_options(auto_aim_test PRIVATE -Wall -Wextra)
target_compile_options(camera_test PRIVATE -Wall -Wextra)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(video_test PRIVATE -g)
    target_compile_options(auto_aim_test PRIVATE -g)
    target_compile_options(camera_test PRIVATE -g)
endif()

add_custom_command(TARGET video_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "#!/bin/bash" > ${CMAKE_BINARY_DIR}/run_video_test.sh
    COMMAND ${CMAKE_COMMAND} -E echo "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6" >> ${CMAKE_BINARY_DIR}/run_video_test.sh
    COMMAND ${CMAKE_COMMAND} -E echo "exec ${CMAKE_BINARY_DIR}/video_test \"$$@\"" >> ${CMAKE_BINARY_DIR}/run_video_test.sh
    COMMAND chmod +x ${CMAKE_BINARY_DIR}/run_video_test.sh
    COMMENT "Creating wrapper script for video_test with LD_PRELOAD"
)
