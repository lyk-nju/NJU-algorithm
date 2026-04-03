cmake_minimum_required(VERSION 3.10)
project(ArmorTask)

# 默认 Release，可通过 -DCMAKE_BUILD_TYPE=Debug 覆盖
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (Release/Debug/RelWithDebInfo/MinSizeRel)" FORCE)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -march=native)
    add_compile_definitions(NDEBUG)
endif()

# TensorRT/CUDA 可选：开发机无 TensorRT 时可关闭以便 CPU-only 编译
option(ARMOR_TASK_WITH_TENSORRT_CUDA "Build with TensorRT/CUDA (NVCC, .cu sources, TensorRT libs)" ON)

# 允许为 NVCC 指定兼容的 host compiler（需在 enable_language(CUDA) 之前设置才生效）
set(ARMOR_TASK_CUDA_HOST_COMPILER "" CACHE STRING "Path to host compiler for NVCC, e.g. /usr/bin/gcc-12")
if(ARMOR_TASK_WITH_TENSORRT_CUDA AND ARMOR_TASK_CUDA_HOST_COMPILER)
    set(CMAKE_CUDA_HOST_COMPILER "${ARMOR_TASK_CUDA_HOST_COMPILER}")
endif()

if(ARMOR_TASK_WITH_TENSORRT_CUDA)
    enable_language(CUDA)
    add_compile_definitions(ARMOR_TASK_WITH_TENSORRT_CUDA=1)
else()
    message(WARNING "ARMOR_TASK_WITH_TENSORRT_CUDA=OFF: build will use OpenCV-DNN CPU path; TensorRT/CUDA kernels are disabled.")
    add_compile_definitions(ARMOR_TASK_WITH_TENSORRT_CUDA=0)
endif()
# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置 RPATH 和链接器选项，优先使用系统的 libstdc++
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# 将系统库路径添加到 RPATH，优先于 Anaconda
set(CMAKE_INSTALL_RPATH "/usr/lib/x86_64-linux-gnu:${CMAKE_INSTALL_RPATH}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "OpenCV library dirs: ${OpenCV_LIB_PATH}")

# 查找 cv_bridge
find_package(cv_bridge REQUIRED)

# 查找 Eigen3w
find_package(Eigen3 REQUIRED)

# 查找 yaml-cpp
find_package(yaml-cpp REQUIRED)

# 查找 spdlog
find_package(spdlog REQUIRED)

# TensorRT/CUDA 链接库（CPU-only 时为空）
set(TRT_CUDA_LIBS "")
if(ARMOR_TASK_WITH_TENSORRT_CUDA)
    list(APPEND TRT_CUDA_LIBS nvinfer cudart)
endif()

# 查找 ROS 2 依赖
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 手动添加 cv_bridge 的包含目录
if(cv_bridge_FOUND)
    message(STATUS "cv_bridge found")
    message(STATUS "cv_bridge_INCLUDE_DIRS: ${cv_bridge_INCLUDE_DIRS}")
    message(STATUS "cv_bridge_LIBRARIES: ${cv_bridge_LIBRARIES}")
endif()

# ROS2 根包含目录（确保可找到 cv_bridge 等头文件）
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(NOT ROS_DISTRO)
    set(ROS_DISTRO "jazzy")
endif()
include_directories(/opt/ros/${ROS_DISTRO}/include)
if(ARMOR_TASK_WITH_TENSORRT_CUDA)
    include_directories(/usr/local/cuda/include)
endif()
# 设置包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tasks
    ${CMAKE_CURRENT_SOURCE_DIR}/tools
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${CMAKE_CURRENT_SOURCE_DIR}/test/utils
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${rclcpp_INCLUDE_DIRS}  # ROS 2 包含目录
)

# 添加 visualization_msgs 的包含目录
# ROS2 消息包的头文件在 /opt/ros/<distro>/include/<package>/<package>/msg/
if(visualization_msgs_FOUND)
    if(DEFINED visualization_msgs_INCLUDE_DIRS)
        include_directories(${visualization_msgs_INCLUDE_DIRS})
        message(STATUS "visualization_msgs_INCLUDE_DIRS: ${visualization_msgs_INCLUDE_DIRS}")
    else()
        # 如果变量未定义，直接添加 ROS2 标准包含路径
        set(ROS_DISTRO $ENV{ROS_DISTRO})
        if(NOT ROS_DISTRO)
            set(ROS_DISTRO "jazzy")
        endif()
        include_directories(/opt/ros/${ROS_DISTRO}/include)
        message(STATUS "Added ROS2 include directory: /opt/ros/${ROS_DISTRO}/include")
    endif()
endif()


# 添加源文件（基础源文件，不包含 ROS2）
set(CORE_SOURCES
    #tasks/main.cpp
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
    tools/parser.cpp
    tools/transfer.cpp
    io/serial_manager.cpp
    io/keyboard_manager.cpp
)

set(SOURCES ${CORE_SOURCES})
if(ARMOR_TASK_WITH_TENSORRT_CUDA)
    list(APPEND SOURCES
        tasks/preprocess.cu
        cuda/yolo_decode.cu
    )
endif()

# 不包含可视化器的源文件（用于不需要 ROS2 的测试）
set(SOURCES_NO_VISUALIZER ${CORE_SOURCES})
list(REMOVE_ITEM SOURCES_NO_VISUALIZER tools/visualizer.cpp)
if(ARMOR_TASK_WITH_TENSORRT_CUDA)
    list(APPEND SOURCES_NO_VISUALIZER
        cuda/yolo_decode.cu
        tasks/preprocess.cu
    )
endif()

# ROS2 相关源文件
set(ROS2_SOURCES
    io/ros2_manager.cpp
)


# 编译可执行文件
# video_test 只需要可视化功能，不需要 ROS2 图像订阅，所以不包含 ros2_manager.cpp
add_executable(video_test test/video/video_test.cpp ${SOURCES})
add_executable(camera_sub_test test/deploy/camera_sub.cpp ${SOURCES} ${ROS2_SOURCES})
add_executable(detector_test test/deploy/detector.cpp ${SOURCES} ${ROS2_SOURCES})
add_executable(pnp_solver_test test/deploy/pnp_solver.cpp ${SOURCES} ${ROS2_SOURCES})
add_executable(target_test test/deploy/target.cpp ${SOURCES} ${ROS2_SOURCES})
add_executable(auto_aimer_test test/deploy/auto_aimer.cpp ${SOURCES} ${ROS2_SOURCES})
add_executable(communication_test test/deploy/communication.cpp ${SOURCES_NO_VISUALIZER})
add_executable(fake_publish test/deploy/fake_publish.cpp)


# 链接 ROS 2 的库
# target_link_libraries(${PROJECT_NAME} rclcpp::rclcpp)

# 链接测试可执行文件（去重）
set(ARMOR_TASK_COMMON_LIBS
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp
    spdlog::spdlog
    pthread
    ${TRT_CUDA_LIBS}
)

function(armor_task_link target)
    target_link_libraries(${target} PRIVATE ${ARMOR_TASK_COMMON_LIBS} ${ARGN})
    if(ARMOR_TASK_WITH_TENSORRT_CUDA)
        target_link_directories(${target} PRIVATE /usr/local/cuda/lib64)
    endif()
endfunction()

armor_task_link(video_test)
armor_task_link(communication_test)

# deploy 测试可执行文件（需要 ROS2 + cv_bridge）
armor_task_link(camera_sub_test rclcpp::rclcpp cv_bridge::cv_bridge visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
armor_task_link(detector_test rclcpp::rclcpp cv_bridge::cv_bridge visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
armor_task_link(pnp_solver_test rclcpp::rclcpp cv_bridge::cv_bridge visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
armor_task_link(target_test rclcpp::rclcpp cv_bridge::cv_bridge visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
armor_task_link(auto_aimer_test rclcpp::rclcpp cv_bridge::cv_bridge visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)

target_link_libraries(fake_publish PRIVATE pthread rclcpp::rclcpp geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)


# 为所有可执行文件设置 RPATH，强制使用系统 libstdc++
set_target_properties(
    video_test 
    camera_sub_test 
    detector_test 
    pnp_solver_test 
    target_test 
    auto_aimer_test 
    communication_test
    fake_publish
    PROPERTIES
        BUILD_RPATH "/usr/lib/x86_64-linux-gnu"
        INSTALL_RPATH "/usr/lib/x86_64-linux-gnu"
        LINK_FLAGS "-Wl,--disable-new-dtags"
)

# 添加编译选项
target_compile_options(video_test PRIVATE -Wall -Wextra)
target_compile_options(camera_sub_test PRIVATE -Wall -Wextra)
target_compile_options(communication_test PRIVATE -Wall -Wextra)
target_compile_options(detector_test PRIVATE -Wall -Wextra)
target_compile_options(pnp_solver_test PRIVATE -Wall -Wextra)
target_compile_options(target_test PRIVATE -Wall -Wextra)
target_compile_options(auto_aimer_test PRIVATE -Wall -Wextra)
target_compile_options(fake_publish PRIVATE -Wall -Wextra)

# 创建一个运行 video_test 的包装脚本
add_custom_command(TARGET video_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "#!/bin/bash" > ${CMAKE_BINARY_DIR}/run_video_test.sh
    COMMAND ${CMAKE_COMMAND} -E echo "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6" >> ${CMAKE_BINARY_DIR}/run_video_test.sh
    COMMAND ${CMAKE_COMMAND} -E echo "exec ${CMAKE_BINARY_DIR}/video_test \"$$@\"" >> ${CMAKE_BINARY_DIR}/run_video_test.sh
    COMMAND chmod +x ${CMAKE_BINARY_DIR}/run_video_test.sh
    COMMENT "Creating wrapper script for video_test with LD_PRELOAD"
)
