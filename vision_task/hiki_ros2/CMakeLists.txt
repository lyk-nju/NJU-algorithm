cmake_minimum_required(VERSION 3.10)
project(hik_camera_calibration_standalone)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 包含头文件
include_directories(
    hikSDK/include
    ${OpenCV_INCLUDE_DIRS}
)

# 设置库路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    link_directories(hikSDK/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    link_directories(hikSDK/lib/arm64)
else()
    message(FATAL_ERROR "Unsupported system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 创建可执行文件
add_executable(${PROJECT_NAME} 
    hik_camera_calibration_standalone.cpp
)

# 创建GUI版本
add_executable(hik_camera_calibration_gui
    hik_camera_calibration_gui.cpp
)

# 链接库
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    FormatConversion
    MediaProcess
    MvCameraControl
    MVRender
    MvUsb3vTL
)

# 链接GUI版本的库
target_link_libraries(hik_camera_calibration_gui
    ${OpenCV_LIBS}
    FormatConversion
    MediaProcess
    MvCameraControl
    MVRender
    MvUsb3vTL
)

# 设置输出目录
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

set_target_properties(hik_camera_calibration_gui PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# 打印信息
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")