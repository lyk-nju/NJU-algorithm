#!/bin/bash

# 相机标定启动脚本

echo "======================================"
echo "相机标定工具"
echo "======================================"
echo ""

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: 未检测到ROS2环境，请先source ROS2环境:"
    echo "  source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "ROS2环境: $ROS_DISTRO"
echo ""

# 使用系统Python而不是Anaconda Python
# ROS2 Humble需要Python 3.10
PYTHON_CMD="/usr/bin/python3"

# 检查系统Python版本
if [ ! -f "$PYTHON_CMD" ]; then
    echo "错误: 找不到系统Python3"
    exit 1
fi

PYTHON_VERSION=$($PYTHON_CMD --version 2>&1 | awk '{print $2}')
echo "使用Python版本: $PYTHON_VERSION ($PYTHON_CMD)"
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 进入脚本目录
cd "$SCRIPT_DIR"

# 检查Python脚本是否存在
if [ ! -f "camera_calibration_node.py" ]; then
    echo "错误: 找不到 camera_calibration_node.py"
    exit 1
fi

# 赋予执行权限
chmod +x camera_calibration_node.py

echo "启动相机标定节点..."
echo "------------------------------------"
echo "操作说明:"
echo "  - 按 C 键: 采集当前图像"
echo "  - 按 Q 键: 完成采集并开始标定"
echo "------------------------------------"
echo ""

# 检查是否使用配置文件
if [ -f "config.yaml" ]; then
    echo "使用配置文件: config.yaml"
    $PYTHON_CMD camera_calibration_node.py --ros-args --params-file config.yaml
else
    echo "使用默认参数"
    $PYTHON_CMD camera_calibration_node.py
fi
