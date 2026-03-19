#!/bin/bash

# 定义退出清理逻辑
trap "kill 0" EXIT

echo "--- 正在并行启动 RoboMaster 视觉任务 ---"

# 1. 启动相机驱动
# 使用 (cd ... && ...) & 的方式，不会改变主脚本的路径
(cd /home/nvidia/NJU-algorithm/vision_task/hiki_ros2 && bash start_camera.sh) &

# 等待相机启动
sleep 2

# 2. 启动自瞄主程序
# 进入 build 目录启动，这样程序才能根据 ../config 找到配置文件
(cd /home/nvidia/NJU-algorithm/vision_task/armor_task/build && ./auto_aimer_test) &

# # 3. 启动模拟发布
# (cd /home/nvidia/NJU-algorithm/vision_task/armor_task/build && ./fake_publish) &

echo "--- 所有程序已在各自路径下并行启动 ---"

wait