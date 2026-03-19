# ROS2 启动指令

## 编译并启动 (Mapping 模式)

```bash
# 重新编译
cd /home/nvidia/NJU-algorithm/navigation_task
colcon build --packages-select fast_lio rm_nav_bringup rm_navigation

# 启动
source install/setup.bash && ros2 launch rm_nav_bringup bringup_real.launch.py world:=my_lab_world mode:=mapping lio:=fastlio lio_rviz:=False nav_rviz:=False
```

## 保存地图

RVIZ可视化界面操作 Panels -> Add New Panel -> slam_toolbox -> SlamToolboxPlugun -> Save Map & Serialize Map

## 使用保存的地图 (Localization 模式)

```bash
# 重新编译
cd /home/nvidia/NJU-algorithm/navigation_task
colcon build --packages-select fast_lio rm_nav_bringup rm_navigation

# 启动
source /home/nvidia/NJU-algorithm/navigation_task/install/setup.bash
ros2 launch rm_nav_bringup bringup_real.launch.py world:=my_lab_map mode:=nav lio:=fastlio localization:=amcl lio_rviz:=False nav_rviz:=False
```

## 参数配置说明

- `world`: 地图名称 (my_lab_world)
- `mode`: 运行模式 (mapping / nav)
- `lio`: 激光里程计算法 (fastlio)
- `lio_rviz`: 是否显示 LIO 可视化 (True/False)
- `nav_rviz`: 是否显示 Nav2 可视化 (True/False)
- `localization`: 定位模式 (slam_toolbox / amcl / icp)
