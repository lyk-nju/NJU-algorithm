# 导航任务命令

## 编译

```bash
cd /home/nvidia/NJU-algorithm/navigation_task
colcon build --packages-select fast_lio rm_nav_bringup rm_navigation
```

## 建图模式 (Mapping)

```bash
source install/setup.bash
ros2 launch rm_nav_bringup bringup_real.launch.py world:=my_lab_world mode:=mapping lio:=fastlio lio_rviz:=False nav_rviz:=False
```

### 保存地图

在 RVIZ 可视化界面操作：
- Panels → Add New Panel → slam_toolbox → SlamToolboxPlugin
- Save Map & Serialize Map

将pgm文件下载到PC端
gimp map.pgm

同时重命名.pgm、.data、.poesgraph、.yaml，并且修改yaml里面的 image:文件名



## 导航模式 (Localization)

```bash
cd /home/nvidia/NJU-algorithm/navigation_task
source install/setup.bash
ros2 launch rm_nav_bringup bringup_real.launch.py world:=new_map mode:=nav lio:=fastlio localization:=slam_toolbox lio_rviz:=False nav_rviz:=False
```
ros2 launch rm_nav_bringup bringup_real.launch.py world:=new_map mode:=nav lio:=fastlio localization:=amcl lio_rviz:=False nav_rviz:=False
```

## 模拟器模式

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py
```

## 参数说明

| 参数 | 选项 | 说明 |
|------|------|------|
| world | my_lab_world, my_lab_map, RMUL | 地图名称 |
| mode | nav, mapping | 导航/建图模式 |
| lio | fastlio, pointlio | 激光里程计算法 |
| localization | amcl, slam_toolbox, icp | 定位算法 |
| lio_rviz | True, False | 是否显示 LIO 可视化 |
| nav_rviz | True, False | 是否显示 Nav2 可视化 |