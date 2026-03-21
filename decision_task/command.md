# Decision Task 命令说明

## 环境准备

```bash
# 在每个新终端中需要先执行
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/home/nvidia/NJU-algorithm/decision_task/install/decision:$AMENT_PREFIX_PATH
```

## 编译

```bash
cd /home/nvidia/NJU-algorithm/decision_task
colcon build
```

## 启动节点

### 1. pose_initializer_node (初始位姿发布)

```bash
# 使用 launch 文件（自动加载 config/pose_init.yaml）
ros2 launch decision pose_init.launch.py

# 使用 ros2 run + 参数文件
ros2 run decision pose_initializer_node --ros-args --params-file /home/nvidia/NJU-algorithm/decision_task/config/pose_init.yaml

# 运行时自定义参数
ros2 run decision pose_initializer_node --ros-args -p initial_pose.x:=2.0 -p initial_pose.y:=1.5
```

### 2. patrol_node (自动巡检 - 基于 RobotDecisionSys)

```bash
# 使用 launch 文件（基于 RobotDecisionSys，复用现有决策逻辑）
ros2 launch decision patrol.launch.py

# 直接运行
/home/nvidia/NJU-algorithm/decision_task/build/decision/patrol_node --ros-args --params-file /home/nvidia/NJU-algorithm/decision_task/config/patrol.yaml
```

### 3. decision_node (哨兵完整决策系统)

```bash
ros2 launch decision decision.launch.py
```

或直接运行：

```bash
ros2 run decision decision_node --ros-args --params-file /home/nvidia/NJU-algorithm/decision_task/config/decision.yaml
```

## 配置文件说明

### pose_init.yaml
```yaml
/pose_initializer_node:
  ros__parameters:
    nav_namespace: ""       # 命名空间，如 "red_standard_robot1"
    initial_pose:          # 初始位姿
      x: 1.0
      y: 1.0
      theta: 1.57         # 弧度
    delay: 1.0             # 延迟发布时间(秒)
```

### patrol.yaml
```yaml
/patrol_node:
  ros__parameters:
    nav_namespace: ""                    # 命名空间
    waypoints_path: "waypoints_test.json"   # 路径点文件
    decisions_path: "decisions_test.json"   # 决策文件
    map_path: "RMUL.png"
    distance_thr: 1.0                    # 到达判定距离(米)
    seek_thr: 5.0
    real_width: 12.0
    real_height: 8.0
    step_distance: 0.1
    car_seek_fov: 70.0
    use_pure_patrol: true
```

### decision.yaml (哨兵决策系统)
```yaml
/decision_node:
  ros__parameters:
    nav_namespace: ""
    waypoints_path: "waypoints.json"     # 路径点文件
    decisions_path: "decisions.json"     # 决策文件
    map_path: "RMUL.png"
    distance_thr: 1.0
    seek_thr: 5.0
    real_width: 12.0
    real_height: 8.0
    step_distance: 0.1
    car_seek_fov: 70.0
```

### 配置文件格式说明

#### waypoints_test.json
- 路径点定义，包含 id, name, x, y, angle, connect(连接的其他点)
- 示例：A(0) → B(1) → C(2) → A(0)

#### decisions_test.json
- 决策定义，包含当前点 id、目标点 id、是否连续
- 根据当前所在路径点决定下一个目标点

## 直接运行可执行文件

如果 ros2 run 找不到包，可以直接运行编译出的可执行文件：

```bash
# pose_initializer_node
/home/nvidia/NJU-algorithm/decision_task/build/decision/pose_initializer_node --ros-args --params-file /home/nvidia/NJU-algorithm/decision_task/config/pose_init.yaml

# patrol_node
/home/nvidia/NJU-algorithm/decision_task/build/decision/patrol_node --ros-args --params-file /home/nvidia/NJU-algorithm/decision_task/config/patrol.yaml
```

## 常见问题

### Package 'decision' not found

确保设置了正确的 AMENT_PREFIX_PATH：
```bash
export AMENT_PREFIX_PATH=/home/nvidia/NJU-algorithm/decision_task/install/decision:$AMENT_PREFIX_PATH
```



# 发送位姿
/home/nvidia/NJU-algorithm/decision_task/build/decision/pose_initializer_node --ros-args --params-file /home/nvidia/NJU-algorithm/decision_task/config/pose_init.yaml

# 