# RoboMaster 自动瞄准系统 (Armor Task)

## 📖 项目简介

本项目是 RoboMaster 机器人的自动瞄准系统，实现了从图像检测到弹道预测的完整视觉识别与瞄准流程。系统使用深度学习模型进行装甲板检测，结合 PnP 位姿解算、目标追踪、弹道预测等算法，实现高精度的自动瞄准功能。

## 📁 项目结构

```
armor_task/
├── tasks/                  # 核心算法模块
│   ├── detector.cpp/hpp    # 装甲板检测器（YOLO + 传统视觉）
│   ├── pnp_solver.cpp/hpp  # PnP 位姿解算器
│   ├── tracker.cpp/hpp     # 目标追踪器
│   ├── target.cpp/hpp      # 目标管理（EKF 滤波）
│   ├── ekf.cpp/hpp         # 扩展卡尔曼滤波器
│   ├── aimer.cpp/hpp       # 瞄准器（弹道预测与瞄准点计算）
│   ├── trajectory_normal.cpp/hpp  # 简单弹道模型
│   └── trajectory_rk4.cpp/hpp     # RK4 弹道模型
│
├── io/                     # 输入输出模块
│   ├── keyboard_manager.cpp/hpp   # 键盘输入管理（多线程）
│   ├── serial_manager.cpp/hpp     # 串口通信（与下位机通信）
│   └── ros2_manager.cpp/hpp       # ROS2 图像订阅节点
│
├── tools/                  # 工具函数库
│   ├── draw.cpp/hpp        # 可视化绘制函数
│   ├── math_tools.cpp/hpp  # 数学工具函数
│   ├── parser.cpp/hpp      # 配置文件解析
│   └── rviz_visualizer.cpp/hpp    # RViz 可视化
│
├── include/                # 数据结构定义
│   ├── armor.hpp           # 装甲板数据结构
│   ├── car.hpp             # 车辆数据结构
│   └── structures.hpp      # 其他结构体定义
│
├── config/                 # 配置文件
│   ├── camera1.yaml           # 主配置文件（相机参数、弹道参数等）
│   ├── deploy.yaml         # 部署配置（串口端口等）
│   └── path.yaml           # 路径配置
│
├── models/                 # AI 模型文件
│   ├── yolov8_armor.onnx   # YOLO 检测模型
│   └── best.onnx           # 数字识别模型
│
├── test/                   # 测试程序
│   ├── deploy/             # 部署测试（使用 ROS2）
│   │   ├── auto_aimer.cpp  # 完整自动瞄准系统测试
│   │   ├── detector.cpp    # 检测模块测试
│   │   ├── pnp_solver.cpp  # PnP 解算测试
│   │   ├── target.cpp      # 目标追踪测试
│   │   ├── camera_sub.cpp  # 相机订阅测试
│   │   └── communication.cpp # 串口通信测试
│   └── video/              # 视频测试
│       └── video_test.cpp  # 视频文件测试（不使用 ROS2）
│
└── src/                    # 源代码
    └── main.cpp            # 主程序入口（旧版本）
```

## 🔄 项目流程

### 完整的自动瞄准流程

```
图像输入
   ↓
[Step 1] 装甲板检测 (Detector)
   ├── YOLO pose 模型检测候装甲板角点
   └── 输出：ArmorArray (装甲板列表)
   ↓
[Step 2] PnP 位姿解算 (PnpSolver)
   ├── 更新云台姿态（从下位机接收 IMU 四元数）
   ├── 坐标系转换：相机 → 云台 → 世界
   ├── 计算装甲板在世界坐标系下的位置和姿态
   └── 输出：带 3D 坐标和姿态的 ArmorArray
   ↓
[Step 3] 目标追踪 (Tracker)
   ├── 数据关联（匹配当前检测与历史目标）
   ├── EKF 滤波预测目标运动
   └── 输出：Target 列表（包含预测位置）
   ↓
[Step 4] 瞄准计算 (Aimer)
   ├── 选择最优瞄准点
   ├── 弹道预测（考虑飞行时间）
   ├── 迭代计算瞄准角度
   └── 输出：yaw, pitch 瞄准角度
   ↓
发送控制命令给下位机
```

### 坐标系转换流程

系统涉及多个坐标系之间的转换：

1. **相机坐标系** → **云台坐标系**
   - 使用 `R_camera2gimbal` 和 `t_camera2gimbal`（从配置文件加载）

2. **云台坐标系** → **世界坐标系**
   - 使用 `R_gimbal2world`（通过 IMU 四元数实时更新）
   - 在部署测试中，每次 PnP 解算前从下位机接收 IMU 数据

3. **世界坐标系** → **装甲板坐标系**
   - 通过 PnP 解算得到装甲板的位姿

### 关键数据流

```cpp
Armor 结构体包含：
├── 2D 信息：box, center, corners, lightbars
├── 3D 坐标：
│   ├── p_camera    // 相机坐标系
│   ├── p_gimbal    // 云台坐标系
│   └── p_world     // 世界坐标系
├── 姿态信息：
│   ├── ypr_in_gimbal  // 云台坐标系下的 yaw/pitch/roll
│   └── ypr_in_world   // 世界坐标系下的 yaw/pitch/roll
└── 球坐标：ypd_in_world (yaw, pitch, distance)
```

## 🛠️ 核心模块说明

### 1. Detector (检测器)
- **功能**：从图像中检测装甲板
- **方法**：YOLO 深度学习模型 + 传统视觉筛选
- **输出**：装甲板候选列表（2D 位置、灯条信息）

### 2. PnpSolver (位姿解算器)
- **功能**：计算装甲板的 3D 位置和姿态
- **输入**：2D 图像坐标 + 3D 模型点
- **输出**：世界坐标系下的 3D 坐标和姿态
- **关键特性**：
  - 支持实时更新云台姿态（通过 IMU）
  - 包含 yaw 角优化算法
  - 坐标系转换：相机 → 云台 → 世界

### 3. Tracker (追踪器)
- **功能**：多目标追踪和数据关联
- **方法**：匈牙利算法 + EKF 滤波
- **输出**：稳定的目标列表（包含预测位置）

### 4. Target (目标管理)
- **功能**：单个目标的运动模型和状态估计
- **方法**：EKF（扩展卡尔曼滤波）
- **状态**：位置、速度、旋转中心、角速度等

### 5. Aimer (瞄准器)
- **功能**：计算瞄准角度
- **流程**：
  1. 选择最优瞄准点
  2. 弹道预测（考虑子弹飞行时间）
  3. 迭代计算（预测-验证-优化）
- **输出**：yaw, pitch 角度指令

### 6. Trajectory (弹道模型)
- **trajectory_normal**：简单抛物线模型
- **trajectory_rk4**：RK4 数值积分模型（考虑空气阻力）


### 编译步骤

```bash
cd armor_task
mkdir -p build && cd build
cmake ..
make
```

### 可执行文件

编译后会在 `build/` 目录下生成以下可执行文件：

- `video_test` - 视频文件测试（不依赖 ROS2）
- `camera_sub_test` - ROS2 相机订阅测试
- `detector_test` - 检测模块测试
- `pnp_solver_test` - PnP 解算测试
- `target_test` - 目标追踪测试
- `auto_aimer_test` - 完整自动瞄准系统测试
- `communication_test` - 串口通信测试



## 📝 测试程序说明

### deploy/ 目录测试程序

| 测试程序            | 功能说明                                             |
| ------------------- | ---------------------------------------------------- |
| `auto_aimer.cpp`    | 完整的自动瞄准系统，包含检测→PnP→追踪→瞄准的完整流程 |
| `detector.cpp`      | 测试装甲板检测模块，查看检测效果和帧率               |
| `pnp_solver.cpp`    | 测试 PnP 位姿解算，查看坐标转换结果                  |
| `target.cpp`        | 测试目标追踪模块，查看 EKF 滤波效果                  |
| `camera_sub.cpp`    | 测试 ROS2 相机图像订阅                               |
| `communication.cpp` | 测试串口通信，验证与下位机的数据交换                 |

### video/ 目录测试程序

- `video_test.cpp`：使用视频文件进行测试，不依赖 ROS2，适合算法调试

## ⚙️ 配置文件说明

### config/camera1.yaml

主配置文件，包含：

1. **相机标定参数**
   - `camera_matrix`: 相机内参矩阵
   - `distort_coeffs`: 畸变系数
   - `R_camera2gimbal`: 相机到云台的旋转矩阵
   - `t_camera2gimbal`: 相机到云台的平移向量
   - `R_gimbal2imubody`: 云台到 IMU 本体的旋转矩阵

2. **决策参数**
   - `enemy_color`: 敌方颜色（red/blue）
   - `max_match_distance_`: 最大匹配距离
   - `min_detect_count`: 最小检测次数
   - `max_temp_lost_count`: 最大临时丢失次数

3. **弹道参数**
   - `yaw_offset`: yaw 角度偏移量
   - `pitch_offset`: pitch 角度偏移量
   - `high_speed_delay_time`: 高速时的延迟时间
   - `low_speed_delay_time`: 低速时的延迟时间
   - `decision_speed_`: 高速/低速切换阈值

### config/deploy.yaml

部署配置文件，包含：

- 串口端口配置（发送端口、接收端口）

## 🔧 使用说明

### 键盘控制（测试模式）

在测试程序中，可以使用键盘控制：

- `W/S`: 增加/减少 pitch 角度
- `A/D`: 增加/减少 yaw 角度
- `Q`: 退出程序

### 日志输出

测试程序会在 `test/deploy/log/` 目录下生成日志文件：

- `auto_aimer.log` - 自动瞄准系统日志
- `detector.log` - 检测模块日志
- `pnp_solver.log` - PnP 解算日志
- `target.log` - 目标追踪日志

日志记录装甲板检测结果、预测位置、瞄准角度等信息。

### 坐标系说明

- **相机坐标系**：以相机光心为原点，z 轴指向图像前方
- **云台坐标系**：以云台中心为原点，随云台运动
- **世界坐标系**：固定坐标系，通过 IMU 获取云台在世界坐标系中的姿态

**注意**：
- 在视频测试中，相机坐标系 = 世界坐标系（`R_gimbal2world` 为单位矩阵）
- 在部署测试中，需要从下位机接收 IMU 四元数实时更新 `R_gimbal2world`
